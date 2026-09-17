"""Controle do robô diferencial por tópicos de motor.

Um único processo com dois nós que compartilham o comando de roda em memória:

    turtle1/cmd_vel --> car_node --(WheelCommand)--> motor_publisher_node
                                                       |--> my_robot/left_motor
                                                       |--> my_robot/right_motor

car_node assina o Twist e aplica a cinemática inversa do robô diferencial.
motor_publisher_node publica, numa frequência fixa, a velocidade de cada roda
(std_msgs/Float32, em rad/s) que o child script do robô no CoppeliaSim assina.

Com o namespace car_control do launch os tópicos ficam
/car_control/turtle1/cmd_vel, /car_control/my_robot/left_motor e
/car_control/my_robot/right_motor.
"""

import threading   # Lock: protege o comando compartilhado entre os dois nós
import time        # time.monotonic: relógio que nunca volta, ideal para prazos

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class WheelCommand:
    """Última velocidade pedida para cada roda (rad/s) e até quando ela vale."""

    def __init__(self):
        self._lock = threading.Lock()
        self._left = 0.0
        self._right = 0.0
        self._deadline = 0.0

    def set(self, left, right, timeout):
        with self._lock:
            self._left, self._right = left, right
            self._deadline = time.monotonic() + timeout

    def get(self):
        """Devolve (esquerda, direita); zero se o comando expirou."""
        with self._lock:
            if time.monotonic() >= self._deadline:
                return 0.0, 0.0
            return self._left, self._right


class CarNode(Node):
    """Converte turtle1/cmd_vel em velocidades de roda."""

    def __init__(self, command: WheelCommand):
        super().__init__('car_node')
        self.command = command

        self.declare_parameter('wheel_radius', 0.05)      # m
        self.declare_parameter('wheel_separation', 0.2)   # m
        self.declare_parameter('max_wheel_speed', 10.0)   # rad/s
        # O turtle_teleop_key manda 2.0 fixo nos dois eixos; isso vira
        # 0.3 m/s e 1.5 rad/s.
        self.declare_parameter('linear_scale', 0.15)
        self.declare_parameter('angular_scale', 0.75)
        self.declare_parameter('cmd_timeout', 0.5)        # s até parar sem comando

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # Tópico relativo: herda o namespace do launch.
        self.create_subscription(Twist, 'turtle1/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info(
            f'aguardando comandos em {self.resolve_topic_name("turtle1/cmd_vel")}'
        )

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x * self.linear_scale
        w = msg.angular.z * self.angular_scale

        # Cinemática inversa: cada roda está a L/2 do centro.
        half_track = self.wheel_separation / 2.0
        left = (v - w * half_track) / self.wheel_radius
        right = (v + w * half_track) / self.wheel_radius

        # Satura mantendo a proporção entre as rodas (mesma curva, mais lenta).
        excess = max(abs(left), abs(right)) / self.max_wheel_speed
        if excess > 1.0:
            left, right = left / excess, right / excess

        self.command.set(left, right, self.cmd_timeout)


class MotorPublisherNode(Node):
    """Publica a velocidade de cada roda em my_robot/left_motor e my_robot/right_motor."""

    def __init__(self, command: WheelCommand):
        super().__init__('motor_publisher_node')
        self.command = command

        self.declare_parameter('rate', 20.0)  # Hz

        self.left_pub = self.create_publisher(Float32, 'my_robot/left_motor', 10)
        self.right_pub = self.create_publisher(Float32, 'my_robot/right_motor', 10)
        # Publica sempre, mesmo sem comando: assim o robô recebe zero e para.
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)

    def step(self):
        left, right = self.command.get()
        self.publish(left, right)

    def publish(self, left, right):
        self.left_pub.publish(Float32(data=float(left)))
        self.right_pub.publish(Float32(data=float(right)))


def main(args=None):
    rclpy.init(args=args)
    command = WheelCommand()
    car = CarNode(command)
    motors = MotorPublisherNode(command)

    executor = MultiThreadedExecutor()
    executor.add_node(car)
    executor.add_node(motors)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            motors.publish(0.0, 0.0)
        executor.shutdown()
        car.destroy_node()
        motors.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
