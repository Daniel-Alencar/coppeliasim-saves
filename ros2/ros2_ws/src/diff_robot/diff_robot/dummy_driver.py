"""Motorista de teste para o robô diferencial.

Publica em cmd_vel (geometry_msgs/Twist) uma sequência fixa de manobras que se
repete: anda para frente, gira à esquerda, anda para frente, gira à direita.
Serve para testar a ponte com o CoppeliaSim sem precisar de teleop.

Ligado pelo launch com dummy:=1.
"""

import rclpy                         # biblioteca cliente do ROS 2 em Python
from rclpy.node import Node          # classe base de todo nó ROS 2
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist  # comando de velocidade linear/angular


class DummyDriver(Node):
    """Nó que repete um roteiro de (v, w, duração) em cmd_vel."""

    def __init__(self):
        super().__init__('dummy_driver')

        # Parâmetros: velocidades do roteiro e frequência de publicação.
        self.declare_parameter('linear_speed', 0.2)    # m/s
        self.declare_parameter('angular_speed', 1.0)   # rad/s
        self.declare_parameter('forward_time', 3.0)    # segundos andando reto
        self.declare_parameter('turn_time', 1.5)       # segundos girando
        self.declare_parameter('rate', 10.0)           # Hz

        v = self.get_parameter('linear_speed').value
        w = self.get_parameter('angular_speed').value
        forward = self.get_parameter('forward_time').value
        turn = self.get_parameter('turn_time').value

        # Roteiro: cada etapa é (velocidade linear, velocidade angular, duração).
        self.script = [
            (v, 0.0, forward),
            (0.0, w, turn),
            (v, 0.0, forward),
            (0.0, -w, turn),
        ]
        self.index = 0
        self.stage_start = self.get_clock().now()

        # Tópico relativo: herda o namespace, o mesmo que a ponte assina.
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Publica continuamente: a ponte para o robô se o comando expirar.
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.get_logger().info(
            f'publicando roteiro de teste em {self.resolve_topic_name("cmd_vel")}'
        )

    def step(self):
        """Avança de etapa quando o tempo acaba e publica o comando atual."""
        v, w, duration = self.script[self.index]
        elapsed = (self.get_clock().now() - self.stage_start).nanoseconds / 1e9
        if elapsed >= duration:
            self.index = (self.index + 1) % len(self.script)
            self.stage_start = self.get_clock().now()
            v, w, _ = self.script[self.index]

        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.pub.publish(msg)

    def stop(self):
        """Manda velocidade zero antes de sair."""
        self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DummyDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
