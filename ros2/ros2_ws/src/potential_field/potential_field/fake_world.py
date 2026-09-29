"""Mundo de mentira: substitui o CoppeliaSim para testar o navegador.

Publica os mesmos tópicos da coppelia_bridge (pose, bananas, poops) e assina o
mesmo cmd_vel, só que integrando a cinemática do robô em vez de falar com o
simulador. Serve para conferir o campo potencial sem abrir o CoppeliaSim, e
para rodar o teste automático do pacote.

    ros2 launch potential_field potential_field.launch.py fake:=1

O mapa padrão põe três bananas com uma parede de poops entre o robô e a
primeira delas, que é justamente o caso em que o campo empata e a componente
tangencial do navegador precisa agir.
"""

import math

from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


class FakeWorld(Node):
    """Cinemática de robô diferencial integrada no tempo do ROS."""

    def __init__(self):
        super().__init__('fake_world')

        # Mapa em coordenadas achatadas [x1, y1, x2, y2, ...], o mesmo formato
        # dos sinais 'banana' e 'poop' da cena.
        self.declare_parameter('bananas', [1.5, 0.0, 0.0, 1.5, -1.5, -1.0])
        self.declare_parameter('poops', [
            0.7, -0.4, 0.7, -0.2, 0.7, 0.0, 0.7, 0.2, 0.7, 0.4,
            -0.6, 0.9, 0.0, -0.9,
        ])
        self.declare_parameter('start', [0.0, 0.0, 0.0])   # x, y, yaw
        self.declare_parameter('rate', 20.0)               # Hz
        self.declare_parameter('cmd_timeout', 0.5)         # s até parar sem comando
        self.declare_parameter('frame_id', 'world')

        self.x, self.y, self.yaw = self.get_parameter('start').value
        self.frame_id = self.get_parameter('frame_id').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.v = 0.0
        self.w = 0.0
        self.deadline = 0.0
        self.last = self.now()

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.banana_pub = self.create_publisher(PoseArray, 'bananas', map_qos)
        self.poop_pub = self.create_publisher(PoseArray, 'poops', map_qos)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.publish_map()
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.get_logger().info('mundo de mentira no ar (sem CoppeliaSim)')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_map(self):
        for name, pub in (('bananas', self.banana_pub), ('poops', self.poop_pub)):
            flat = self.get_parameter(name).value
            array = PoseArray()
            array.header.stamp = self.get_clock().now().to_msg()
            array.header.frame_id = self.frame_id
            for i in range(0, len(flat) - 1, 2):
                pose = Pose()
                pose.position.x = float(flat[i])
                pose.position.y = float(flat[i + 1])
                pose.orientation.w = 1.0
                array.poses.append(pose)
            pub.publish(array)

    def cmd_vel_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.deadline = self.now() + self.cmd_timeout

    def step(self):
        """Integra a pose e publica, como a ponte faria com a cena."""
        now = self.now()
        dt = now - self.last
        self.last = now
        if now >= self.deadline:
            self.v = self.w = 0.0

        # Cinemática do uniciclo: a pose anda na direção do próprio rumo.
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + self.w * dt), math.cos(self.yaw + self.w * dt)
        )

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.yaw / 2.0)
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeWorld()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
