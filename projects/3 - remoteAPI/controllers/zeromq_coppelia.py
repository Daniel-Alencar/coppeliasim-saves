"""Ponte ROS 2 <-> CoppeliaSim para o robô diferencial /myRobot.

Assina /turtle1/cmd_vel (geometry_msgs/Twist) — o tópico que o teleop do
turtlesim publica —, converte a velocidade do corpo em velocidades de roda e
aplica nos motores pela ZeroMQ Remote API. Publica de volta /joint_states e
/proximity.

Atenção: os child scripts da cena (/myRobot/controler e /myRobot/python_controler)
também escrevem nos motores a cada passo de simulação. Desabilite os dois na
cena, senão eles sobrescrevem os comandos enviados por aqui.

Uso (um terminal para cada comando, com a cena aberta no CoppeliaSim):

    source /opt/ros/jazzy/setup.bash
    export PYTHONPATH="$HOME/Softwares/CoppeliaSim/CoppeliaSim_Edu/programming/zmqRemoteApi/clients/python/src:$PYTHONPATH"

    /usr/bin/python3 zeromq_coppelia.py
    ros2 run turtlesim turtle_teleop_key

Dirija com as setas, mantendo o foco no terminal do turtle_teleop_key. Ele envia
uma mensagem por tecla, então cada toque move o robô por cmd_timeout segundos;
segure a tecla para andar continuamente.

Use /usr/bin/python3: o python do Anaconda não consegue importar o rclpy.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Range

# Coppelia ZeroMQ Remote API
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class CoppeliaBridge(Node):
    def __init__(self):
        super().__init__('coppelia_bridge')

        self.declare_parameter('robot', '/myRobot')
        self.declare_parameter('wheel_radius', 0.05)
        # 0.0 = medir a distância entre as rodas na própria cena.
        self.declare_parameter('wheel_separation', 0.0)
        self.declare_parameter('max_wheel_speed', 10.0)
        # O turtle_teleop_key manda 2.0 fixo nos dois eixos, pensado para a
        # tartaruga. Aqui isso vira 0.3 m/s e 1.5 rad/s.
        self.declare_parameter('linear_scale', 0.15)
        self.declare_parameter('angular_scale', 0.75)
        self.declare_parameter('sensor_range', 1.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('autostart', True)

        robot = self.get_parameter('robot').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.sim = self.connect()
        self.leftMotorHandle = self.find(f'{robot}/leftMotor')
        self.rightMotorHandle = self.find(f'{robot}/rightMotor')
        self.sensorNariz = self.find(f'{robot}/proximitySensor')

        self.wheel_separation = self.get_parameter('wheel_separation').value
        if self.wheel_separation <= 0.0:
            self.wheel_separation = self.measure_wheel_separation()
        self.get_logger().info(
            f'raio da roda: {self.wheel_radius:.3f} m | '
            f'distância entre rodas: {self.wheel_separation:.3f} m'
        )

        self.range_max = self.get_parameter('sensor_range').value

        self.started_here = (
            self.get_parameter('autostart').value
            and self.sim.getSimulationState() == self.sim.simulation_stopped
        )
        if self.started_here:
            self.sim.startSimulation()

        self.velocity = (0.0, 0.0)
        self.deadline = 0.0
        self.create_subscription(Twist, '/turtle1/cmd_vel', self.cmd_vel_callback, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.range_pub = self.create_publisher(Range, '/proximity', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.get_logger().info(
            'ponte pronta. rode: ros2 run turtlesim turtle_teleop_key'
        )

    def connect(self):
        """Conecta ao CoppeliaSim, explicando o motivo quando não dá."""
        try:
            self.client = RemoteAPIClient()
            return self.client.getObject('sim')
        except Exception:
            raise SystemExit(
                'Não consegui falar com o CoppeliaSim na porta 23000.\n'
                '  - O simulador está aberto?\n'
                '  - Ele responde? Um child script da cena preso em laço (por exemplo\n'
                '    um controlador em Python com curses, ou que abre um RemoteAPIClient\n'
                '    para o próprio simulador) trava a thread principal: a porta continua\n'
                '    aberta, mas nenhuma chamada é respondida. O log da cena mostra\n'
                '    "script execution was terminated externally" quando é esse o caso.\n'
                '    Pare a simulação e desabilite esse script.'
            )

    def find(self, path):
        """Busca um objeto e, se não achar, mostra o que existe na cena."""
        try:
            return self.sim.getObject(path)
        except Exception:
            existentes = [
                self.sim.getObjectAlias(h, 2)
                for h in self.sim.getObjectsInTree(self.sim.handle_scene)
            ]
            raise SystemExit(
                f'Objeto "{path}" não existe na cena aberta.\n'
                'Objetos disponíveis:\n  '
                + '\n  '.join(existentes)
                + '\nAjuste o parâmetro robot, por exemplo:\n'
                '  /usr/bin/python3 zeromq_coppelia.py --ros-args -p robot:=/meuRobo'
            )

    def measure_wheel_separation(self):
        """Distância entre os dois motores, lida direto da cena."""
        left = self.sim.getObjectPosition(self.leftMotorHandle, self.sim.handle_world)
        right = self.sim.getObjectPosition(self.rightMotorHandle, self.sim.handle_world)
        return math.dist(left, right)

    def cmd_vel_callback(self, msg: Twist):
        # Cinemática inversa do robô diferencial: m/s e rad/s -> rad/s de cada roda.
        v = msg.linear.x * self.linear_scale
        w = msg.angular.z * self.angular_scale
        half_track = self.wheel_separation / 2.0
        left = (v - w * half_track) / self.wheel_radius
        right = (v + w * half_track) / self.wheel_radius

        limit = self.max_wheel_speed
        excess = max(abs(left), abs(right)) / limit
        if excess > 1.0:
            left, right = left / excess, right / excess

        self.velocity = (left, right)
        self.deadline = time.monotonic() + self.cmd_timeout

    def step(self):
        # Durante o encerramento o contexto já pode ter sido destruído.
        if not rclpy.ok():
            return

        # Sem comando recente o robô para sozinho, mesmo que o teleop caia.
        if time.monotonic() >= self.deadline:
            self.velocity = (0.0, 0.0)

        self.sim.setJointTargetVelocity(self.leftMotorHandle, self.velocity[0])
        self.sim.setJointTargetVelocity(self.rightMotorHandle, self.velocity[1])

        now = self.get_clock().now().to_msg()

        joints = JointState()
        joints.header.stamp = now
        joints.name = ['leftMotor', 'rightMotor']
        joints.position = [
            self.sim.getJointPosition(self.leftMotorHandle),
            self.sim.getJointPosition(self.rightMotorHandle),
        ]
        joints.velocity = [
            self.sim.getJointVelocity(self.leftMotorHandle),
            self.sim.getJointVelocity(self.rightMotorHandle),
        ]
        self.joint_pub.publish(joints)

        detected, distance = self.sim.readProximitySensor(self.sensorNariz)[:2]
        scan = Range()
        scan.header.stamp = now
        scan.header.frame_id = 'proximity_sensor'
        scan.radiation_type = Range.INFRARED
        scan.field_of_view = 0.1
        scan.min_range = 0.0
        scan.max_range = self.range_max
        # Convenção do ROS: fora de alcance é publicado como +infinito.
        scan.range = float(distance) if detected else float('inf')
        self.range_pub.publish(scan)

    def stop(self):
        self.sim.setJointTargetVelocity(self.leftMotorHandle, 0.0)
        self.sim.setJointTargetVelocity(self.rightMotorHandle, 0.0)
        if self.started_here:
            self.sim.stopSimulation()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CoppeliaBridge()
    except SystemExit as erro:
        print(erro)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
