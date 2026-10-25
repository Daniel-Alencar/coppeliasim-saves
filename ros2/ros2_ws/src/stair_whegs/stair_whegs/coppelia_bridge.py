"""Ponte ROS 2 <-> CoppeliaSim para o rover de whegs.

Só comunicação, como as pontes dos exercícios anteriores: não gera marcha, não
decide nada. A marcha está em `gait` e as decisões em `climber`, e nenhum dos
dois conhece o CoppeliaSim.

    CoppeliaSim                              ROS 2 (namespace whegs)
    pose do chassi                 --->   pose          geometry_msgs/PoseStamped
    atitude do dummy imu           --->   attitude      geometry_msgs/Vector3Stamped
                                   --->   /tf           world -> base_link
    as quatro juntas de wheg       --->   wheg_state    sensor_msgs/JointState
    probe_l, probe_r, edge         --->   probes        std_msgs/Float64MultiArray
    relógio da simulação           --->   sim_time      std_msgs/Float64
    as quatro juntas de wheg       <---   wheg_joints   std_msgs/Float64MultiArray
    parar a simulação              <---   done          std_msgs/Bool

A ATITUDE VEM DE UM DUMMY, NÃO DO CHASSI
----------------------------------------
O chassi é construído com `sim.groupShapes` (corpo + cauda), e o CoppeliaSim
realinha o frame do shape agrupado com os eixos principais de inércia: com a
cauda, ele nasce 20° inclinado em relação ao mundo. Ler a orientação dele daria
um rover permanentemente empinado.

O dummy `/rover/chassis/imu` é criado depois, alinhado ao mundo, e preso ao
chassi — é o que um IMU de verdade seria, um referencial solidário ao corpo
montado nivelado. A ponte lê a atitude dele, e por isso `attitude` vale zero
com o rover parado no plano.

AS SONDAS PUBLICAM DISTÂNCIA, NÃO "DETECTEI"
--------------------------------------------
Os três sensores apontam para baixo e para a frente, e o que eles informam é a
DISTÂNCIA até o terreno (ver `scene/build_scene.add_sensors` para o porquê).
Quando nada é detectado — o caso da beirada do patamar, em que o raio mergulha
no vazio — a ponte publica o ALCANCE MÁXIMO do sensor, que é a leitura
fisicamente correta: "mais longe do que eu enxergo". Assim o nó que consome
não precisa tratar nenhum valor especial, e "longe" e "nada" querem dizer a
mesma coisa, que é o que são.
"""

import math
import signal
import threading
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray
import tf2_ros

from stair_whegs.geometry import WHEGS

#: Nomes dos três sensores, na ordem em que saem no tópico `probes`.
PROBES = ('probe_l', 'probe_r', 'edge')


class CoppeliaBridge(Node):
    """Nó ROS 2 que liga a cena do rover de whegs a tópicos ROS."""

    def __init__(self):
        super().__init__('coppelia_bridge')

        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 23000)
        self.declare_parameter('rover', '/rover')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('autostart', True)
        self.declare_parameter('wheg_torque', 2.0)        # [N.m]
        self.declare_parameter('cmd_timeout', 0.5)        # [s]
        # Em stepping a ponte é quem dá os passos da simulação, e cada passo
        # leva um comando calculado a partir do estado daquele mesmo passo. É a
        # mesma escolha das pontes anteriores, e pelo mesmo motivo: em curso
        # livre o laço de controle fica à mercê da carga da máquina.
        self.declare_parameter('stepping', True)
        self.declare_parameter('rate', 20.0)              # [Hz] em curso livre
        self.declare_parameter('stop_on_done', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        rover = self.get_parameter('rover').value
        self.sim = self.connect(host, port)

        self.chassis = self.find(f'{rover}/chassis', 'o corpo do rover')
        self.imu = self.find(f'{rover}/chassis/imu', 'o referencial de atitude')
        self.joints = [self.find(f'{rover}/chassis/{name}_joint',
                                 f'a junta do wheg {name}') for name in WHEGS]
        self.probes = [self.find(f'{rover}/chassis/{name}', f'o sensor {name}')
                       for name in PROBES]
        # O alcance de cada sensor é lido DA CENA, e não repetido aqui: se
        # alguém montar a cena com outro alcance, a ponte acompanha.
        self.probe_range = [float(self.sim.getFloatProperty(handle, 'volume_range'))
                            for handle in self.probes]

        torque = self.get_parameter('wheg_torque').value
        for joint in self.joints:
            self.sim.setObjectInt32Param(joint,
                                         self.sim.jointintparam_dynctrlmode,
                                         self.sim.jointdynctrl_position)
            self.sim.setJointTargetForce(joint, torque)

        self.started_here = (
            self.get_parameter('autostart').value
            and self.sim.getSimulationState() == self.sim.simulation_stopped)
        if self.started_here:
            self.sim.startSimulation()
        elif self.sim.getSimulationState() == self.sim.simulation_paused:
            self.get_logger().warn('a simulação está pausada: nada muda até o play.')

        self.targets = None               # últimas posições de junta pedidas
        self.deadline = 0.0
        self.done = threading.Event()

        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.attitude_pub = self.create_publisher(Vector3Stamped, 'attitude', 10)
        self.state_pub = self.create_publisher(JointState, 'wheg_state', 10)
        self.probe_pub = self.create_publisher(Float64MultiArray, 'probes', 10)
        self.sim_time_pub = self.create_publisher(Float64, 'sim_time', 10)
        self.create_subscription(Float64MultiArray, 'wheg_joints',
                                 self.joints_callback, 10)
        self.create_subscription(Bool, 'done', self.done_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.step_time = self.sim.getFloatParam(
            self.sim.floatparam_simulation_time_step)
        self.sim_time = self.sim.getSimulationTime()
        self.stepping = self.get_parameter('stepping').value
        if self.stepping:
            self.client.setStepping(True)
            self.get_logger().info(
                f'modo stepping: um ciclo por passo de '
                f'{self.step_time * 1000:.0f} ms')
        else:
            rate = self.get_parameter('rate').value
            self.create_timer(1.0 / rate, self.publish_state)
            self.create_timer(1.0 / rate, self.actuate)

        self.get_logger().info(
            f'ponte pronta. comandos em {self.resolve_topic_name("wheg_joints")}')

    # =========================================================================
    #  Conexão
    # =========================================================================

    def connect(self, host, port):
        try:
            self.client = RemoteAPIClient(host, port)
            return self.client.require('sim')
        except Exception:
            raise SystemExit(
                f'Não consegui falar com o CoppeliaSim em {host}:{port}.\n'
                '  - O simulador está aberto com a cena stair_whegs.ttt?\n'
                '  - Monte a cena com: python3 scene/build_scene.py')

    def find(self, path, what=''):
        handle = self.sim.getObject(path, {'noError': True})
        if handle != -1:
            return handle
        existing = [self.sim.getObjectAlias(h, 2)
                    for h in self.sim.getObjectsInTree(self.sim.handle_scene)]
        raise SystemExit(
            f'não há "{path}" na cena{f" ({what})" if what else ""}.\n'
            'Monte a cena com scene/build_scene.py. A cena tem:\n  '
            + '\n  '.join(existing))

    # =========================================================================
    #  Um ciclo
    # =========================================================================

    def cycle(self):
        """Um passo de simulação, com o ciclo de controle inteiro dentro dele."""
        self.publish_state()
        rclpy.spin_once(self, timeout_sec=0.02)
        for _ in range(6):
            rclpy.spin_once(self, timeout_sec=0.0)
        self.actuate()
        self.sim.step()
        self.sim_time += self.step_time
        self.sim_time_pub.publish(Float64(data=self.sim_time))

    # =========================================================================
    #  CoppeliaSim -> ROS
    # =========================================================================

    def publish_state(self):
        """Pose, atitude, juntas e sondas."""
        stamp = self.get_clock().now().to_msg()

        x, y, z = self.sim.getObjectPosition(self.chassis, self.sim.handle_world)
        matrix = self.sim.getObjectMatrix(self.imu, self.sim.handle_world)
        roll, pitch, yaw = self.attitude_of(matrix)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(pose)

        attitude = Vector3Stamped()
        attitude.header.stamp = stamp
        attitude.header.frame_id = self.base_frame
        attitude.vector.x = roll
        attitude.vector.y = pitch
        attitude.vector.z = yaw
        self.attitude_pub.publish(attitude)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = z
        transform.transform.rotation.z = pose.pose.orientation.z
        transform.transform.rotation.w = pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(transform)

        state = JointState()
        state.header.stamp = stamp
        state.name = list(WHEGS)
        state.position = [self.sim.getJointPosition(joint)
                          for joint in self.joints]
        self.state_pub.publish(state)

        readings = []
        for handle, reach in zip(self.probes, self.probe_range):
            result = self.sim.readProximitySensor(handle)
            # Sem detecção = mais longe do que o sensor enxerga. Publicar o
            # alcance máximo diz exatamente isso, e evita que o consumidor
            # tenha de tratar um valor especial.
            readings.append(float(result[1]) if result[0] > 0 else float(reach))
        self.probe_pub.publish(Float64MultiArray(data=readings))

        if not self.stepping:
            self.sim_time = self.sim.getSimulationTime()
            self.sim_time_pub.publish(Float64(data=self.sim_time))

    @staticmethod
    def attitude_of(m):
        """(roll, pitch, yaw) da matriz 3x4 do CoppeliaSim, em ordem de linhas.

        O pitch é medido pela subida do eixo x do corpo (m[8]), e não por uma
        decomposição de Euler qualquer: perto de 90° os ângulos de Euler trocam
        de ramo, e um rover empinado passaria a parecer capotado. Esta forma é
        monótona em toda a faixa que interessa.
        """
        pitch = math.asin(max(-1.0, min(1.0, m[8])))
        roll = math.atan2(-m[9], m[10])
        yaw = math.atan2(m[4], m[0])
        return roll, pitch, yaw

    # =========================================================================
    #  ROS -> CoppeliaSim
    # =========================================================================

    def joints_callback(self, msg: Float64MultiArray):
        if len(msg.data) != len(self.joints):
            self.get_logger().warn(
                f'wheg_joints com {len(msg.data)} valores; '
                f'esperava {len(self.joints)}')
            return
        self.targets = list(msg.data)
        self.deadline = time.monotonic() + self.get_parameter('cmd_timeout').value

    def done_callback(self, msg: Bool):
        if msg.data and not self.done.is_set():
            self.get_logger().info('missão terminou: encerrando a ponte')
            self.done.set()

    def actuate(self):
        """Escreve nas juntas o último comando recebido.

        Sem comando recente, as juntas ficam ONDE ESTÃO — e não em zero. Num
        robô de pernas, mandar as juntas para a origem ao perder o comando
        faria o corpo desabar; parar de girar é a reação segura.
        """
        if self.targets is None:
            return
        if time.monotonic() >= self.deadline:
            self.targets = [self.sim.getJointPosition(joint)
                            for joint in self.joints]
            self.deadline = time.monotonic() + 1e9    # congela aqui até voltar
        for joint, target in zip(self.joints, self.targets):
            self.sim.setJointTargetPosition(joint, target)

    def stop(self):
        try:
            if self.stepping:
                self.sim.step()
                self.client.setStepping(False)
            if self.started_here or (self.done.is_set()
                                     and self.get_parameter('stop_on_done').value):
                self.sim.stopSimulation()
        except Exception as error:
            self.get_logger().warn(f'na saída: {error}')


def main(args=None):
    # O Ctrl+C só levanta uma flag: deixar o rclpy cortar o laço no meio de uma
    # chamada ZeroMQ invalida o socket, e aí a simulação fica esperando um
    # passo que ninguém mais vai pedir.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_requested.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_requested.set())

    try:
        node = CoppeliaBridge()
    except SystemExit as error:
        print(error)
        rclpy.shutdown()
        return 1
    try:
        while not stop_requested.is_set() and not node.done.is_set():
            if node.stepping:
                node.cycle()
            else:
                rclpy.spin_once(node, timeout_sec=0.1)
        if node.done.is_set():
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
