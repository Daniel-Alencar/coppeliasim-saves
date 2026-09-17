"""Ponte ROS 2 <-> CoppeliaSim para o robô diferencial /myRobot.

Assina cmd_vel (geometry_msgs/Twist), converte a velocidade do corpo em
velocidades de roda e aplica nos motores pela ZeroMQ Remote API. Publica de
volta joint_states e proximity.

Os tópicos são relativos: com o namespace diff_robot definido no launch eles
viram /diff_robot/cmd_vel, /diff_robot/joint_states e /diff_robot/proximity.

Fluxo de dados:

    dummy_driver / teleop --(cmd_vel)--> esta ponte --(ZeroMQ)--> CoppeliaSim
    CoppeliaSim --(ZeroMQ)--> esta ponte --(joint_states, proximity)--> ROS 2
"""

import math        # math.dist: distância entre os dois motores
import signal      # tratamento manual do Ctrl+C (SIGINT) e do SIGTERM
import threading   # threading.Event: flag de parada segura entre sinal e laço
import time        # time.monotonic: relógio que nunca volta, ideal para prazos

import rclpy                                    # biblioteca cliente do ROS 2 em Python
from rclpy.node import Node                     # classe base de todo nó ROS 2
from rclpy.signals import SignalHandlerOptions  # permite desligar o Ctrl+C do rclpy
from geometry_msgs.msg import Twist             # comando de velocidade linear/angular
from sensor_msgs.msg import JointState, Range   # estado das juntas e leitura de distância

# Coppelia ZeroMQ Remote API: permite chamar as funções sim.* do CoppeliaSim a
# partir de um processo externo, pela porta TCP 23000.
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class CoppeliaBridge(Node):
    """Nó ROS 2 que traduz comandos de velocidade em comandos de motor no CoppeliaSim."""

    def __init__(self):
        # Registra o nó no ROS com o nome coppelia_bridge (é o nome que aparece
        # em `ros2 node list`).
        super().__init__('coppelia_bridge')

        # --- Parâmetros ------------------------------------------------------
        # Parâmetros ROS podem ser trocados na linha de comando sem editar o
        # código, por exemplo: --ros-args -p cmd_timeout:=1.0
        self.declare_parameter('robot', '/myRobot')     # caminho do robô na cena
        self.declare_parameter('wheel_radius', 0.05)    # raio da roda, em metros
        # 0.0 = medir a distância entre as rodas na própria cena.
        self.declare_parameter('wheel_separation', 0.0)
        self.declare_parameter('max_wheel_speed', 10.0)  # limite por roda, em rad/s
        # Multiplicam o Twist recebido. 1.0 = comando já em m/s e rad/s; para o
        # turtle_teleop_key (2.0 fixo) use 0.15 e 0.75.
        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale', 1.0)
        self.declare_parameter('sensor_range', 1.0)     # alcance do sensor, em metros
        self.declare_parameter('cmd_timeout', 0.5)      # segundos até parar sem comando
        self.declare_parameter('rate', 20.0)            # frequência do laço step(), em Hz
        self.declare_parameter('autostart', True)       # dar play se a simulação estiver parada

        # Lê os valores finais: o padrão acima ou o que veio na linha de comando.
        robot = self.get_parameter('robot').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.linear_scale = self.get_parameter('linear_scale').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # --- Conexão com o CoppeliaSim ---------------------------------------
        # Handles são identificadores inteiros dos objetos da cena. Toda chamada
        # sim.* que age sobre um objeto recebe o handle dele.
        self.sim = self.connect()
        self.leftMotorHandle = self.find(f'{robot}/leftMotor')
        self.rightMotorHandle = self.find(f'{robot}/rightMotor')
        self.sensorNariz = self.find(f'{robot}/proximitySensor')
        self.warn_enabled_scripts(robot)

        # --- Geometria do robô -----------------------------------------------
        # A distância entre as rodas é necessária para a cinemática do giro.
        self.wheel_separation = self.get_parameter('wheel_separation').value
        if self.wheel_separation <= 0.0:
            self.wheel_separation = self.measure_wheel_separation()
        self.get_logger().info(
            f'raio da roda: {self.wheel_radius:.3f} m | '
            f'distância entre rodas: {self.wheel_separation:.3f} m'
        )

        self.range_max = self.get_parameter('sensor_range').value

        # --- Estado da simulação ---------------------------------------------
        # started_here lembra se foi esta ponte que deu o play. Só nesse caso
        # ela para a simulação ao sair, para não interromper uma simulação que
        # o usuário iniciou.
        self.started_here = (
            self.get_parameter('autostart').value
            and self.sim.getSimulationState() == self.sim.simulation_stopped
        )
        if self.started_here:
            self.sim.startSimulation()
        elif self.sim.getSimulationState() == self.sim.simulation_paused:
            self.get_logger().warn(
                'a simulação está pausada: os motores só respondem depois do play.'
            )

        # --- Estado do comando -----------------------------------------------
        # velocity: última velocidade pedida para (roda esquerda, roda direita), em rad/s.
        # deadline: instante, em time.monotonic(), em que esse comando expira.
        self.velocity = (0.0, 0.0)
        self.deadline = 0.0

        # --- Comunicação ROS -------------------------------------------------
        # Assinatura: cada Twist publicado em cmd_vel chama cmd_vel_callback.
        # Sem a barra inicial o tópico herda o namespace do nó. O 10 é a
        # profundidade da fila de mensagens (QoS).
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # Publicadores: devolvem ao ROS as leituras dos motores e do sensor.
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.range_pub = self.create_publisher(Range, 'proximity', 10)

        # Timer: chama step() a cada 1/rate segundos (50 ms com rate = 20). É o
        # step() que conversa com o simulador; o callback só guarda o comando.
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.get_logger().info(
            f'ponte pronta. aguardando comandos em {self.resolve_topic_name("cmd_vel")}'
        )

    def connect(self):
        """Conecta ao CoppeliaSim, explicando o motivo quando não dá."""
        try:
            # RemoteAPIClient() conecta em localhost:23000. getObject('sim')
            # devolve um objeto com as mesmas funções sim.* dos scripts da cena.
            self.client = RemoteAPIClient()
            return self.client.getObject('sim')
        except Exception:
            # SystemExit é capturado em main(), que mostra a mensagem e encerra.
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
            # Lista o caminho de todos os objetos da cena para ajudar a achar o
            # nome certo. O 2 em getObjectAlias pede o caminho completo.
            existentes = [
                self.sim.getObjectAlias(h, 2)
                for h in self.sim.getObjectsInTree(self.sim.handle_scene)
            ]
            raise SystemExit(
                f'Objeto "{path}" não existe na cena aberta.\n'
                'Objetos disponíveis:\n  '
                + '\n  '.join(existentes)
                + '\nAjuste o parâmetro robot, por exemplo:\n'
                '  ros2 launch diff_robot diff_robot.launch.py robot:=/meuRobo'
            )

    def warn_enabled_scripts(self, robot):
        """Avisa sobre child scripts do robô que ainda disputam os motores."""
        # Todos os objetos do tipo script pendurados na árvore do robô.
        scripts = self.sim.getObjectsInTree(
            self.sim.getObject(robot), self.sim.sceneobject_script
        )
        for h in scripts:
            if not self.sim.getBoolProperty(h, 'scriptDisabled'):
                self.get_logger().warn(
                    f'o script {self.sim.getObjectAlias(h, 2)} está habilitado e '
                    'sobrescreve os comandos do teleop. Desabilite-o na cena.'
                )

    def measure_wheel_separation(self):
        """Distância entre os dois motores, lida direto da cena."""
        # Posições (x, y, z) dos motores no referencial do mundo.
        left = self.sim.getObjectPosition(self.leftMotorHandle, self.sim.handle_world)
        right = self.sim.getObjectPosition(self.rightMotorHandle, self.sim.handle_world)
        return math.dist(left, right)

    def cmd_vel_callback(self, msg: Twist):
        """Converte o Twist recebido em velocidades de roda.

        Não fala com o simulador: só guarda o comando, que o próximo step() aplica.
        """
        # No Twist, linear.x é a velocidade para frente (m/s) e angular.z é a
        # velocidade de giro em torno do eixo vertical (rad/s; positivo = virar
        # à esquerda). As escalas adaptam os valores fixos do teleop a este robô.
        v = msg.linear.x * self.linear_scale
        w = msg.angular.z * self.angular_scale

        # Cinemática inversa do robô diferencial: m/s e rad/s -> rad/s de cada roda.
        # Cada roda está a L/2 do centro, então sua velocidade linear é v ∓ w·L/2.
        # Dividir pelo raio converte velocidade linear da roda em angular.
        # Com w > 0 a roda direita gira mais rápido e o robô vira à esquerda.
        half_track = self.wheel_separation / 2.0
        left = (v - w * half_track) / self.wheel_radius
        right = (v + w * half_track) / self.wheel_radius

        # Se alguma roda passar do limite, divide as duas pelo mesmo fator.
        # Isso mantém a proporção entre elas, ou seja, a mesma curva, só mais lenta.
        limit = self.max_wheel_speed
        excess = max(abs(left), abs(right)) / limit
        if excess > 1.0:
            left, right = left / excess, right / excess

        self.velocity = (left, right)
        # O comando vale por cmd_timeout segundos a partir de agora.
        self.deadline = time.monotonic() + self.cmd_timeout

    def step(self):
        """Executado pelo timer: aplica o comando atual e publica as leituras."""
        # Sem comando recente o robô para sozinho, mesmo que o teleop caia.
        # Um teleop de teclado só publica quando uma tecla chega, então soltar
        # a tecla faz o comando expirar e o robô parar.
        if time.monotonic() >= self.deadline:
            self.velocity = (0.0, 0.0)

        # Define a velocidade alvo (rad/s) de cada motor; o motor da simulação
        # aplica torque para alcançá-la.
        self.sim.setJointTargetVelocity(self.leftMotorHandle, self.velocity[0])
        self.sim.setJointTargetVelocity(self.rightMotorHandle, self.velocity[1])

        # O mesmo carimbo de tempo vale para as duas mensagens deste ciclo.
        now = self.get_clock().now().to_msg()

        # JointState: posição (rad, acumulada) e velocidade (rad/s) medidas nos
        # motores. As listas seguem a ordem de joints.name.
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

        # readProximitySensor devolve (detectou, distância, ponto detectado,
        # handle do objeto, normal da superfície); só os dois primeiros importam.
        detected, distance = self.sim.readProximitySensor(self.sensorNariz)[:2]
        # Range é a mensagem padrão do ROS para sensores de distância de um feixe.
        scan = Range()
        scan.header.stamp = now
        scan.header.frame_id = 'proximity_sensor'  # referencial em que a medida vale
        scan.radiation_type = Range.INFRARED
        scan.field_of_view = 0.1                   # abertura do feixe, em rad
        scan.min_range = 0.0
        scan.max_range = self.range_max
        # Convenção do ROS: fora de alcance é publicado como +infinito.
        scan.range = float(distance) if detected else float('inf')
        self.range_pub.publish(scan)

    def stop(self):
        """Para os motores e, se foi a ponte que deu o play, para a simulação."""
        self.sim.setJointTargetVelocity(self.leftMotorHandle, 0.0)
        self.sim.setJointTargetVelocity(self.rightMotorHandle, 0.0)
        if self.started_here:
            self.sim.stopSimulation()


def main(args=None):
    # O Ctrl+C só levanta uma flag. Deixar o rclpy ou o KeyboardInterrupt cortarem
    # o laço no meio de uma chamada ZeroMQ invalida o socket e o contexto, e aí o
    # comando de parada dos motores nunca chega ao simulador.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_requested.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_requested.set())

    # Cria o nó. Falhas de conexão ou objeto inexistente chegam aqui como
    # SystemExit com uma mensagem explicativa (ver connect e find).
    try:
        node = CoppeliaBridge()
    except SystemExit as erro:
        print(erro)
        rclpy.shutdown()
        return 1
    try:
        # Equivale a rclpy.spin(node), mas confere a flag a cada 0,1 s.
        # Cada spin_once executa no máximo um callback pronto (mensagem ou timer).
        while not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        # Roda sempre, inclusive depois do Ctrl+C: para o robô e libera o ROS.
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    # O valor devolvido por main() vira o código de saída do processo.
    raise SystemExit(main())
