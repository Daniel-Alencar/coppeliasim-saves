"""Ponte ROS 2 <-> CoppeliaSim para a cena das bananas e dos poops.

Expõe pelo ROS 2 tudo o que o navegador por campos potenciais precisa, lendo a
cena pela ZeroMQ Remote API (porta 23000). Não é preciso o plugin simROS2.

    CoppeliaSim                             ROS 2 (namespace myRobot)
    matriz do /myRobot           --->       pose        geometry_msgs/PoseStamped
    sinal 'banana' (tabela Lua)  --->       bananas     geometry_msgs/PoseArray
    sinal 'poop'   (tabela Lua)  --->       poops       geometry_msgs/PoseArray
    motores                      <---       cmd_vel     geometry_msgs/Twist

A ponte NÃO implementa o campo potencial: ela é só a comunicação. O algoritmo
está no nó navigator, que não conhece o CoppeliaSim — só tópicos ROS.

O mapa (bananas e poops)
------------------------
As posições são criadas pelo script /buildScene da cena no início da simulação,
nos sinais 'banana' e 'poop', como tabelas Lua empacotadas no formato
[x1, y1, x2, y2, ...]. Antes do play os sinais não existem, então a ponte tenta
lê-los a cada ciclo até aparecerem. Os dois tópicos usam QoS transient local
(a última mensagem fica guardada), então o navegador recebe o mapa mesmo que
suba depois da ponte.

O eixo da frente do robô
------------------------
Nesta cena a frente do /myRobot é o eixo +y do referencial dele, e não o +x
como é usual no ROS. A conversão é feita aqui (ver read_pose), e o que sai no
tópico pose é o rumo no sentido do ROS: yaw = 0 apontando para o +x do mundo.

Os motores
----------
As juntas estão montadas de forma que velocidade NEGATIVA faz o robô andar para
a frente, a mesma convenção do /myRobot/python_controler da cena. É o que o
parâmetro motor_sign (-1.0) resolve. Esse script precisa estar DESABILITADO:
ele escreve nos motores a cada passo de simulação e anula o cmd_vel.
"""

import math        # math.dist, math.hypot, math.sin/cos
import signal      # tratamento manual do Ctrl+C (SIGINT) e do SIGTERM
import threading   # threading.Event: flag de parada segura entre sinal e laço
import time        # time.monotonic: relógio que nunca volta, ideal para prazos

# Coppelia ZeroMQ Remote API: chama as funções sim.* a partir de outro processo.
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions


def unpack_pairs(flat):
    """[x1, y1, x2, y2, ...] -> [(x1, y1), (x2, y2), ...]."""
    if not flat:
        return []
    # O unpackTable devolve um dicionário quando a tabela Lua vem com chaves
    # numéricas esparsas; nesse caso a ordem é a das chaves 1, 2, 3...
    if isinstance(flat, dict):
        flat = [flat[k] for k in sorted(flat, key=lambda v: int(v))]
    return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]


class CoppeliaBridge(Node):
    """Nó ROS 2 que liga a cena das bananas a tópicos ROS."""

    def __init__(self):
        super().__init__('coppelia_bridge')

        # --- Parâmetros ------------------------------------------------------
        self.declare_parameter('host', 'localhost')     # onde está o CoppeliaSim
        self.declare_parameter('port', 23000)           # porta da ZeroMQ Remote API
        self.declare_parameter('robot', '/myRobot')     # caminho do robô na cena
        self.declare_parameter('banana_signal', 'banana')
        self.declare_parameter('poop_signal', 'poop')
        self.declare_parameter('wheel_radius', 0.05)    # raio da roda, em metros
        # 0.0 = medir a distância entre as rodas na própria cena.
        self.declare_parameter('wheel_separation', 0.0)
        # Nesta cena velocidade negativa na junta faz o robô andar para a frente.
        self.declare_parameter('motor_sign', -1.0)
        self.declare_parameter('max_wheel_speed', 10.0)  # limite por roda, em rad/s
        self.declare_parameter('cmd_timeout', 0.5)       # segundos até parar sem comando
        # Com a simulação rodando cada chamada da Remote API só é atendida entre
        # passos e custa ~12 ms, o que dá um teto de ~80 chamadas por segundo
        # para a ponte inteira. Por isso as frequências são baixas.
        self.declare_parameter('rate', 20.0)             # pose, em Hz (1 chamada por ciclo)
        self.declare_parameter('motor_rate', 20.0)       # motores, em Hz (2 chamadas por ciclo)
        self.declare_parameter('map_rate', 0.5)          # releitura do mapa, em Hz
        self.declare_parameter('frame_id', 'world')      # referencial das mensagens
        self.declare_parameter('autostart', True)        # dar play se estiver parada

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        robot = self.get_parameter('robot').value
        self.banana_signal = self.get_parameter('banana_signal').value
        self.poop_signal = self.get_parameter('poop_signal').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.motor_sign = self.get_parameter('motor_sign').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.frame_id = self.get_parameter('frame_id').value

        # --- Conexão com o CoppeliaSim ---------------------------------------
        self.sim = self.connect(host, port)
        self.robotHandle = self.find(robot)
        self.leftMotorHandle = self.find(f'{robot}/leftMotor')
        self.rightMotorHandle = self.find(f'{robot}/rightMotor')
        self.warn_motor_scripts(robot)

        self.wheel_separation = self.get_parameter('wheel_separation').value
        if self.wheel_separation <= 0.0:
            self.wheel_separation = self.measure_wheel_separation()
        self.get_logger().info(
            f'robô {robot} (handle {self.robotHandle}) | raio da roda: '
            f'{self.wheel_radius:.3f} m | distância entre rodas: '
            f'{self.wheel_separation:.3f} m'
        )

        # --- Estado da simulação ---------------------------------------------
        # started_here lembra se foi a ponte que deu o play. Só nesse caso ela
        # para a simulação ao sair.
        self.started_here = (
            self.get_parameter('autostart').value
            and self.sim.getSimulationState() == self.sim.simulation_stopped
        )
        if self.started_here:
            self.sim.startSimulation()
        elif self.sim.getSimulationState() == self.sim.simulation_paused:
            self.get_logger().warn(
                'a simulação está pausada: nada muda até o play.'
            )

        # --- Estado interno --------------------------------------------------
        self.velocity = (0.0, 0.0)   # (esquerda, direita) em m/s na roda
        self.deadline = 0.0          # instante em que o último cmd_vel expira
        self.map_warned = False      # já avisou que os sinais não existem?
        self.map_seen = (None, None)  # último mapa publicado, para não repetir log

        # --- Comunicação ROS -------------------------------------------------
        # Mapa: transient local guarda a última mensagem para quem chegar depois.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.banana_pub = self.create_publisher(PoseArray, 'bananas', map_qos)
        self.poop_pub = self.create_publisher(PoseArray, 'poops', map_qos)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Três timers: pose, motores e releitura do mapa têm ritmos diferentes.
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.create_timer(
            1.0 / self.get_parameter('motor_rate').value, self.apply_motors
        )
        self.create_timer(
            1.0 / self.get_parameter('map_rate').value, self.publish_map
        )
        self.publish_map()
        self.get_logger().info(
            f'ponte pronta. comandos em {self.resolve_topic_name("cmd_vel")}'
        )

    # =========================================================================
    #  Conexão e cena
    # =========================================================================

    def connect(self, host, port):
        """Conecta ao CoppeliaSim, explicando o motivo quando não dá."""
        try:
            self.client = RemoteAPIClient(host, port)
            return self.client.require('sim')
        except Exception:
            raise SystemExit(
                f'Não consegui falar com o CoppeliaSim em {host}:{port}.\n'
                '  - O simulador está aberto, com a cena pega_banana_potential_field?\n'
                '  - Ele responde? Um child script da cena preso em laço trava a\n'
                '    thread principal: a porta continua aberta, mas nenhuma chamada\n'
                '    é respondida. Pare a simulação e desabilite esse script.'
            )

    def find(self, path):
        """Busca um objeto e, se não achar, mostra o que existe na cena."""
        handle = self.sim.getObject(path, {'noError': True})
        if handle != -1:
            return handle
        existentes = [
            self.sim.getObjectAlias(h, 2)
            for h in self.sim.getObjectsInTree(self.sim.handle_scene)
        ]
        raise SystemExit(
            f'Objeto "{path}" não existe na cena aberta.\n'
            'Objetos disponíveis:\n  '
            + '\n  '.join(existentes)
            + '\nAjuste o parâmetro robot, por exemplo:\n'
            '  ros2 launch potential_field potential_field.launch.py robot:=/meuRobo'
        )

    def warn_motor_scripts(self, robot):
        """Avisa sobre scripts da cena que também escrevem nos motores.

        Os dois escrevem nas mesmas juntas: quem escrever por último vence, e o
        script da cena escreve a cada passo de simulação. Na cena desta
        atividade o candidato é o /myRobot/python_controler.
        """
        rivals = []
        for h in self.sim.getObjectsInTree(
            self.sim.getObject(robot), self.sim.sceneobject_script
        ):
            if self.sim.getBoolProperty(h, 'scriptDisabled'):
                continue
            if 'setJointTargetVelocity' in self.sim.getStringProperty(h, 'code'):
                rivals.append(self.sim.getObjectAlias(h, 2))
        if rivals:
            self.get_logger().warn(
                'estes scripts da cena escrevem nos motores a cada passo e vão '
                f'sobrescrever o cmd_vel: {", ".join(rivals)}. Desabilite-os na cena.'
            )

    def measure_wheel_separation(self):
        """Distância entre os dois motores, lida direto da cena."""
        left = self.sim.getObjectPosition(self.leftMotorHandle, self.sim.handle_world)
        right = self.sim.getObjectPosition(self.rightMotorHandle, self.sim.handle_world)
        return math.dist(left, right)

    # =========================================================================
    #  CoppeliaSim -> ROS
    # =========================================================================

    def read_signal(self, name):
        """Lê um sinal da cena e desempacota a tabela Lua.

        Devolve None enquanto o sinal não existir: ele só é criado pelo script
        /buildScene no início da simulação.

        Tem que ser getBufferSignal, e não getStringSignal: a tabela empacotada
        é binária, e tudo que volta do CoppeliaSim é codificado em CBOR. Uma
        string vira CBOR TEXT, que exige UTF-8 válido, e a leitura morre com
        "TEXT: not UTF-8 text". O getBufferSignal marca o valor como buffer, que
        vira CBOR BIN e passa intacto.
        """
        packed = self.sim.getBufferSignal(name)
        if not packed:
            return None
        return unpack_pairs(self.sim.unpackTable(packed))

    def publish_map(self):
        """Publica bananas e poops, quando os sinais da cena já existirem."""
        bananas = self.read_signal(self.banana_signal)
        poops = self.read_signal(self.poop_signal)
        if bananas is None or poops is None:
            if not self.map_warned:
                self.get_logger().warn(
                    f'sinais "{self.banana_signal}" e "{self.poop_signal}" ainda não '
                    'existem. Eles são criados pelo script /buildScene no início da '
                    'simulação: dê play no CoppeliaSim.'
                )
                self.map_warned = True
            return

        self.map_warned = False
        if (len(bananas), len(poops)) != self.map_seen:
            self.map_seen = (len(bananas), len(poops))
            self.get_logger().info(
                f'mapa: {len(bananas)} bananas e {len(poops)} poops lidos dos sinais'
            )

        self.banana_pub.publish(self.to_pose_array(bananas))
        self.poop_pub.publish(self.to_pose_array(poops))

    def to_pose_array(self, points):
        """Lista de (x, y) -> PoseArray, uma pose por ponto (orientação neutra)."""
        array = PoseArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = self.frame_id
        for x, y in points:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation.w = 1.0
            array.poses.append(pose)
        return array

    def read_pose(self):
        """(x, y, yaw) do robô no mundo, já no sentido do ROS.

        getObjectMatrix devolve a matriz 3x4 em ordem de linhas. A frente do
        robô nesta cena é o eixo +y dele, que é a segunda coluna da matriz:
        (m[1], m[5]). O yaw publicado é a direção desse eixo no mundo.
        """
        m = self.sim.getObjectMatrix(self.robotHandle, self.sim.handle_world)
        return m[3], m[7], math.atan2(m[5], m[1])

    def step(self):
        """Executado pelo timer da pose: lê a cena e publica no ROS."""
        x, y, yaw = self.read_pose()
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        # Quaternion de uma rotação só em torno do eixo vertical.
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(msg)

    # =========================================================================
    #  ROS -> CoppeliaSim
    # =========================================================================

    def cmd_vel_callback(self, msg: Twist):
        """Guarda as velocidades de roda pedidas pelo Twist; apply_motors aplica."""
        v = msg.linear.x    # m/s, positivo = para a frente
        w = msg.angular.z   # rad/s, positivo = virar à esquerda

        # Cinemática inversa do robô diferencial: cada roda está a L/2 do centro,
        # então sua velocidade linear é v ∓ w·L/2.
        half_track = self.wheel_separation / 2.0
        left = v - w * half_track     # m/s
        right = v + w * half_track    # m/s

        # Satura mantendo a proporção entre as rodas (mesma curva, mais lenta).
        # max_wheel_speed está em rad/s; vezes o raio dá o limite em m/s.
        limit = self.max_wheel_speed * self.wheel_radius
        excess = max(abs(left), abs(right)) / limit
        if excess > 1.0:
            left, right = left / excess, right / excess

        self.velocity = (left, right)
        self.deadline = time.monotonic() + self.cmd_timeout

    def apply_motors(self):
        """Executado pelo timer dos motores: repõe o último comando na cena."""
        # Sem comando recente o robô para sozinho, mesmo que o navegador caia.
        if time.monotonic() >= self.deadline:
            self.velocity = (0.0, 0.0)
        left, right = self.velocity
        self.write_motors(left, right)

    def write_motors(self, left, right):
        """Escreve nas juntas (esquerda, direita), em m/s na roda."""
        self.sim.setJointTargetVelocity(
            self.leftMotorHandle, self.motor_sign * left / self.wheel_radius
        )
        self.sim.setJointTargetVelocity(
            self.rightMotorHandle, self.motor_sign * right / self.wheel_radius
        )

    def stop(self):
        """Para os motores e, se foi a ponte que deu o play, para a simulação."""
        self.write_motors(0.0, 0.0)
        if self.started_here:
            self.sim.stopSimulation()


def main(args=None):
    # O Ctrl+C só levanta uma flag. Deixar o rclpy ou o KeyboardInterrupt cortarem
    # o laço no meio de uma chamada ZeroMQ invalida o socket, e aí o comando de
    # parada dos motores nunca chega ao simulador.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_requested.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_requested.set())

    try:
        node = CoppeliaBridge()
    except SystemExit as erro:
        print(erro)
        rclpy.shutdown()
        return 1
    try:
        while not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
