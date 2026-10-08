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
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped, Twist
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros


def matrix_to_quaternion(m):
    """Quaternion (x, y, z, w) da matriz 3x4 do CoppeliaSim, em ordem de linhas."""
    traco = m[0] + m[5] + m[10]
    if traco > 0.0:
        s = math.sqrt(traco + 1.0) * 2.0
        return ((m[9] - m[6]) / s, (m[2] - m[8]) / s, (m[4] - m[1]) / s, 0.25 * s)
    if m[0] > m[5] and m[0] > m[10]:
        s = math.sqrt(1.0 + m[0] - m[5] - m[10]) * 2.0
        return (0.25 * s, (m[1] + m[4]) / s, (m[2] + m[8]) / s, (m[9] - m[6]) / s)
    if m[5] > m[10]:
        s = math.sqrt(1.0 + m[5] - m[0] - m[10]) * 2.0
        return ((m[1] + m[4]) / s, 0.25 * s, (m[6] + m[9]) / s, (m[2] - m[8]) / s)
    s = math.sqrt(1.0 + m[10] - m[0] - m[5]) * 2.0
    return ((m[2] + m[8]) / s, (m[6] + m[9]) / s, 0.25 * s, (m[4] - m[1]) / s)


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

        # --- De onde vem o mapa ----------------------------------------------
        # 'signals': lê as posições prontas dos sinais da cena (gabarito).
        # 'perception': não publica mapa nenhum; quem monta é o perception_map,
        #   a partir do que a câmera vê. É o modo do exercício com YOLO.
        self.declare_parameter('map_source', 'signals')

        # --- Câmera e TF ------------------------------------------------------
        # Com 'perception' a ponte também publica as imagens da Kinect da cena e
        # a TF mundo -> robô -> câmera, que o perception_map usa para levar as
        # detecções da câmera para o mundo.
        self.declare_parameter('publish_images', False)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('rgb_sensor', 'kinect/rgb')     # relativo ao robô
        self.declare_parameter('depth_sensor', 'kinect/depth')
        self.declare_parameter('image_rate', 5.0)              # [Hz]
        # A Kinect da cena renderiza a cada passo (20 vezes por segundo), mas a
        # ponte só usa image_rate imagens por segundo. Medido: só a renderização
        # deixa a simulação a 0,52x do tempo real; renderizando sob demanda, a
        # 5 Hz, ela vai a 0,94x. O tratamento original é restaurado ao sair.
        self.declare_parameter('render_on_demand', True)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')

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
        self.map_source = self.get_parameter('map_source').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        # No modo percepção as imagens são obrigatórias: sem elas não há detecção.
        self.publish_images = (
            self.get_parameter('publish_images').value or self.map_source == 'perception'
        )

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
        self.last_written = (None, None)  # último comando escrito nas juntas
        self.last_write_time = 0.0
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

        # Câmera e TF, usados pelo caminho da percepção.
        self.rgb_pub = self.create_publisher(Image, 'rgb/image', 1)
        self.depth_pub = self.create_publisher(Image, 'depth/image', 1)
        self.info_pub = self.create_publisher(CameraInfo, 'rgb/camera_info', 1)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.rgbHandle = -1
        self.depthHandle = -1
        self.camera_info = None
        self.camera_pose = None
        if self.publish_images:
            self.setup_camera(robot)

        # Três timers: pose, motores e releitura do mapa têm ritmos diferentes.
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.create_timer(
            1.0 / self.get_parameter('motor_rate').value, self.apply_motors
        )
        if self.map_source == 'signals':
            self.create_timer(
                1.0 / self.get_parameter('map_rate').value, self.publish_map
            )
            self.publish_map()
        else:
            self.get_logger().info(
                'map_source=perception: a ponte não publica o mapa. '
                'Quem monta bananas e poops é o perception_map, pela câmera.'
            )
        # A câmera roda numa thread própria, com uma conexão própria ao
        # simulador. Medido: dentro do laço do ROS, cada ciclo de câmera
        # bloqueava ~150 ms, e a pose e os motores, que deveriam rodar a 20 Hz,
        # caíam para 2,7 Hz — o robô girava ~40° entre um comando e outro.
        # O socket da Remote API não pode ser dividido entre threads, por isso
        # a segunda conexão.
        self.camera_stop = threading.Event()
        self.camera_thread = None
        if self.publish_images:
            self.cam_client = RemoteAPIClient(
                self.get_parameter('host').value, self.get_parameter('port').value)
            self.sim_cam = self.cam_client.require('sim')
            self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
            self.camera_thread.start()
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

    def setup_camera(self, robot):
        """Acha os sensores da Kinect e monta o CameraInfo a partir da cena."""
        rgb = f"{robot}/{self.get_parameter('rgb_sensor').value}"
        depth = f"{robot}/{self.get_parameter('depth_sensor').value}"
        self.rgbHandle = self.find(rgb)
        self.depthHandle = self.find(depth)
        self.on_demand = self.get_parameter('render_on_demand').value
        self.original_handling = None
        if self.on_demand:
            self.original_handling = (
                self.sim.getExplicitHandling(self.rgbHandle),
                self.sim.getExplicitHandling(self.depthHandle),
            )
            self.sim.setExplicitHandling(self.rgbHandle, 1)
            self.sim.setExplicitHandling(self.depthHandle, 1)

        width, height = self.sim.getVisionSensorRes(self.rgbHandle)
        # No CoppeliaSim o ângulo de perspectiva é o da maior dimensão da imagem.
        angle = self.sim.getObjectFloatParam(
            self.rgbHandle, self.sim.visionfloatparam_perspective_angle)
        maior = max(width, height)
        f = (maior / 2.0) / math.tan(angle / 2.0)
        cx, cy = width / 2.0, height / 2.0

        info = CameraInfo()
        info.header.frame_id = self.camera_frame
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]   # a câmera simulada não distorce
        info.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info = info
        self.near = self.sim.getObjectFloatParam(
            self.depthHandle, self.sim.visionfloatparam_near_clipping)
        self.far = self.sim.getObjectFloatParam(
            self.depthHandle, self.sim.visionfloatparam_far_clipping)
        # O referencial do sensor do CoppeliaSim é o referencial óptico do ROS
        # girado 180° em torno do eixo óptico: medido nesta cena, o +x do sensor
        # aponta para a esquerda da imagem e o +y para cima. Publicar a TF já
        # com esse giro deixa camera_color_optical_frame ser um referencial
        # óptico de verdade (x para a direita, y para baixo, z para a frente),
        # e a projeção no yolo_vision fica a fórmula padrão.
        pos = self.sim.getObjectPosition(self.rgbHandle, self.robotHandle)
        qx, qy, qz, qw = self.sim.getObjectQuaternion(self.rgbHandle, self.robotHandle)
        self.camera_pose = (pos, (qy, -qx, qw, -qz))
        self.get_logger().info(
            f'câmera {width}x{height}, f={f:.1f} px, profundidade de '
            f'{self.near:.2f} a {self.far:.2f} m'
        )

    def camera_loop(self):
        """Publica as imagens no ritmo image_rate, fora do laço do controle."""
        periodo = 1.0 / self.get_parameter('image_rate').value
        while not self.camera_stop.is_set():
            inicio = time.monotonic()
            try:
                self.publish_camera()
            except Exception as erro:   # simulação parando, conexão fechando
                if not self.camera_stop.is_set():
                    self.get_logger().warn(
                        f'câmera: {erro}', throttle_duration_sec=5.0)
            self.camera_stop.wait(max(0.0, periodo - (time.monotonic() - inicio)))

    def publish_camera(self):
        """Publica imagem colorida, profundidade e a TF do mesmo instante.

        A TF sai daqui, e não do laço da pose, para carregar o mesmo carimbo de
        tempo da imagem. Com o robô girando, usar a pose de outro instante joga
        a detecção para o lado: a 0,5 rad/s, 100 ms de diferença já giram o
        referencial 3°, o que a 2 m vira 10 cm de erro — e o mapa se enche de
        cópias do mesmo objeto.

        Só três chamadas ao simulador: pose, imagem e profundidade. Virar a
        imagem e decodificar a profundidade é feito aqui, com numpy; pedir isso
        ao simulador (transformImage, unpackFloatTable) custava ~75 ms cada.
        """
        sim = self.sim_cam
        stamp = self.get_clock().now().to_msg()
        if self.get_parameter('publish_tf').value:
            m = sim.getObjectMatrix(self.robotHandle, sim.handle_world)
            self.publish_transforms(m[3], m[7], matrix_to_quaternion(m), stamp)

        if self.on_demand:
            sim.handleVisionSensor(self.rgbHandle)
            sim.handleVisionSensor(self.depthHandle)
        data, (width, height) = sim.getVisionSensorImg(self.rgbHandle)
        # O CoppeliaSim entrega a imagem de baixo para cima: vira as linhas.
        imagem = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)[::-1]
        rgb = Image()
        rgb.header.stamp = stamp
        rgb.header.frame_id = self.camera_frame
        rgb.height, rgb.width = height, width
        rgb.encoding = 'rgb8'
        rgb.is_bigendian = 0
        rgb.step = width * 3
        rgb.data = np.ascontiguousarray(imagem).tobytes()
        self.rgb_pub.publish(rgb)

        # Floats de 0 (near) a 1 (far), também de baixo para cima; vira metro.
        buf = sim.getVisionSensorDepthBuffer(self.depthHandle + sim.handleflag_codedstring)
        normal = np.frombuffer(buf, dtype=np.float32).reshape(height, width)[::-1]
        metros = (self.near + normal * (self.far - self.near)).astype(np.float32)
        depth = Image()
        depth.header.stamp = stamp
        depth.header.frame_id = self.camera_frame
        depth.height, depth.width = height, width
        depth.encoding = '32FC1'
        depth.is_bigendian = 0
        depth.step = width * 4
        depth.data = np.ascontiguousarray(metros).tobytes()
        self.depth_pub.publish(depth)

        self.camera_info.header.stamp = stamp
        self.info_pub.publish(self.camera_info)

    def publish_transforms(self, x, y, quat, stamp):
        """TF mundo -> robô -> câmera, para o mapa sair no referencial do mundo.

        A pose da câmera é lida da cena, então o referencial publicado é
        exatamente o do sensor: o eixo z aponta para onde ele olha, e é nesse
        referencial que o yolo_vision devolve os pontos.
        """
        base = TransformStamped()
        base.header.stamp = stamp
        base.header.frame_id = self.frame_id
        base.child_frame_id = self.base_frame
        base.transform.translation.x = x
        base.transform.translation.y = y
        base.transform.rotation.x = quat[0]
        base.transform.rotation.y = quat[1]
        base.transform.rotation.z = quat[2]
        base.transform.rotation.w = quat[3]
        envio = [base]

        if self.camera_pose is not None:
            # A câmera é fixa no robô: a pose dela foi lida uma vez, na partida.
            pos, rot = self.camera_pose
            cam = TransformStamped()
            cam.header.stamp = stamp
            cam.header.frame_id = self.base_frame
            cam.child_frame_id = self.camera_frame
            cam.transform.translation.x = pos[0]
            cam.transform.translation.y = pos[1]
            cam.transform.translation.z = pos[2]
            cam.transform.rotation.x = rot[0]
            cam.transform.rotation.y = rot[1]
            cam.transform.rotation.z = rot[2]
            cam.transform.rotation.w = rot[3]
            envio.append(cam)
        self.tf_broadcaster.sendTransform(envio)

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
        """(x, y, yaw, quaternion) do robô no mundo.

        getObjectMatrix devolve a matriz 3x4 em ordem de linhas. A frente do
        robô nesta cena é o eixo +y dele, que é a segunda coluna da matriz:
        (m[1], m[5]). O yaw publicado é a direção desse eixo no mundo.

        O quaternion é a rotação inteira do robô, tirada da mesma matriz. Ele
        vai para a TF, e não o yaw: a TF precisa do referencial do robô como
        ele é na cena, para que a pose da câmera, medida nesse referencial,
        componha certo.
        """
        m = self.sim.getObjectMatrix(self.robotHandle, self.sim.handle_world)
        return m[3], m[7], math.atan2(m[5], m[1]), matrix_to_quaternion(m)

    def step(self):
        """Executado pelo timer da pose: lê a cena e publica no ROS."""
        x, y, yaw, quat = self.read_pose()
        stamp = self.get_clock().now().to_msg()
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        # Quaternion de uma rotação só em torno do eixo vertical.
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(msg)

        # Com a câmera ligada, a TF sai junto da imagem (ver publish_camera).
        if self.get_parameter('publish_tf').value and not self.publish_images:
            self.publish_transforms(x, y, quat, stamp)

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
        # A velocidade alvo fica guardada na junta: reescrever o mesmo valor só
        # gasta chamadas, e cada chamada disputa o simulador com a pose e a
        # câmera. Escreve quando muda, e uma vez por segundo por garantia.
        agora = time.monotonic()
        if (left, right) != self.last_written or agora - self.last_write_time > 1.0:
            self.write_motors(left, right)
            self.last_written = (left, right)
            self.last_write_time = agora

    def write_motors(self, left, right):
        """Escreve nas juntas (esquerda, direita), em m/s na roda."""
        self.sim.setJointTargetVelocity(
            self.leftMotorHandle, self.motor_sign * left / self.wheel_radius
        )
        self.sim.setJointTargetVelocity(
            self.rightMotorHandle, self.motor_sign * right / self.wheel_radius
        )

    def stop(self):
        """Para a câmera, os motores e, se foi a ponte que deu o play, a simulação."""
        if self.camera_thread is not None:
            self.camera_stop.set()
            self.camera_thread.join(timeout=2.0)
        if getattr(self, 'original_handling', None) is not None:
            self.sim.setExplicitHandling(self.rgbHandle, self.original_handling[0])
            self.sim.setExplicitHandling(self.depthHandle, self.original_handling[1])
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
