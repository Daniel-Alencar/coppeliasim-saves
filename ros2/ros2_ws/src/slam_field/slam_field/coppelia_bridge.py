"""Ponte ROS 2 <-> CoppeliaSim para a cena p3_slam_toolbox.

Expõe pelo ROS 2 o que o SLAM Toolbox e o navegador precisam, lendo a cena pela
ZeroMQ Remote API (porta 23000). Não é preciso o plugin simROS2.

    CoppeliaSim                                ROS 2
    sinal distSignal do laser 2D    --->       /scan          sensor_msgs/LaserScan
    pose do robô OU juntas das rodas --->      /odom          nav_msgs/Odometry
                                    --->       /tf            odom -> base_link
                                    --->       /tf_static     base_link -> laser
    matriz do /myRobot (gabarito)   --->       /ground_truth  geometry_msgs/PoseStamped
    sinal 'banana' (tabela Lua)     --->       /bananas       geometry_msgs/PoseArray
    motores                         <---       /cmd_vel       geometry_msgs/Twist
    banana some da cena             <---       /collected_banana  PointStamped
    simulação para                  <---       /done          std_msgs/Bool

A transformação map -> odom NÃO sai daqui: é o SLAM Toolbox que a publica,
corrigindo a odometria com o casamento dos scans.

O laser
-------
O script /myRobot/LaserScanner_2D gira um sensor de proximidade de -A/2 a +A/2
(A = scanningAngle, 180° na cena) com passo 1/D grau (D = scanningDensity, 2,5
na cena) e grava as distâncias na propriedade 'signal.distSignal' do robô. O
índice 0 é a direita do robô e o ângulo cresce no sentido anti-horário, como
no LaserScan. Sem retorno o script grava range_max + 1 (11 m); aqui isso vira
inf, como manda a REP 117.

A odometria
-----------
odom_source = 'ground_truth': a pose da cena, sem erro nenhum. É o primeiro
  passo do exercício, para separar problemas de SLAM de problemas de odometria.
odom_source = 'wheel': integração das juntas das rodas (dead reckoning), com o
  raio e a distância entre rodas. Começa na pose verdadeira do robô, para que
  odom (e portanto map) coincida com o mundo da cena no início: é isso que
  permite usar as posições das bananas direto no referencial map.

O /myRobot/odometry da cena não é usado: ele supõe que a frente do robô é o
eixo +x dele, e nesta cena a frente é o +y.

O eixo da frente do robô
------------------------
A frente do /myRobot é o eixo +y do referencial dele. No base_link publicado a
frente é +x, como é usual no ROS: yaw = atan2(m[5], m[1]).

Os motores
----------
Velocidade NEGATIVA na junta faz o robô andar para a frente (motor_sign = -1).
Os scripts /myRobot/controler e /myRobot/python_controler escrevem nos motores
a cada passo e precisam estar DESABILITADOS (estão, na cena entregue).
"""

import math
import signal
import threading
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import Pose, PoseArray, PointStamped, PoseStamped
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import tf2_ros


def wrap(angle):
    """Normaliza um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def unpack_pairs(flat):
    """[x1, y1, x2, y2, ...] -> [(x1, y1), (x2, y2), ...]."""
    if not flat:
        return []
    if isinstance(flat, dict):
        flat = [flat[k] for k in sorted(flat, key=lambda v: int(v))]
    return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]


def yaw_quaternion(yaw):
    """Quaternion (x, y, z, w) de uma rotação só em torno do eixo vertical."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class CoppeliaBridge(Node):
    """Nó ROS 2 que liga a cena do SLAM a tópicos ROS."""

    def __init__(self):
        super().__init__('coppelia_bridge')

        # --- Conexão e cena --------------------------------------------------
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 23000)
        self.declare_parameter('robot', '/myRobot')
        self.declare_parameter('laser_script', 'LaserScanner_2D')  # relativo ao robô
        self.declare_parameter('laser_signal', 'signal.distSignal')
        self.declare_parameter('banana_signal', 'banana')
        self.declare_parameter('banana_alias', 'Banana')
        self.declare_parameter('autostart', True)

        # --- Odometria -------------------------------------------------------
        self.declare_parameter('odom_source', 'ground_truth')   # ou 'wheel'
        self.declare_parameter('wheel_radius', 0.05)            # [m]
        self.declare_parameter('wheel_separation', 0.0)         # 0 = medir na cena
        self.declare_parameter('motor_sign', -1.0)

        # --- Laser -----------------------------------------------------------
        # Se os parâmetros do script não puderem ser lidos, valem estes.
        self.declare_parameter('scan_angle_deg', 180.0)
        self.declare_parameter('scan_density', 2.5)             # leituras por grau
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 10.0)
        # Posição do laser no base_link (x para a frente, y para a esquerda).
        # Medida na cena: 3 mm à frente do centro, feixe a 20 cm do chão.
        self.declare_parameter('laser_x', 0.003)
        self.declare_parameter('laser_y', 0.0)
        self.declare_parameter('laser_z', 0.20)

        # --- Ritmos e motores ------------------------------------------------
        # Cada chamada à Remote API custa ~12 ms com a simulação rodando. O
        # ciclo de sensores faz 4 chamadas (laser, pose, duas juntas), numa
        # conexão própria, em outra thread: ~50 ms, então 10 Hz é folgado.
        self.declare_parameter('sensor_rate', 10.0)
        self.declare_parameter('motor_rate', 20.0)
        self.declare_parameter('max_wheel_speed', 10.0)          # [rad/s]
        self.declare_parameter('cmd_timeout', 0.5)               # [s]

        # --- Referenciais ----------------------------------------------------
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('map_frame', 'map')

        # --- Bananas e fim ---------------------------------------------------
        # A cena não tem /dirt_script: nada some sozinho. Quando o navegador
        # alcança uma banana, a ponte a manda para z = 1000 (fica visível no
        # vídeo). Com stop_on_done, a ponte para a simulação no fim.
        self.declare_parameter('hide_collected', True)
        self.declare_parameter('stop_on_done', True)

        p = self.get_parameter
        host, port = p('host').value, p('port').value
        robot = p('robot').value
        self.odom_source = p('odom_source').value
        if self.odom_source not in ('ground_truth', 'wheel'):
            raise SystemExit(
                f'odom_source="{self.odom_source}" inválido: use ground_truth ou wheel')
        self.wheel_radius = p('wheel_radius').value
        self.motor_sign = p('motor_sign').value
        self.max_wheel_speed = p('max_wheel_speed').value
        self.cmd_timeout = p('cmd_timeout').value
        self.range_min = p('range_min').value
        self.range_max = p('range_max').value
        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value
        self.laser_frame = p('laser_frame').value
        self.map_frame = p('map_frame').value

        # --- Conexão principal: motores e bananas -----------------------------
        self.client, self.sim = self.connect(host, port)
        self.robotHandle = self.find(self.sim, robot)
        self.leftMotorHandle = self.find(self.sim, f'{robot}/leftMotor')
        self.rightMotorHandle = self.find(self.sim, f'{robot}/rightMotor')
        self.warn_motor_scripts(robot)

        self.wheel_separation = p('wheel_separation').value
        if self.wheel_separation <= 0.0:
            left = self.sim.getObjectPosition(self.leftMotorHandle, self.sim.handle_world)
            right = self.sim.getObjectPosition(self.rightMotorHandle, self.sim.handle_world)
            self.wheel_separation = math.dist(left, right)

        self.read_scan_geometry(robot)

        self.started_here = (
            p('autostart').value
            and self.sim.getSimulationState() == self.sim.simulation_stopped
        )
        if self.started_here:
            self.sim.startSimulation()
        elif self.sim.getSimulationState() == self.sim.simulation_paused:
            self.get_logger().warn('a simulação está pausada: nada muda até o play.')

        self.get_logger().info(
            f'robô {robot} | odometria: {self.odom_source} | roda r={self.wheel_radius:.3f} m'
            f', L={self.wheel_separation:.3f} m | laser {self.scan_count} leituras de '
            f'{math.degrees(self.angle_min):.1f}° a {math.degrees(self.angle_max):.1f}°'
        )

        # --- Estado ----------------------------------------------------------
        self.velocity = (0.0, 0.0)
        self.last_written = (None, None)
        self.last_write_time = 0.0
        self.deadline = 0.0
        self.done = threading.Event()
        self.banana_handles = None
        self.bananas_published = False
        self.banana_warned = False
        self.odom_pose = None          # (x, y, yaw) integrada pelas rodas
        self.last_joints = None        # (esquerda, direita) [rad]
        self.last_odom_time = None

        # --- Comunicação ROS -------------------------------------------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.truth_pub = self.create_publisher(PoseStamped, 'ground_truth', 10)
        self.banana_pub = self.create_publisher(PoseArray, 'bananas', latched)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(
            PointStamped, 'collected_banana', self.collected_callback, 10)
        self.create_subscription(Bool, 'done', self.done_callback, latched)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_laser_tf()

        self.create_timer(1.0 / p('motor_rate').value, self.apply_motors)
        self.create_timer(1.0, self.publish_bananas)

        # Sensores numa thread com conexão própria: o socket da Remote API não
        # pode ser dividido entre threads, e assim os motores não esperam o
        # laser (nem o contrário).
        self.sensor_client = RemoteAPIClient(host, port)
        self.sim_sensor = self.sensor_client.require('sim')
        self.sensor_stop = threading.Event()
        self.sensor_thread = threading.Thread(target=self.sensor_loop, daemon=True)
        self.sensor_thread.start()

        self.get_logger().info(
            f'ponte pronta. comandos em {self.resolve_topic_name("cmd_vel")}')

    # =========================================================================
    #  Conexão e cena
    # =========================================================================

    def connect(self, host, port):
        try:
            client = RemoteAPIClient(host, port)
            return client, client.require('sim')
        except Exception:
            raise SystemExit(
                f'Não consegui falar com o CoppeliaSim em {host}:{port}.\n'
                '  - O simulador está aberto, com a cena p3_slam_toolbox.ttt?\n'
                '  - Um child script preso em laço trava o simulador: a porta\n'
                '    continua aberta, mas nenhuma chamada é respondida.'
            )

    def find(self, sim, path):
        handle = sim.getObject(path, {'noError': True})
        if handle != -1:
            return handle
        existentes = [sim.getObjectAlias(h, 2)
                      for h in sim.getObjectsInTree(sim.handle_scene)]
        raise SystemExit(
            f'Objeto "{path}" não existe na cena aberta.\nObjetos disponíveis:\n  '
            + '\n  '.join(existentes))

    def warn_motor_scripts(self, robot):
        """Avisa sobre scripts habilitados da cena que escrevem nos motores."""
        rivals = []
        for h in self.sim.getObjectsInTree(
                self.sim.getObject(robot), self.sim.sceneobject_script):
            if self.sim.getBoolProperty(h, 'scriptDisabled'):
                continue
            if 'setJointTargetVelocity' in self.sim.getStringProperty(h, 'code'):
                rivals.append(self.sim.getObjectAlias(h, 2))
        if rivals:
            self.get_logger().warn(
                'estes scripts da cena escrevem nos motores a cada passo e vão '
                f'sobrescrever o cmd_vel: {", ".join(rivals)}. Desabilite-os na cena.')

    def read_scan_geometry(self, robot):
        """Ângulo e densidade do varrimento, lidos do script do laser.

        São os parâmetros de simulação scanningAngle e scanningDensity do
        script /myRobot/LaserScanner_2D, com o mesmo limite que ele aplica.
        Se não der para ler, valem os parâmetros scan_angle_deg e scan_density.
        """
        p = self.get_parameter
        angle, density = p('scan_angle_deg').value, p('scan_density').value
        try:
            # O script é um objeto filho do LaserScanner_2D: acha pelo código.
            laser = self.find(self.sim, f'{robot}/{p("laser_script").value}')
            script = next(
                h for h in self.sim.getObjectsInTree(laser, self.sim.sceneobject_script)
                if 'distSignal' in self.sim.getStringProperty(h, 'code'))
            a = self.sim.getScriptSimulationParameter(script, 'scanningAngle')
            d = self.sim.getScriptSimulationParameter(script, 'scanningDensity')
            if a is not None:
                angle = min(180.0, max(5.0, float(a)))
            if d is not None:
                density = min(5.0, max(0.1, float(d)))
        except Exception as erro:
            self.get_logger().warn(
                f'não li os parâmetros do laser ({erro}); usando {angle}° e {density}/°')
        # Mesmas contas do script: começa em -A/2 e soma pi/(D*180) por leitura.
        # O laço dele vai de 0 a A*D inclusive e insere uma leitura a mais no
        # fim, então são A*D + 2 leituras.
        self.angle_min = -angle * math.pi / 360.0
        self.angle_increment = math.pi / (density * 180.0)
        self.scan_count = int(angle * density) + 2
        self.angle_max = self.angle_min + (self.scan_count - 1) * self.angle_increment

    # =========================================================================
    #  Sensores (thread própria)
    # =========================================================================

    def sensor_loop(self):
        periodo = 1.0 / self.get_parameter('sensor_rate').value
        while not self.sensor_stop.is_set():
            inicio = time.monotonic()
            try:
                self.sensor_step(self.sim_sensor)
            except Exception as erro:   # simulação parando, conexão fechando
                if not self.sensor_stop.is_set():
                    self.get_logger().warn(f'sensores: {erro}', throttle_duration_sec=5.0)
            self.sensor_stop.wait(max(0.0, periodo - (time.monotonic() - inicio)))

    def sensor_step(self, sim):
        """Lê laser, pose e juntas e publica tudo com o mesmo carimbo de tempo.

        O SLAM Toolbox casa cada scan com a TF odom -> base_link do instante do
        scan. Publicar os dois com o mesmo carimbo evita que ele precise
        interpolar (ou descartar o scan por falta de TF).
        """
        ranges = sim.getFloatArrayProperty(
            self.robotHandle, self.get_parameter('laser_signal').value, {'noError': True})
        m = sim.getObjectMatrix(self.robotHandle, sim.handle_world)
        joints = None
        if self.odom_source == 'wheel':
            joints = (sim.getJointPosition(self.leftMotorHandle),
                      sim.getJointPosition(self.rightMotorHandle))
        now = self.get_clock().now()
        stamp = now.to_msg()

        truth = (m[3], m[7], math.atan2(m[5], m[1]))
        self.publish_truth(truth, stamp)
        x, y, yaw, v, w = self.update_odometry(truth, joints, now.nanoseconds * 1e-9)
        self.publish_odometry(x, y, yaw, v, w, stamp)
        if ranges:
            self.publish_scan(ranges, stamp)

    def update_odometry(self, truth, joints, t):
        """Pose (x, y, yaw) e velocidades (v, w) da fonte escolhida."""
        if self.odom_source == 'ground_truth':
            if self.odom_pose is None or self.last_odom_time is None:
                v = w = 0.0
            else:
                dt = max(t - self.last_odom_time, 1e-6)
                dx, dy = truth[0] - self.odom_pose[0], truth[1] - self.odom_pose[1]
                v = (dx * math.cos(truth[2]) + dy * math.sin(truth[2])) / dt
                w = wrap(truth[2] - self.odom_pose[2]) / dt
            self.odom_pose = truth
            self.last_odom_time = t
            return truth[0], truth[1], truth[2], v, w

        # Rodas: começa na pose verdadeira (map = odom = mundo no início).
        if self.odom_pose is None:
            self.odom_pose = truth
            self.last_joints = joints
            self.last_odom_time = t
            return truth[0], truth[1], truth[2], 0.0, 0.0

        # As juntas são cíclicas (voltam a -pi depois de pi): o incremento é
        # normalizado. A 10 rad/s e 10 Hz são 1 rad por ciclo, longe de pi.
        dl = wrap(joints[0] - self.last_joints[0])
        dr = wrap(joints[1] - self.last_joints[1])
        self.last_joints = joints
        sl = self.motor_sign * dl * self.wheel_radius   # avanço da roda [m]
        sr = self.motor_sign * dr * self.wheel_radius
        ds = (sl + sr) / 2.0
        dth = (sr - sl) / self.wheel_separation

        x, y, yaw = self.odom_pose
        # Integração pelo ponto médio do arco: melhor que Euler nas curvas.
        x += ds * math.cos(yaw + dth / 2.0)
        y += ds * math.sin(yaw + dth / 2.0)
        yaw = wrap(yaw + dth)
        self.odom_pose = (x, y, yaw)
        dt = max(t - self.last_odom_time, 1e-6)
        self.last_odom_time = t
        return x, y, yaw, ds / dt, dth / dt

    def publish_odometry(self, x, y, yaw, v, w, stamp):
        q = yaw_quaternion(yaw)
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)
        self.odom_pub.publish(odom)

    def publish_truth(self, truth, stamp):
        """Pose verdadeira no mundo, só para comparar (o controle não usa)."""
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.position.x, msg.pose.position.y = truth[0], truth[1]
        q = yaw_quaternion(truth[2])
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        self.truth_pub.publish(msg)

    def publish_scan(self, ranges, stamp):
        if len(ranges) != self.scan_count:
            # O script mudou de parâmetros com a simulação rodando: refaz o
            # passo angular a partir do número de leituras.
            span = self.angle_max - self.angle_min
            self.scan_count = len(ranges)
            self.angle_increment = span / max(1, self.scan_count - 1)
            self.get_logger().warn(f'laser agora com {self.scan_count} leituras')
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.laser_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + (len(ranges) - 1) * self.angle_increment
        scan.angle_increment = self.angle_increment
        # O script varre tudo dentro de um único passo de simulação.
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.get_parameter('sensor_rate').value
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [float(r) if r <= self.range_max else math.inf for r in ranges]
        self.scan_pub.publish(scan)

    def publish_laser_tf(self):
        p = self.get_parameter
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.base_frame
        tf.child_frame_id = self.laser_frame
        tf.transform.translation.x = p('laser_x').value
        tf.transform.translation.y = p('laser_y').value
        tf.transform.translation.z = p('laser_z').value
        tf.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(tf)

    # =========================================================================
    #  Bananas
    # =========================================================================

    def publish_bananas(self):
        """Publica as bananas do sinal da cena (gabarito), uma vez.

        O sinal só existe depois do play (o /buildScene o cria). As posições
        estão no mundo da cena, que é o referencial map (ver odometria).
        """
        if self.bananas_published:
            return
        packed = self.sim.getBufferSignal(self.get_parameter('banana_signal').value)
        if not packed:
            if not self.banana_warned:
                self.banana_warned = True
                self.get_logger().warn(
                    'sinal "banana" ainda não existe: ele é criado pelo /buildScene '
                    'no play da simulação.')
            return
        bananas = unpack_pairs(self.sim.unpackTable(packed))
        array = PoseArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = self.map_frame
        for bx, by in bananas:
            pose = Pose()
            pose.position.x, pose.position.y = bx, by
            pose.orientation.w = 1.0
            array.poses.append(pose)
        self.banana_pub.publish(array)
        self.bananas_published = True
        self.get_logger().info(f'{len(bananas)} bananas publicadas em /bananas')

    def collected_callback(self, msg: PointStamped):
        """Tira da cena a banana mais próxima do ponto alcançado."""
        if not self.get_parameter('hide_collected').value:
            return
        if self.banana_handles is None:
            alias = self.get_parameter('banana_alias').value
            self.banana_handles = [h for h in self.sim.getObjectsInTree(self.sim.handle_scene)
                                   if self.sim.getObjectAlias(h) == alias]
        melhor, menor = None, 0.3
        for h in self.banana_handles:
            bx, by, bz = self.sim.getObjectPosition(h, self.sim.handle_world)
            d = math.hypot(bx - msg.point.x, by - msg.point.y)
            if bz < 100.0 and d < menor:
                melhor, menor = h, d
        if melhor is not None:
            pos = self.sim.getObjectPosition(melhor, self.sim.handle_world)
            self.sim.setObjectPosition(melhor, [pos[0], pos[1], 1000.0], self.sim.handle_world)

    def done_callback(self, msg: Bool):
        if msg.data and self.get_parameter('stop_on_done').value:
            self.get_logger().info('navegador terminou: parando a simulação')
            self.done.set()

    # =========================================================================
    #  Motores
    # =========================================================================

    def cmd_vel_callback(self, msg: Twist):
        v, w = msg.linear.x, msg.angular.z
        half_track = self.wheel_separation / 2.0
        left, right = v - w * half_track, v + w * half_track
        limit = self.max_wheel_speed * self.wheel_radius
        excess = max(abs(left), abs(right)) / limit
        if excess > 1.0:
            left, right = left / excess, right / excess
        self.velocity = (left, right)
        self.deadline = time.monotonic() + self.cmd_timeout

    def apply_motors(self):
        if time.monotonic() >= self.deadline:
            self.velocity = (0.0, 0.0)
        left, right = self.velocity
        agora = time.monotonic()
        if (left, right) != self.last_written or agora - self.last_write_time > 1.0:
            self.write_motors(left, right)
            self.last_written = (left, right)
            self.last_write_time = agora

    def write_motors(self, left, right):
        self.sim.setJointTargetVelocity(
            self.leftMotorHandle, self.motor_sign * left / self.wheel_radius)
        self.sim.setJointTargetVelocity(
            self.rightMotorHandle, self.motor_sign * right / self.wheel_radius)

    def stop(self):
        self.sensor_stop.set()
        self.sensor_thread.join(timeout=2.0)
        self.write_motors(0.0, 0.0)
        if self.started_here or self.done.is_set():
            self.sim.stopSimulation()


def main(args=None):
    # O Ctrl+C só levanta uma flag: cortar uma chamada ZeroMQ no meio invalida
    # o socket, e aí a parada dos motores nunca chega ao simulador.
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
        while not stop_requested.is_set() and not node.done.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
