"""Ponte ROS 2 <-> CoppeliaSim para a cena a_star_banana.

Tudo o que o planejador e o controlador de posição precisam da cena sai daqui,
pela ZeroMQ Remote API (porta 23000). Não é preciso o plugin simROS2, e nenhum
objeto tem coordenada fixa no código: as posições vêm de percorrer a
hierarquia da cena (/poops/poop[i], /bananas/Banana[i], /myRobot).

    CoppeliaSim                              ROS 2 (namespace myRobot)
    matriz do /myRobot            --->       pose           geometry_msgs/PoseStamped
                                  --->       /tf            world -> base_link
    relógio da simulação          --->       sim_time       std_msgs/Float64
    filhos de /poops              --->       poops          geometry_msgs/PoseArray
    filhos de /bananas            --->       bananas        geometry_msgs/PoseArray
                                  --->       banana_paths   std_msgs/String (JSON)
    placar do /dirt_script        --->       collected      std_msgs/Int32
    motores                       <---       cmd_vel        geometry_msgs/Twist
    parar a simulação             <---       done           std_msgs/Bool

A ponte não planeja e não controla: ela é só a comunicação. O A* está no nó
planner e a lei de controle no nó position_control, e nenhum dos dois conhece
o CoppeliaSim.

As bananas e o placar
---------------------
Quem recolhe a banana é a própria cena: o /dirt_script teleporta para z = 1000
o que passar sob o sensor /myRobot/poopximity e soma um ponto. Então a lista
publicada em `bananas` é a das bananas que ainda estão no chão (z < 100), e
`collected` é quantas sumiram. É assim que o planner sabe que chegou.

O mesmo script espalha poop se o robô pisar num: por isso o caminho é
planejado com os poops inflados, e não em cima deles.

O eixo da frente do robô
------------------------
A frente do /myRobot é o eixo +y do referencial dele, e não o +x como é usual
no ROS. A conversão é feita aqui (ver read_pose): o que sai no tópico pose já
é o rumo no sentido do ROS, com yaw = 0 apontando para o +x do mundo.

Os motores
----------
As juntas estão montadas de forma que velocidade NEGATIVA faz o robô andar
para a frente (motor_sign = -1). O script /myRobot/python_controler da cena
escreve nos motores a cada passo e anularia o cmd_vel; a ponte o desabilita ao
subir e o devolve ao estado original ao sair (ver disable_rival_scripts).
"""

import json
import math
import signal
import threading
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64, Int32, String
import tf2_ros

# A cena manda para z = 1000 o que o sensor recolhe; qualquer coisa acima disto
# está fora do chão.
COLLECTED_Z = 100.0


def matrix_to_quaternion(m):
    """Quaternion (x, y, z, w) da matriz 3x4 do CoppeliaSim, em ordem de linhas."""
    trace = m[0] + m[5] + m[10]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((m[9] - m[6]) / s, (m[2] - m[8]) / s, (m[4] - m[1]) / s, 0.25 * s)
    if m[0] > m[5] and m[0] > m[10]:
        s = math.sqrt(1.0 + m[0] - m[5] - m[10]) * 2.0
        return (0.25 * s, (m[1] + m[4]) / s, (m[2] + m[8]) / s, (m[9] - m[6]) / s)
    if m[5] > m[10]:
        s = math.sqrt(1.0 + m[5] - m[0] - m[10]) * 2.0
        return ((m[1] + m[4]) / s, 0.25 * s, (m[6] + m[9]) / s, (m[2] - m[8]) / s)
    s = math.sqrt(1.0 + m[10] - m[0] - m[5]) * 2.0
    return ((m[2] + m[8]) / s, (m[6] + m[9]) / s, 0.25 * s, (m[4] - m[1]) / s)


class CoppeliaBridge(Node):
    """Nó ROS 2 que liga a cena do A* a tópicos ROS."""

    def __init__(self):
        super().__init__('coppelia_bridge')

        # --- Conexão e cena --------------------------------------------------
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 23000)
        self.declare_parameter('robot', '/myRobot')
        self.declare_parameter('poops_path', '/poops')
        self.declare_parameter('bananas_path', '/bananas')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('autostart', True)
        # O python_controler da cena escreve nos motores a cada passo. Sem
        # desabilitá-lo, o cmd_vel não chega às rodas.
        self.declare_parameter('disable_rival_scripts', True)

        # --- Robô ------------------------------------------------------------
        self.declare_parameter('wheel_radius', 0.05)     # [m]
        self.declare_parameter('wheel_separation', 0.0)  # 0 = medir na cena
        self.declare_parameter('motor_sign', -1.0)
        self.declare_parameter('max_wheel_speed', 10.0)  # [rad/s] por roda
        self.declare_parameter('cmd_timeout', 0.5)       # [s] sem comando = parar
        # Escrever nos motores custa duas chamadas. Mudanças menores que isto
        # não valem o gasto: a roda nem sente 1 cm/s de diferença.
        self.declare_parameter('motor_deadband', 0.01)   # [m/s] na roda

        # --- Ritmo -----------------------------------------------------------
        # Com a simulação em curso livre, cada chamada da Remote API espera a
        # vez entre dois passos de simulação, e quantas são atendidas por passo
        # depende da carga do simulador. Medido nesta cena (que renderiza a
        # Kinect a cada passo): a pose saía a 5 Hz com buracos de 1,2 s, e o
        # controlador parava o robô achando que a ponte tinha caído.
        #
        # Em stepping a ponte é quem dá os passos: a cada ciclo ela lê a pose,
        # recebe o cmd_vel, escreve nos motores e só então pede o passo. O laço
        # de controle passa a ser exato — um comando por passo de 50 ms — e o
        # ritmo do simulador deixa de depender da máquina. É o mesmo esquema do
        # script autônomo do projeto 8.
        self.declare_parameter('stepping', True)
        # Só valem em curso livre (stepping=false): frequências dos timers.
        self.declare_parameter('rate', 10.0)        # pose [Hz]
        self.declare_parameter('motor_rate', 10.0)  # motores [Hz]
        # Os poops são lidos uma única vez, na partida: eles não se mexem (e um
        # poop atropelado continua marcado no mapa, que é o lado seguro).
        self.declare_parameter('map_rate', 3.0)     # bananas [Hz]

        # --- Fim da corrida ---------------------------------------------------
        self.declare_parameter('stop_on_done', True)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        robot = self.get_parameter('robot').value
        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.motor_sign = self.get_parameter('motor_sign').value
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # --- Conexão ----------------------------------------------------------
        self.sim = self.connect(host, port)
        self.robotHandle = self.find(robot)
        self.leftMotorHandle = self.find(f'{robot}/leftMotor')
        self.rightMotorHandle = self.find(f'{robot}/rightMotor')
        self.silenced = self.silence_rival_scripts(robot)

        self.wheel_separation = self.get_parameter('wheel_separation').value
        if self.wheel_separation <= 0.0:
            left = self.sim.getObjectPosition(self.leftMotorHandle, self.sim.handle_world)
            right = self.sim.getObjectPosition(self.rightMotorHandle, self.sim.handle_world)
            self.wheel_separation = math.dist(left, right)

        # --- Objetos da cena ---------------------------------------------------
        # Os handles são descobertos uma vez, percorrendo a hierarquia; depois
        # só as posições são relidas, que é o que muda durante a simulação.
        self.poops = self.read_group(self.get_parameter('poops_path').value)
        self.bananas = self.read_group(self.get_parameter('bananas_path').value)
        self.poop_points = [self.position(o)[:2] for o in self.poops]
        # Posição de cada banana, relida uma de cada vez durante a corrida.
        # None quer dizer "a cena já recolheu esta".
        self.banana_points = [self.position(o)[:2] for o in self.bananas]
        self.next_banana = 0
        self.get_logger().info(
            f'robô {robot} | raio da roda {self.wheel_radius:.3f} m | '
            f'distância entre rodas {self.wheel_separation:.3f} m | '
            f'{len(self.poops)} poops e {len(self.bananas)} bananas na cena'
        )
        if not self.bananas:
            self.get_logger().warn(
                'nenhuma banana encontrada: confira o caminho bananas_path')

        # --- Estado da simulação -----------------------------------------------
        self.started_here = (
            self.get_parameter('autostart').value
            and self.sim.getSimulationState() == self.sim.simulation_stopped
        )
        if self.started_here:
            self.sim.startSimulation()
        elif self.sim.getSimulationState() == self.sim.simulation_paused:
            self.get_logger().warn('a simulação está pausada: nada muda até o play.')

        # --- Estado interno ----------------------------------------------------
        self.velocity = (0.0, 0.0)        # (esquerda, direita), em m/s na roda
        self.last_written = (None, None)
        self.last_write_time = 0.0
        self.deadline = 0.0               # quando o último cmd_vel expira
        self.collected = -1               # placar anterior, para não repetir log
        self.done = threading.Event()     # planner avisou que acabou

        # --- Comunicação ROS ----------------------------------------------------
        # Transient local: o planner recebe a última lista mesmo subindo depois.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.poop_pub = self.create_publisher(PoseArray, 'poops', map_qos)
        self.banana_pub = self.create_publisher(PoseArray, 'bananas', map_qos)
        self.paths_pub = self.create_publisher(String, 'banana_paths', map_qos)
        self.collected_pub = self.create_publisher(Int32, 'collected', map_qos)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        # O relógio da cena. Quem mede prazos (o planner) usa este tempo, e não
        # o do computador: com a simulação a 0,2x do tempo real — que é o que
        # acontece numa máquina sem GPU, com a Kinect da cena renderizando a
        # cada passo — um prazo de 25 s de relógio viraria 5 s de simulação.
        self.sim_time_pub = self.create_publisher(Float64, 'sim_time', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, 'done', self.done_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.step_time = self.sim.getFloatParam(
            self.sim.floatparam_simulation_time_step)
        self.sim_time = self.sim.getSimulationTime()
        self.stepping = self.get_parameter('stepping').value
        if self.stepping:
            # A cada passo de simulação (50 ms nesta cena) a ponte faz um ciclo
            # inteiro; só a leitura das bananas é espaçada, pelo rodízio.
            step_time = self.step_time
            self.object_period = max(
                1, round(1.0 / (step_time * self.get_parameter('map_rate').value)))
            self.client.setStepping(True)
            self.get_logger().info(
                f'modo stepping: um ciclo por passo de {step_time * 1000:.0f} ms '
                f'({1.0 / step_time:.0f} Hz de simulação)')
        else:
            self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
            self.create_timer(
                1.0 / self.get_parameter('motor_rate').value, self.apply_motors)
            self.create_timer(
                1.0 / self.get_parameter('map_rate').value, self.publish_objects)
        self.cycles = 0
        self.publish_objects()
        self.get_logger().info(
            f'ponte pronta. comandos em {self.resolve_topic_name("cmd_vel")}')

    def cycle(self):
        """Um passo de simulação, com o ciclo de controle inteiro dentro dele.

        A ordem importa: a pose sai primeiro, o nó dá tempo de o controlador
        responder com o cmd_vel daquela pose, e só então o comando vai para as
        juntas e o passo acontece. Assim cada passo da simulação usa um comando
        calculado a partir da pose daquele mesmo passo.
        """
        self.step()
        rclpy.spin_once(self, timeout_sec=0.05)   # espera o cmd_vel desta pose
        for _ in range(4):                        # e esvazia o que mais chegou
            rclpy.spin_once(self, timeout_sec=0.0)
        self.apply_motors()
        self.cycles += 1
        if self.cycles % self.object_period == 0:
            self.publish_objects()
        self.sim.step()
        # Em stepping o relógio da cena é sabido sem gastar chamada: cada passo
        # avança exatamente o passo de simulação.
        self.sim_time += self.step_time
        self.sim_time_pub.publish(Float64(data=self.sim_time))

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
                '  - O simulador está aberto com a cena a_star_banana.ttt?\n'
                '  - Ele responde? Um script da cena preso em laço trava a\n'
                '    thread principal: a porta continua aberta, mas nenhuma\n'
                '    chamada é respondida.'
            )

    def find(self, path):
        """Busca um objeto e, se não achar, mostra o que existe na cena."""
        handle = self.sim.getObject(path, {'noError': True})
        if handle != -1:
            return handle
        existing = [self.sim.getObjectAlias(h, 2)
                    for h in self.sim.getObjectsInTree(self.sim.handle_scene)]
        raise SystemExit(
            f'Objeto "{path}" não existe na cena aberta.\n'
            'Objetos disponíveis:\n  ' + '\n  '.join(existing))

    def read_group(self, group_path):
        """Handles e caminhos dos filhos diretos de um grupo da hierarquia.

        É o que o enunciado pede na tarefa 1: descobrir os objetos percorrendo
        a cena, pelo caminho da API (/bananas/Banana[0]), em vez de escrever
        coordenadas no código.
        """
        group = self.sim.getObject(group_path, {'noError': True})
        if group == -1:
            self.get_logger().warn(f'grupo {group_path} não existe na cena')
            return []
        objects = []
        index = 0
        while True:
            child = self.sim.getObjectChild(group, index)
            if child == -1:
                break
            objects.append({'handle': child,
                            'path': self.sim.getObjectAlias(child, 2)})
            index += 1
        return objects

    def silence_rival_scripts(self, robot):
        """Desabilita os scripts da cena que também escrevem nos motores.

        Os dois escreveriam nas mesmas juntas, e quem escreve por último vence:
        o script da cena roda a cada passo de simulação, então ele venceria
        sempre. Nesta cena o candidato é o /myRobot/python_controler (o
        joystick). A mudança é só na cena carregada em memória, e é desfeita
        em stop(); o arquivo .ttt no disco não é tocado.
        """
        silenced = []
        for handle in self.sim.getObjectsInTree(
                self.sim.getObject(robot), self.sim.sceneobject_script):
            if self.sim.getBoolProperty(handle, 'scriptDisabled'):
                continue
            if 'setJointTargetVelocity' not in self.sim.getStringProperty(handle, 'code'):
                continue
            alias = self.sim.getObjectAlias(handle, 2)
            if not self.get_parameter('disable_rival_scripts').value:
                self.get_logger().warn(
                    f'{alias} escreve nos motores a cada passo e vai sobrescrever '
                    'o cmd_vel. Desabilite-o na cena.')
                continue
            self.sim.setBoolProperty(handle, 'scriptDisabled', True)
            silenced.append(handle)
            self.get_logger().info(
                f'{alias} desabilitado enquanto a ponte estiver rodando '
                '(ele escreveria nos motores a cada passo)')
        return silenced

    # =========================================================================
    #  CoppeliaSim -> ROS
    # =========================================================================

    def publish_objects(self):
        """Publica poops, bananas ainda no chão e o placar da cena.

        Só UMA banana é relida por ciclo, em rodízio. Ler as onze de uma vez
        custaria onze chamadas seguidas, e durante esse tempo a pose não sai —
        medido: a pose caía de 10 Hz para 5 Hz, com buracos de 1,1 s, e o
        controlador parava o robô achando que a ponte tinha caído. Em rodízio,
        a volta inteira leva poucos segundos, que é tempo de sobra para notar
        que a cena recolheu a banana.
        """
        if not self.stepping:
            # Em curso livre o relógio da cena custa uma chamada: lê aqui, no
            # ritmo mais lento, e não a cada pose.
            self.sim_time = self.sim.getSimulationTime()
            self.sim_time_pub.publish(Float64(data=self.sim_time))

        if self.bananas:
            index = self.next_banana % len(self.bananas)
            self.next_banana = index + 1
            if self.banana_points[index] is not None:
                x, y, z = self.position(self.bananas[index])
                self.banana_points[index] = None if z >= COLLECTED_Z else (x, y)

        on_floor = [(banana, point)
                    for banana, point in zip(self.bananas, self.banana_points)
                    if point is not None]

        self.poop_pub.publish(self.to_pose_array(self.poop_points))
        self.banana_pub.publish(self.to_pose_array([xy for _, xy in on_floor]))
        # PoseArray não carrega nomes: os caminhos vão num tópico ao lado, na
        # mesma ordem, para o planner poder pedir uma banana pelo caminho da
        # API (goal:='/bananas/Banana[4]').
        self.paths_pub.publish(
            String(data=json.dumps([banana['path'] for banana, _ in on_floor])))

        collected = len(self.bananas) - len(on_floor)
        if collected != self.collected:
            self.collected = collected
            self.collected_pub.publish(Int32(data=collected))
            if collected:
                self.get_logger().info(
                    f'placar da cena: {collected}/{len(self.bananas)} bananas')

    def position(self, obj):
        return self.sim.getObjectPosition(obj['handle'], self.sim.handle_world)

    def to_pose_array(self, points):
        """Lista de (x, y) -> PoseArray, uma pose por ponto."""
        array = PoseArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.header.frame_id = self.frame_id
        for x, y in points:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.orientation.w = 1.0
            array.poses.append(pose)
        return array

    def read_pose(self):
        """(x, y, yaw, quaternion) do robô no mundo.

        getObjectMatrix devolve a matriz 3x4 em ordem de linhas. A frente do
        robô nesta cena é o eixo +y dele, que é a segunda coluna da matriz:
        (m[1], m[5]). O yaw publicado é a direção desse eixo no mundo, de modo
        que, no tópico pose, a frente do robô é o +x — a convenção do ROS e a
        que o controlador de posição espera.
        """
        m = self.sim.getObjectMatrix(self.robotHandle, self.sim.handle_world)
        return m[3], m[7], math.atan2(m[5], m[1]), matrix_to_quaternion(m)

    def step(self):
        """Timer da pose: lê a cena e publica pose e TF."""
        x, y, yaw, _ = self.read_pose()
        stamp = self.get_clock().now().to_msg()

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(msg)

        # A TF leva o mesmo rumo da pose (frente em +x), para o rviz desenhar o
        # robô no sentido em que ele anda.
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.frame_id
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = msg.pose.orientation.z
        tf.transform.rotation.w = msg.pose.orientation.w
        self.tf_broadcaster.sendTransform(tf)

    # =========================================================================
    #  ROS -> CoppeliaSim
    # =========================================================================

    def cmd_vel_callback(self, msg: Twist):
        """Guarda as velocidades de roda pedidas pelo Twist; apply_motors aplica."""
        v = msg.linear.x    # m/s, positivo para a frente
        w = msg.angular.z   # rad/s, positivo para a esquerda

        # Cinemática inversa do diferencial: cada roda está a L/2 do centro.
        half_track = self.wheel_separation / 2.0
        left = v - w * half_track
        right = v + w * half_track

        # Satura mantendo a proporção entre as rodas: mesma curva, mais devagar.
        limit = self.max_wheel_speed * self.wheel_radius
        excess = max(abs(left), abs(right)) / limit
        if excess > 1.0:
            left, right = left / excess, right / excess

        self.velocity = (left, right)
        self.deadline = time.monotonic() + self.cmd_timeout

    def done_callback(self, msg: Bool):
        """O planner avisou que terminou (ou desistiu)."""
        if msg.data and not self.done.is_set():
            self.get_logger().info('planner terminou: encerrando a ponte')
            self.done.set()

    def apply_motors(self):
        """Timer dos motores: repõe o último comando na cena."""
        # Sem comando recente o robô para sozinho, mesmo que o controlador caia.
        if time.monotonic() >= self.deadline:
            self.velocity = (0.0, 0.0)
        left, right = self.velocity
        # A velocidade alvo fica guardada na junta: reescrever o mesmo valor só
        # gasta chamadas. Escreve quando muda mais que a zona morta, e uma vez
        # por segundo por garantia.
        now = time.monotonic()
        deadband = self.get_parameter('motor_deadband').value
        changed = (self.last_written[0] is None
                   or abs(left - self.last_written[0]) > deadband
                   or abs(right - self.last_written[1]) > deadband
                   # parar é sempre exato: nada de ficar andando devagarinho
                   or ((left, right) == (0.0, 0.0) and self.last_written != (0.0, 0.0)))
        if changed or now - self.last_write_time > 1.0:
            self.write_motors(left, right)
            self.last_written = (left, right)
            self.last_write_time = now

    def write_motors(self, left, right):
        """Escreve nas juntas (esquerda, direita), em m/s na roda."""
        self.sim.setJointTargetVelocity(
            self.leftMotorHandle, self.motor_sign * left / self.wheel_radius)
        self.sim.setJointTargetVelocity(
            self.rightMotorHandle, self.motor_sign * right / self.wheel_radius)

    def stop(self):
        """Para os motores, devolve os scripts da cena e fecha a simulação."""
        try:
            self.write_motors(0.0, 0.0)
            if self.stepping:
                # Sem isto a simulação ficaria esperando um passo que ninguém
                # mais vai pedir, e o simulador pareceria travado.
                self.sim.step()
                self.client.setStepping(False)
            for handle in self.silenced:
                self.sim.setBoolProperty(handle, 'scriptDisabled', False)
            if self.started_here or (self.done.is_set()
                                     and self.get_parameter('stop_on_done').value):
                self.sim.stopSimulation()
        except Exception as error:
            self.get_logger().warn(f'na saída: {error}')


def main(args=None):
    # O Ctrl+C só levanta uma flag. Deixar o rclpy cortar o laço no meio de uma
    # chamada ZeroMQ invalida o socket, e aí o comando de parada dos motores
    # nunca chega ao simulador.
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
            # Dá tempo de o planner publicar o último cmd_vel de parada.
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
