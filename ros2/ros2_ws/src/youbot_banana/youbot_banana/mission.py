"""A missão: planeja, vai, pega, carrega e deposita — uma banana por vez.

Este é o nó que amarra o trabalho. Ele não conhece o CoppeliaSim, não comanda
motor nem junta: decide o que fazer em seguida e pede aos dois controladores.

    bananas     (PoseArray)   --->  |         |  ---> goal        (PoseStamped) -> base_control
    banana_info (String)      --->  |         |  ---> arm_goal    (PoseStamped) -> arm_control
    poops       (PoseArray)   --->  |         |  ---> arm_posture (String)      -> arm_control
    deposit     (String)      --->  | mission |  ---> gripper     (Float64)     -> arm_control
    pose        (PoseStamped) --->  |         |  ---> path        (nav_msgs/Path)
    goal_reached, arm_result  --->  |         |  ---> grid        (OccupancyGrid)
    grasped, sim_time         --->  |         |  ---> done        (Bool)        -> ponte

O A* E O MAPA SÃO OS DO EXERCÍCIO ANTERIOR
------------------------------------------
`from a_star_banana import a_star`, `from a_star_banana.grid import Grid`: o
planejamento global não foi reescrito, é importado do pacote do exercício do A*.
Mudam só os números que descrevem ESTE robô — o youBot tem 0,42 x 0,57 m, logo
0,36 m de raio circunscrito, contra os 0,16 m do robô diferencial — e um
obstáculo novo, o próprio depósito.

O que é novo aqui
-----------------
1. O alvo da base não é a banana: é uma POSE DE PREENSÃO, a 0,40 m dela e
   virada para ela, de onde o braço alcança. Qual das direções de aproximação
   usar é uma escolha de planejamento, e o nó experimenta várias (ver
   `choose_approach`).
2. Há uma sequência de manipulação, que é uma máquina de estados: aproximar,
   descer, fechar, levantar, recolher, ir ao depósito, soltar. Cada passo tem
   como falhar, e cada falha tem um jeito de seguir adiante.
3. O sucesso é medido: a banana tem de ACABAR no depósito. Pegar e derrubar no
   caminho não conta, e o nó percebe a diferença.

A máquina de estados
--------------------
    ESPERA ──> PLANEJA ──> VAI_ATE ──> APROXIMA ──> DESCE ──> FECHA
                  ^                                             │
                  │                                             v
          PROXIMA <── SOLTA <── POSICIONA <── VAI_DEPOSITO <── LEVANTA
                  │
                  └──> FIM (sem banana alcançável)

Qualquer estado pode voltar a PLANEJA (replanejamento) ou pular a banana atual.
"""

import json
import math

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64, String

from a_star_banana import a_star
from a_star_banana.grid import OCCUPIED, Grid, bounds_from_points
from youbot_banana import kinematics
from youbot_banana.kinematics import ARM

#: Os estados da missão. São strings porque é o que aparece no log, e um log
#: legível vale mais que uma enumeração.
ESPERA = 'espera'
RECOLHE = 'recolhe'
PLANEJA = 'planeja'
VAI_ATE = 'vai_ate'
APROXIMA = 'aproxima'
DESCE = 'desce'
FECHA = 'fecha'
LEVANTA = 'levanta'
VAI_DEPOSITO = 'vai_deposito'
POSICIONA = 'posiciona'
SOLTA = 'solta'
PROXIMA = 'proxima'
FIM = 'fim'


def yaw_quaternion(yaw):
    """(z, w) de uma rotação só em torno do eixo vertical."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class Mission(Node):
    """A máquina de estados da coleta."""

    def __init__(self):
        super().__init__('mission')

        # --- Mapa de ocupação ---------------------------------------------------
        self.declare_parameter('resolution', 0.10)        # lado da célula [m]
        # Raio circunscrito do youBot, medido na cena: 0,42 x 0,57 m de
        # footprint. É o raio certo para uma base HOLONÔMICA, que pode se
        # deslocar em qualquer direção e girar no lugar — não há uma direção
        # "estreita" que se possa aproveitar como num robô diferencial.
        self.declare_parameter('robot_radius', 0.36)      # [m]
        self.declare_parameter('safety_margin', 0.05)     # [m]
        self.declare_parameter('arena_margin', 0.30)      # [m]
        self.declare_parameter('diagonal', True)

        # --- Preensão -----------------------------------------------------------
        # Raio em que o braço pega do chão, do eixo da junta 0. O envelope
        # medido vai de 0,15 a 0,315 m; o meio é o lugar mais tolerante a erro
        # da base.
        self.declare_parameter('grasp_reach', 0.233)      # [m]
        # Altura da PONTA da garra ao agarrar. Abaixo disto as pastilhas dos
        # dedos raspam o chão (elas têm 56 mm de altura, centradas na ponta).
        self.declare_parameter('grasp_height', 0.028)     # [m]
        self.declare_parameter('approach_height', 0.14)   # [m] acima da banana
        self.declare_parameter('lift_height', 0.20)       # [m]
        # Folga de cada lado ao descer em volta da banana.
        self.declare_parameter('grasp_clearance', 0.008)  # [m]
        # Quantas direções de aproximação testar em volta da banana.
        self.declare_parameter('approach_candidates', 16)

        # --- Depósito -----------------------------------------------------------
        self.declare_parameter('release_height', 0.10)    # [m]
        # Meio-comprimento do robô, medido na cena: a frente dele fica a 0,293 m
        # do centro. É esta medida, e não o raio circunscrito, que diz a que
        # distância do depósito a base pode parar — ela chega de frente.
        self.declare_parameter('robot_half_length', 0.293)   # [m]
        # Folga entre a frente do robô e a parede do depósito.
        self.declare_parameter('deposit_clearance', 0.05)    # [m]
        # Quanto para dentro da borda a banana é solta.
        self.declare_parameter('release_inset', 0.05)        # [m]
        self.declare_parameter('delivered_radius', 0.20)  # [m] do centro

        # --- Execução -----------------------------------------------------------
        self.declare_parameter('waypoint_tolerance', 0.12)   # [m]
        self.declare_parameter('drive_timeout', 60.0)        # [s] de simulação
        self.declare_parameter('grip_timeout', 8.0)          # [s]
        self.declare_parameter('attempts_per_banana', 2)
        self.declare_parameter('show_map', True)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('stop_when_done', True)

        self.frame_id = self.get_parameter('frame_id').value

        # --- Estado da cena -----------------------------------------------------
        self.bananas = []        # [(x, y, z)] na ordem publicada
        self.info = []           # [{path, yaw, length, width, height}]
        self.poops = None        # [(x, y)]
        self.deposit = None      # {x, y, width, rim, ...}
        self.robot = None        # (x, y, yaw)
        self.clock = None        # relógio da simulação

        # --- Estado da missão ---------------------------------------------------
        self.state = ESPERA
        self.target = None       # índice da banana sendo buscada
        self.attempts = {}       # índice -> tentativas gastas
        self.skipped = set()
        self.delivered = set()
        self.grid = None
        self.waypoints = []
        self.waypoint = 0
        self.since = None        # quando o estado atual começou [s de simulação]
        self.base_arrived = False
        self.arm_result = None   # {'ok': bool, 'reason': str} do último movimento
        self.holding = False

        # --- Comunicação ROS -----------------------------------------------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseArray, 'bananas', self.bananas_callback, latched)
        self.create_subscription(String, 'banana_info', self.info_callback, latched)
        self.create_subscription(PoseArray, 'poops', self.poops_callback, latched)
        self.create_subscription(String, 'deposit', self.deposit_callback, latched)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.create_subscription(Float64, 'sim_time', self.clock_callback, 10)
        self.create_subscription(Bool, 'goal_reached', self.reached_callback, 10)
        self.create_subscription(String, 'arm_result', self.arm_callback, 10)
        self.create_subscription(Bool, 'grasped', self.grasped_callback, 10)

        self.goal_pub = self.create_publisher(PoseStamped, 'goal', 10)
        self.arm_goal_pub = self.create_publisher(PoseStamped, 'arm_goal', 10)
        self.posture_pub = self.create_publisher(String, 'arm_posture', 10)
        self.gripper_pub = self.create_publisher(Float64, 'gripper', 10)
        self.path_pub = self.create_publisher(Path, 'path', latched)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'grid', latched)
        self.done_pub = self.create_publisher(Bool, 'done', 10)

        self.create_timer(0.1, self.tick)
        self.get_logger().info('missão pronta, esperando a cena')

    # =========================================================================
    #  Entradas
    # =========================================================================

    def bananas_callback(self, msg: PoseArray):
        self.bananas = [(pose.position.x, pose.position.y, pose.position.z)
                        for pose in msg.poses]

    def info_callback(self, msg: String):
        self.info = json.loads(msg.data)

    def poops_callback(self, msg: PoseArray):
        self.poops = [(pose.position.x, pose.position.y) for pose in msg.poses]

    def deposit_callback(self, msg: String):
        self.deposit = json.loads(msg.data)

    def pose_callback(self, msg: PoseStamped):
        self.robot = (msg.pose.position.x, msg.pose.position.y,
                      2.0 * math.atan2(msg.pose.orientation.z,
                                       msg.pose.orientation.w))

    def clock_callback(self, msg: Float64):
        self.clock = msg.data

    def reached_callback(self, msg: Bool):
        if msg.data:
            self.base_arrived = True

    def arm_callback(self, msg: String):
        self.arm_result = json.loads(msg.data)

    def grasped_callback(self, msg: Bool):
        self.holding = msg.data

    # =========================================================================
    #  O laço
    # =========================================================================

    def tick(self):
        """Um passo da máquina de estados."""
        if self.state == FIM:
            return
        if self.robot is None or self.poops is None or self.clock is None:
            return
        if self.state == ESPERA:
            if not self.bananas or not self.info or self.deposit is None:
                return
            # O braço começa na postura em que o modelo do youBot foi salvo:
            # dobrado para trás, com a ponta do lado oposto ao eixo da junta 0.
            # Antes de qualquer alvo cartesiano ele tem de ir para uma postura
            # conhecida, ou o caminho até o primeiro alvo passa por q0 = 180°,
            # fora do limite da junta.
            self.enter(RECOLHE)
            self.posture_pub.publish(String(data='home'))
            self.gripper_pub.publish(Float64(data=1.0))
            return

        handler = {
            RECOLHE: self.do_recolhe,
            PLANEJA: self.do_planeja,
            VAI_ATE: self.do_vai_ate,
            APROXIMA: self.do_aproxima,
            DESCE: self.do_desce,
            FECHA: self.do_fecha,
            LEVANTA: self.do_levanta,
            VAI_DEPOSITO: self.do_vai_deposito,
            POSICIONA: self.do_posiciona,
            SOLTA: self.do_solta,
            PROXIMA: self.do_proxima,
        }.get(self.state)
        if handler:
            handler()

    def enter(self, state):
        """Troca de estado, zerando o que é por-estado."""
        self.state = state
        self.since = self.clock
        self.base_arrived = False
        self.arm_result = None
        self.get_logger().info(f'-> {state}')

    def elapsed(self):
        """Tempo no estado atual, em segundos de SIMULAÇÃO.

        O relógio da cena, e não o do computador: com a simulação a 0,3x do
        tempo real os prazos medidos no relógio de parede expirariam no meio de
        movimentos que estavam indo bem.
        """
        if self.since is None or self.clock is None:
            return 0.0
        return self.clock - self.since

    # =========================================================================
    #  Mapa e planejamento
    # =========================================================================

    def build_grid(self, ignore=None, deposit_radius=None):
        """Mapa de ocupação da arena, com tudo que a base tem de evitar.

        Obstáculos: os poops, o depósito e as OUTRAS bananas. As outras bananas
        entram porque passar por cima delas as arrasta pela arena, e uma banana
        arrastada para debaixo de um poop deixa de ser alcançável. A banana
        alvo, claro, fica de fora — é para cima dela que o robô vai.
        """
        obstacles = list(self.poops)
        radius = (self.get_parameter('robot_radius').value
                  + self.get_parameter('safety_margin').value)

        deposit_points = []
        if self.deposit:
            # O depósito é um quadrado, e o mapa trabalha com pontos: entra
            # como uma grade de pontos cobrindo a área dele, para a inflação
            # fechar o bloco inteiro em vez de só o centro.
            half_x = self.deposit['width'] / 2.0
            half_y = self.deposit['depth'] / 2.0
            step = self.get_parameter('resolution').value
            x = -half_x
            while x <= half_x:
                y = -half_y
                while y <= half_y:
                    deposit_points.append(
                        (self.deposit['x'] + x, self.deposit['y'] + y))
                    y += step
                x += step
            if deposit_radius is None:
                obstacles.extend(deposit_points)

        for index, banana in enumerate(self.bananas):
            if index == ignore or index in self.delivered:
                continue
            obstacles.append(banana[:2])

        points = list(obstacles) + list(deposit_points) + [self.robot[:2]]
        points += [banana[:2] for banana in self.bananas]
        bounds = bounds_from_points(
            points, margin=self.get_parameter('arena_margin').value)
        grid = Grid(obstacles, bounds,
                    self.get_parameter('resolution').value,
                    inflation_radius=radius, border=True)
        if deposit_radius is not None and deposit_points:
            # O depósito entra DEPOIS, com inflação própria (ver
            # plan_to_deposit para o porquê): a classe Grid infla tudo com o
            # mesmo raio, então a segunda inflação é feita aqui, à mão, sobre o
            # mapa já montado.
            for point in deposit_points:
                cell = grid.to_cell(point)
                if cell is not None:
                    grid.cells[cell[0]][cell[1]] = OCCUPIED
                    grid.obstacle_cells.append(cell)
            grid.inflate(deposit_radius)
        return grid

    def choose_approach(self, index):
        """De que lado chegar à banana, e por que caminho.

        O alvo da base não é a banana: é uma pose a `grasp_reach` + 0,166 m
        dela, virada para ela. Há infinitas dessas poses — uma por direção de
        aproximação — e elas não são equivalentes: uma pode cair dentro da
        margem de um poop, outra do lado oposto da arena.

        Então o nó testa `approach_candidates` direções em volta da banana, joga
        fora as que caem em célula bloqueada, roda o A* para as que sobram e
        escolhe a de MENOR CAMINHO. É o planejamento decidindo a pose de
        preensão, e não o contrário.

        Devolve (pose_da_base, caminho_em_células, grid) ou None.
        """
        banana = self.bananas[index][:2]
        grid = self.build_grid(ignore=index)
        start = grid.to_cell(self.robot[:2])
        if start is None:
            self.get_logger().warn('o robô está fora da área mapeada')
            return None
        start = grid.nearest_free(start)
        if start is None:
            self.get_logger().warn(
                'o robô está dentro da margem de um obstáculo e sem célula '
                'livre por perto')
            return None

        reach = self.get_parameter('grasp_reach').value
        count = self.get_parameter('approach_candidates').value
        best = None
        blocked = 0
        for step in range(count):
            yaw = -math.pi + 2.0 * math.pi * step / count
            pose = kinematics.base_pose_for_grasp(banana, yaw, reach)
            cell = grid.to_cell(pose[:2])
            if cell is None or grid.blocked(*cell):
                blocked += 1
                continue
            path, stats = a_star.search(
                grid, start, cell, self.get_parameter('diagonal').value)
            if path is None:
                continue
            if best is None or stats['cost_cells'] < best[2]:
                best = (pose, path, stats['cost_cells'])

        if best is None:
            self.get_logger().warn(
                f'banana {index}: nenhuma das {count} direções de aproximação '
                f'serve ({blocked} caem em célula bloqueada, o resto sem '
                'caminho)')
            return None
        return best[0], best[1], grid

    def do_recolhe(self):
        """Espera o braço chegar à postura de repouso antes de começar."""
        if self.arm_result is not None and self.arm_result.get('ok'):
            self.enter(PLANEJA)
            return
        if self.arm_result is not None:
            self.get_logger().error(
                f'o braço não foi para a postura de repouso: '
                f'{self.arm_result.get("reason", "?")}')
            self.finish()
            return
        if self.elapsed() > 15.0:
            self.get_logger().error(
                'o braço não respondeu ao pedido de postura em 15 s de '
                'simulação. O arm_control está rodando?')
            self.finish()

    def do_planeja(self):
        """Escolhe a próxima banana e o caminho até a pose de preensão dela."""
        candidates = [index for index in range(len(self.bananas))
                      if index not in self.delivered and index not in self.skipped]
        if not candidates:
            self.finish()
            return

        # A mais perto primeiro, em linha reta: é só a ORDEM de tentativa, e o
        # custo de verdade sai do A* dentro de choose_approach.
        candidates.sort(key=lambda i: math.dist(self.robot[:2], self.bananas[i][:2]))

        for index in candidates:
            chosen = self.choose_approach(index)
            if chosen is None:
                self.skip(index, 'sem pose de preensão alcançável')
                continue
            pose, path, grid = chosen
            self.target = index
            self.grid = grid
            self.waypoints = self.to_waypoints(path, pose)
            self.waypoint = 0
            self.publish_grid(grid, path)
            self.publish_path(self.waypoints)
            banana = self.bananas[index]
            self.get_logger().info(
                f'banana {index} em ({banana[0]:+.2f}, {banana[1]:+.2f}): '
                f'{len(self.waypoints)} waypoints até '
                f'({pose[0]:+.2f}, {pose[1]:+.2f}) rumo '
                f'{math.degrees(pose[2]):+.0f}°')
            if self.get_parameter('show_map').value:
                self.get_logger().info(
                    '\n' + grid.render(path=path,
                                       robot=grid.to_cell(self.robot[:2]),
                                       goal=grid.to_cell(banana[:2])))
            self.enter(VAI_ATE)
            self.send_waypoint()
            return

        self.finish()

    def to_waypoints(self, path, final_pose):
        """Caminho em células -> waypoints em metros, com a pose final no fim.

        Os pontos do meio vão com rumo LIVRE: a base é holonômica e não precisa
        se virar para andar, então exigir rumo em cada curva só a faria parar
        para girar sem motivo. O último waypoint é a pose de preensão, e aí o
        rumo importa — é ele que põe a banana à frente do braço.
        """
        simplified = a_star.simplify(path)
        waypoints = [(self.grid_point(cell), None) for cell in simplified[1:-1]]
        waypoints.append(((final_pose[0], final_pose[1]), final_pose[2]))
        return waypoints

    def grid_point(self, cell):
        return self.grid.to_world(*cell)

    # =========================================================================
    #  Os estados
    # =========================================================================

    def do_vai_ate(self):
        """Segue os waypoints até a pose de preensão."""
        if self.base_arrived:
            self.base_arrived = False
            self.waypoint += 1
            if self.waypoint >= len(self.waypoints):
                self.enter(APROXIMA)
                self.send_arm_to_banana(
                    self.get_parameter('grasp_height').value
                    + self.get_parameter('approach_height').value)
                self.gripper_pub.publish(Float64(data=self.open_for_target()))
                return
            self.send_waypoint()
            return
        if self.elapsed() > self.get_parameter('drive_timeout').value:
            self.retry('a base não chegou no prazo')

    def do_aproxima(self):
        """Braço parado acima da banana, garra aberta."""
        if self.arm_result is None:
            return
        if not self.arm_result.get('ok'):
            self.retry(f'braço não alcançou a aproximação: '
                       f'{self.arm_result.get("reason", "?")}')
            return
        self.enter(DESCE)
        self.send_arm_to_banana(self.get_parameter('grasp_height').value)

    def do_desce(self):
        """Garra em volta da banana, ainda aberta."""
        if self.arm_result is None:
            return
        if not self.arm_result.get('ok'):
            self.retry(f'braço não desceu: {self.arm_result.get("reason", "?")}')
            return
        self.enter(FECHA)
        self.gripper_pub.publish(Float64(data=0.0))

    def do_fecha(self):
        """Fecha a garra e espera travar em algo."""
        if self.holding:
            self.enter(LEVANTA)
            self.send_arm_to_banana(self.get_parameter('lift_height').value)
            return
        if self.elapsed() > self.get_parameter('grip_timeout').value:
            # Fechou no vazio: a banana não estava onde o nó pensava, ou
            # escorregou na descida.
            self.gripper_pub.publish(Float64(data=1.0))
            self.retry('a garra fechou sem pegar nada')

    def do_levanta(self):
        """Levanta a banana e recolhe o braço para andar."""
        if self.arm_result is None:
            return
        if not self.holding:
            self.retry('a banana escapou no levantamento')
            return
        if not self.arm_result.get('ok'):
            self.retry(f'braço não levantou: {self.arm_result.get("reason", "?")}')
            return
        self.posture_pub.publish(String(data='carry'))
        self.enter(VAI_DEPOSITO)
        self.plan_to_deposit()

    def do_vai_deposito(self):
        """Leva a banana até a pose de depósito."""
        if not self.holding:
            self.retry('a banana caiu no caminho do depósito')
            return
        if self.base_arrived:
            self.base_arrived = False
            self.waypoint += 1
            if self.waypoint >= len(self.waypoints):
                self.enter(POSICIONA)
                self.send_arm_to_deposit()
                return
            self.send_waypoint()
            return
        if self.elapsed() > self.get_parameter('drive_timeout').value:
            self.retry('a base não chegou ao depósito no prazo')

    def do_posiciona(self):
        """Braço estendido sobre o depósito."""
        if self.arm_result is None:
            return
        if not self.arm_result.get('ok'):
            self.retry(f'braço não alcançou o depósito: '
                       f'{self.arm_result.get("reason", "?")}')
            return
        self.enter(SOLTA)
        self.gripper_pub.publish(Float64(data=1.0))

    def do_solta(self):
        """Abre a garra e confere que a banana ficou no depósito."""
        if self.elapsed() < 1.5:
            return      # dá tempo de a garra abrir e a banana cair
        if self.target is None:
            self.enter(PROXIMA)
            return
        banana = self.bananas[self.target]
        distance = math.dist(banana[:2], (self.deposit['x'], self.deposit['y']))
        if distance <= self.get_parameter('delivered_radius').value:
            self.delivered.add(self.target)
            self.get_logger().info(
                f'banana {self.target} depositada '
                f'({len(self.delivered)}/{len(self.bananas)}) — a '
                f'{distance * 100:.0f} cm do centro do depósito')
            self.posture_pub.publish(String(data='home'))
            self.enter(PROXIMA)
            return
        if self.elapsed() > 5.0:
            self.retry(f'a banana ficou a {distance * 100:.0f} cm do depósito, '
                       'fora dele')

    def do_proxima(self):
        """Volta a planejar para a próxima banana."""
        self.target = None
        self.enter(PLANEJA)

    # =========================================================================
    #  Comandos
    # =========================================================================

    def send_waypoint(self):
        """Publica o waypoint atual para o controlador da base."""
        (x, y), yaw = self.waypoints[self.waypoint]
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        if yaw is None:
            # Quaternion nulo: o controlador da base lê isso como "rumo livre".
            # Nos pontos do meio do caminho o rumo não importa, e numa base
            # holonômica exigi-lo só custaria tempo.
            msg.pose.orientation.w = 0.0
        else:
            z, w = yaw_quaternion(yaw)
            msg.pose.orientation.z = z
            msg.pose.orientation.w = w
        self.goal_pub.publish(msg)
        self.base_arrived = False
        self.since = self.clock
        self.get_logger().info(
            f'waypoint {self.waypoint + 1}/{len(self.waypoints)}: '
            f'({x:+.2f}, {y:+.2f})'
            + ('' if yaw is None else f' rumo {math.degrees(yaw):+.0f}°'))

    def open_for_target(self):
        """Abertura da garra para a banana alvo, pelo tamanho MEDIDO dela.

        O tamanho vem da cena (a ponte publica em `banana_info`), e não do
        código: uma cena montada com outra escala de banana continua
        funcionando. Se a banana não couber na garra, o nó diz isso em vez de
        tentar e falhar sem explicação.
        """
        if self.target is None or self.target >= len(self.info):
            return 1.0
        width = self.info[self.target]['width']
        opening = kinematics.gripper_opening_for(
            width, clearance=self.get_parameter('grasp_clearance').value)
        if opening is None:
            self.get_logger().warn(
                f'banana {self.target} tem {width * 1000:.0f} mm de largura e a '
                f'garra abre {ARM.gripper_gap_open * 1000:.0f} mm: não cabe. '
                'Monte a cena com --banana-scale menor.')
            return 1.0
        return opening

    def send_arm_to_banana(self, height):
        """Manda a garra para cima da banana alvo, na altura pedida.

        A posição é recalculada com a pose ATUAL do robô, e não com a pose que
        o planejador pediu: a base para onde consegue, com alguns centímetros
        de erro, e o braço corrige a diferença. É o que torna a preensão
        tolerante ao escorregamento das rodas suecas.
        """
        banana = self.bananas[self.target]
        info = self.info[self.target] if self.target < len(self.info) else {'yaw': 0.0}
        local = self.to_base_frame(banana[:2])
        self.publish_arm_goal((local[0], local[1], height),
                              wrap(info['yaw'] - self.robot[2]))

    def send_arm_to_deposit(self):
        """Manda a garra para dentro do depósito, na borda mais próxima.

        O ponto é calculado a partir da pose ATUAL do robô (ver
        `deposit_geometry` para o porquê de não ser o centro): a `release`
        metros do eixo da junta 0, na direção em que o robô está olhando.
        """
        _, release, _ = self.deposit_geometry()
        center = self.to_base_frame((self.deposit['x'], self.deposit['y']))
        distance = math.hypot(center[0], center[1])
        if distance < 1e-6:
            self.retry('o robô parou em cima do depósito')
            return
        # Na direção do depósito, mas só até onde o braço alcança com folga.
        wanted = ARM.axis_y + release
        local = (center[0] * wanted / distance, center[1] * wanted / distance)
        height = max(self.get_parameter('release_height').value,
                     self.deposit['rim'] + 0.05)
        # A banana fica atravessada na garra, e solta-se na mesma orientação:
        # o yaw pedido é o da própria garra, que já está onde está.
        self.publish_arm_goal((local[0], local[1], height), 0.0)

    def publish_arm_goal(self, point, object_yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float(point[0])
        msg.pose.position.y = float(point[1])
        msg.pose.position.z = float(point[2])
        z, w = yaw_quaternion(object_yaw)
        msg.pose.orientation.z = z
        msg.pose.orientation.w = w
        self.arm_goal_pub.publish(msg)
        self.arm_result = None

    def to_base_frame(self, point):
        """Ponto do mundo -> frame da base, na convenção do ROS (+x à frente)."""
        dx = point[0] - self.robot[0]
        dy = point[1] - self.robot[1]
        cos_theta, sin_theta = math.cos(self.robot[2]), math.sin(self.robot[2])
        return (cos_theta * dx + sin_theta * dy,
                -sin_theta * dx + cos_theta * dy)

    def deposit_geometry(self):
        """A que distância parar do depósito, e onde soltar a banana.

        Esta é a conta que decide se a tarefa é possível, e ela quase não fecha.

            o braço alcança            0,15 a 0,32 m do eixo da junta 0
            o eixo da junta 0 fica     0,166 m à frente do centro da base
            a frente do robô fica      0,293 m à frente do centro da base

        Para não encostar no depósito, a base para a

            S = 0,293 + (meia-largura do depósito) + folga

        do centro dele. Com a bandeja de 0,28 m isso dá S = 0,47 m, e o eixo da
        junta 0 fica a 0,30 m do CENTRO do depósito — quase no limite dos
        0,32 m de alcance, e isso na altura de soltar, onde o envelope é ainda
        menor.

        A saída é não mirar no centro. A banana pode cair em QUALQUER ponto
        dentro da bandeja, e o ponto mais fácil é o da BORDA MAIS PRÓXIMA,
        alguns centímetros para dentro:

            raio pedido ao braço = S - (meia-largura) + recuo - 0,166

        que dá cerca de 0,19 m — bem no meio do envelope, com folga de sobra. É
        a mesma ideia de pegar a banana no meio do envelope em vez de na ponta
        dele: deixar margem para o erro da base.
        """
        half = max(self.deposit['width'], self.deposit['depth']) / 2.0
        stop = (self.get_parameter('robot_half_length').value + half
                + self.get_parameter('deposit_clearance').value)
        # Distância do EIXO DA JUNTA 0 ao ponto de soltar.
        release = (stop - half + self.get_parameter('release_inset').value
                   - ARM.axis_y)
        return stop, release, half

    def plan_to_deposit(self):
        """Caminho até a pose de onde o braço alcança dentro do depósito.

        O mapa usado aqui infla o DEPÓSITO menos que os outros obstáculos: pelo
        meio-comprimento do robô (0,293 m) em vez do raio circunscrito
        (0,36 m). A razão é geométrica, não uma licença: a pose final é, por
        construção, de FRENTE para o depósito, e a folga que importa nessa
        direção é a da frente do robô. Inflar pelo raio circunscrito aqui torna
        a tarefa impossível — o robô nunca poderia estender o braço sobre
        coisa alguma, já que ele alcança 0,32 m e o raio é 0,36 m.
        """
        _, release, _ = self.deposit_geometry()
        half_length = self.get_parameter('robot_half_length').value
        grid = self.build_grid(ignore=self.target, deposit_radius=half_length)
        start = grid.nearest_free(grid.to_cell(self.robot[:2]))
        center = (self.deposit['x'], self.deposit['y'])
        reach = release
        count = self.get_parameter('approach_candidates').value
        best = None
        for step in range(count):
            yaw = -math.pi + 2.0 * math.pi * step / count
            pose = kinematics.base_pose_for_grasp(center, yaw, reach)
            cell = grid.to_cell(pose[:2])
            if cell is None or grid.blocked(*cell):
                continue
            path, stats = a_star.search(grid, start, cell,
                                        self.get_parameter('diagonal').value)
            if path is None:
                continue
            if best is None or stats['cost_cells'] < best[2]:
                best = (pose, path, stats['cost_cells'])
        if best is None:
            self.get_logger().error(
                'não há caminho até o depósito. O robô está preso, ou o '
                'depósito está cercado.')
            self.finish()
            return
        self.grid = grid
        self.waypoints = self.to_waypoints(best[1], best[0])
        self.waypoint = 0
        self.publish_grid(grid, best[1])
        self.publish_path(self.waypoints)
        self.send_waypoint()

    # =========================================================================
    #  Falhas e fim
    # =========================================================================

    def retry(self, reason):
        """Conta a tentativa, solta o que estiver preso e volta a planejar."""
        index = self.target
        self.get_logger().warn(f'{self.state}: {reason}')
        self.gripper_pub.publish(Float64(data=1.0))
        self.posture_pub.publish(String(data='home'))
        if index is None:
            self.enter(PLANEJA)
            return
        self.attempts[index] = self.attempts.get(index, 0) + 1
        if self.attempts[index] >= self.get_parameter('attempts_per_banana').value:
            self.skip(index, f'{self.attempts[index]} tentativas sem sucesso')
        self.target = None
        self.enter(PLANEJA)

    def skip(self, index, reason):
        """Desiste desta banana, registrando o motivo."""
        if index in self.skipped:
            return
        self.skipped.add(index)
        self.get_logger().warn(f'banana {index} descartada: {reason}')

    def finish(self):
        """Fim da missão, com o placar."""
        if self.state == FIM:
            return
        self.state = FIM
        total = len(self.bananas)
        self.posture_pub.publish(String(data='home'))
        self.get_logger().info(
            f'FIM: {len(self.delivered)}/{total} bananas depositadas'
            + (f', {len(self.skipped)} descartadas '
               f'({sorted(self.skipped)})' if self.skipped else ''))
        if self.get_parameter('stop_when_done').value:
            self.done_pub.publish(Bool(data=True))

    # =========================================================================
    #  Saídas para o rviz
    # =========================================================================

    def publish_grid(self, grid, path=None):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = grid.resolution
        msg.info.width = grid.cols
        msg.info.height = grid.rows
        msg.info.origin.position.x = grid.x_min
        msg.info.origin.position.y = grid.y_min
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.occupancy_data()
        self.grid_pub.publish(msg)

    def publish_path(self, waypoints):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for (x, y), yaw in waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            z, w = yaw_quaternion(yaw or 0.0)
            pose.pose.orientation.z = z
            pose.pose.orientation.w = w
            msg.poses.append(pose)
        self.path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Mission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
