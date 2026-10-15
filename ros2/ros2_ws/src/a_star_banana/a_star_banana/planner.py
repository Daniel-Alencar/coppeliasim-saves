"""Planejamento global com A* e execução do caminho pelos waypoints.

Este nó não conhece o CoppeliaSim e não mexe nos motores: ele recebe as
posições pela ponte, planeja, e manda um waypoint de cada vez para o
controlador de posição do exercício anterior.

    poops        (PoseArray)   --->  |         |  --->  goal   (PoseStamped) -> position_control
    bananas      (PoseArray)   --->  | planner |  --->  path   (nav_msgs/Path)
    banana_paths (String)      --->  |         |  --->  grid   (OccupancyGrid)
    pose         (PoseStamped) --->  |         |  --->  done   (Bool) -> ponte

O ciclo é:

    1. monta o mapa de ocupação com os poops inflados (grid.py)
    2. escolhe a banana alvo e roda o A* (a_star.py)
    3. simplifica o caminho nos pontos onde ele muda de direção
    4. publica o waypoint atual em `goal`; quando o robô chega a menos de
       waypoint_tolerance dele, publica o próximo
    5. no último ponto (a banana), espera a cena recolhê-la

O alvo é escolhido pelo parâmetro `goal`:

    ''                      a banana de menor custo A* a partir do robô
    '/bananas/Banana[4]'    aquela banana, pelo caminho da API
    '1.5,-0.8'              um ponto qualquer (serve para testar um destino
                            inalcançável, como o experimento pede)
"""

import json
import math

from geometry_msgs.msg import Point, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64, Int32, String

from a_star_banana import a_star
from a_star_banana.grid import Grid, bounds_from_points


def yaw_quaternion(yaw):
    """Quaternion (z, w) de uma rotação só em torno do eixo vertical."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Planner(Node):
    """A* sobre o mapa de ocupação, executado waypoint a waypoint."""

    def __init__(self):
        super().__init__('planner')

        # --- Mapa de ocupação -------------------------------------------------
        self.declare_parameter('resolution', 0.2)     # lado da célula [m]
        # Raio do robô (0,16 m, medido na cena: 0,32 m de diâmetro) mais a
        # folga de segurança. O centro do robô é o que anda pelas células, e
        # com esta inflação o corpo dele nunca encosta num poop.
        self.declare_parameter('robot_radius', 0.16)  # [m]
        self.declare_parameter('safety_margin', 0.09)  # [m]
        self.declare_parameter('arena_margin', 0.4)   # folga da caixa do mapa [m]
        self.declare_parameter('diagonal', True)      # vizinhança de 8

        # --- Alvo e execução ---------------------------------------------------
        self.declare_parameter('goal', '')
        # Sem alvo pedido, o robô não para na primeira: recolhida uma banana,
        # ele replaneja para a próxima até não sobrar nenhuma alcançável. Com
        # `goal` preenchido isto não vale — aí o alvo é aquele, e só aquele,
        # que é o experimento que o enunciado pede.
        self.declare_parameter('collect_all', True)
        self.declare_parameter('waypoint_tolerance', 0.15)  # [m]
        self.declare_parameter('goal_tolerance', 0.06)      # último ponto [m]
        # Sem chegar ao waypoint neste tempo, replaneja da pose atual.
        self.declare_parameter('replan_timeout', 25.0)      # [s]
        self.declare_parameter('attempts_per_banana', 3)
        # Depois de parar em cima da banana, quanto esperar a cena recolhê-la.
        self.declare_parameter('collect_timeout', 6.0)      # [s]
        self.declare_parameter('show_map', True)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('stop_on_done', True)

        self.frame_id = self.get_parameter('frame_id').value

        # --- Estado --------------------------------------------------------
        self.poops = None         # [(x, y)]
        self.bananas = []         # [(x, y)]
        self.banana_paths = []    # caminhos da API, na mesma ordem
        self.robot = None         # (x, y, yaw)
        self.sim_clock = None     # relógio da cena, vindo da ponte
        self.scene_score = 0      # placar da cena: inclui as bananas que o
                                  # robô pegou de passagem, sem mirar nelas
        self.grid = None
        self.waypoints = []       # [(x, y)] do caminho atual
        self.index = 0            # waypoint perseguido agora
        self.target = None        # (x, y) da banana alvo
        self.target_path = None   # caminho da API dela
        self.since = None         # quando o waypoint atual foi publicado [s]
        self.arrived_at = None    # quando o robô chegou no último waypoint [s]
        self.attempts = {}        # caminho da API -> tentativas gastas
        self.skipped = []         # bananas que não deram certo
        self.collected = 0
        self.finished = False

        # --- Comunicação ROS ---------------------------------------------------
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseArray, 'poops', self.poops_callback, map_qos)
        self.create_subscription(PoseArray, 'bananas', self.bananas_callback, map_qos)
        self.create_subscription(String, 'banana_paths', self.paths_callback, map_qos)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.create_subscription(Float64, 'sim_time', self.clock_callback, 10)
        self.create_subscription(Int32, 'collected', self.score_callback, map_qos)

        self.goal_pub = self.create_publisher(PoseStamped, 'goal', 10)
        self.path_pub = self.create_publisher(Path, 'path', map_qos)
        self.grid_pub = self.create_publisher(OccupancyGrid, 'grid', map_qos)
        self.done_pub = self.create_publisher(Bool, 'done', 10)

        self.get_logger().info('planner pronto, esperando o mapa e a pose')

    # =========================================================================
    #  Entradas
    # =========================================================================

    def poops_callback(self, msg: PoseArray):
        poops = [(p.position.x, p.position.y) for p in msg.poses]
        if poops != self.poops:
            self.poops = poops
            self.grid = None      # o mapa precisa ser remontado
            self.get_logger().info(f'{len(poops)} poops recebidos')

    def bananas_callback(self, msg: PoseArray):
        self.bananas = [(p.position.x, p.position.y) for p in msg.poses]

    def paths_callback(self, msg: String):
        self.banana_paths = json.loads(msg.data)

    def score_callback(self, msg: Int32):
        self.scene_score = msg.data

    def clock_callback(self, msg: Float64):
        """Relógio da cena, publicado pela ponte."""
        self.sim_clock = msg.data

    def now(self):
        """O tempo que importa aqui é o da simulação, não o do computador.

        Numa máquina lenta a simulação roda a uma fração do tempo real (medido:
        0,17x desta cena sem GPU). Contando pelo relógio do computador, os
        prazos de replanejamento disparariam com o robô tendo andado poucos
        centímetros. Enquanto o `sim_time` não chega, vale o relógio mesmo.
        """
        if self.sim_clock is not None:
            return self.sim_clock
        return self.get_clock().now().nanoseconds / 1e9

    def pose_callback(self, msg: PoseStamped):
        """Dispara o ciclo: a pose é o relógio deste nó, como no controlador."""
        self.robot = (msg.pose.position.x, msg.pose.position.y,
                      2.0 * math.atan2(msg.pose.orientation.z,
                                       msg.pose.orientation.w))
        if self.finished or self.poops is None:
            return
        now = self.now()

        if not self.waypoints:
            self.plan(now)
            return
        self.follow(now)

    # =========================================================================
    #  Planejamento
    # =========================================================================

    def build_grid(self):
        """Monta a matriz de ocupação a partir dos poops, bananas e robô.

        A caixa do mapa sai das posições da cena mais uma folga — nada de
        limites fixos no código. Cada poop ocupa a sua célula, e a inflação
        (raio do robô + margem) bloqueia o que estiver a menos disso dela.
        """
        points = list(self.poops) + list(self.bananas) + [self.robot[:2]]
        bounds = bounds_from_points(
            points, self.get_parameter('arena_margin').value)
        inflation = (self.get_parameter('robot_radius').value
                     + self.get_parameter('safety_margin').value)
        grid = Grid(self.poops, bounds,
                    self.get_parameter('resolution').value, inflation)
        self.get_logger().info(
            f'mapa {grid.rows}x{grid.cols} células de '
            f'{grid.resolution:.2f} m, inflação {inflation:.2f} m '
            f'({int(math.ceil(inflation / grid.resolution))} células), '
            f'{len(grid.obstacle_cells)} poops no mapa')
        self.publish_grid(grid)
        return grid

    def plan(self, now):
        """Escolhe o alvo, roda o A* e começa a seguir o caminho."""
        if self.grid is None:
            self.grid = self.build_grid()

        start = self.grid.to_cell(self.robot[:2])
        if start is None:
            self.give_up('o robô está fora da área mapeada')
            return
        start_free = self.grid.nearest_free(start)
        if start_free is None:
            self.give_up('o robô está cercado de obstáculos inflados; '
                         'diminua safety_margin ou resolution')
            return
        if start_free != start:
            self.get_logger().warn(
                f'robô dentro da margem de inflação na célula {start}; '
                f'partindo da célula livre mais próxima {start_free}')

        target = self.choose_target(start_free)
        if target is None:
            return                     # choose_target já explicou e encerrou

        point, api_path = target
        goal_cell = self.grid.to_cell(point)
        if goal_cell is None:
            self.fail_target(api_path, 'o alvo está fora da área mapeada')
            return
        goal_free = self.grid.nearest_free(goal_cell)
        if goal_free is None:
            self.fail_target(api_path, 'o alvo está cercado de obstáculos inflados')
            return
        if goal_free != goal_cell:
            self.get_logger().warn(
                f'alvo dentro da margem de inflação na célula {goal_cell}; '
                f'mirando a célula livre mais próxima {goal_free}')

        cells, stats = a_star.search(
            self.grid, start_free, goal_free,
            self.get_parameter('diagonal').value)
        if cells is None:
            self.fail_target(
                api_path,
                f'sem caminho até {goal_free} ({stats["expanded"]} nós '
                'expandidos e nenhum chegou lá)')
            return

        waypoints = [self.grid.to_world(row, col) for row, col in a_star.simplify(cells)]
        # O último ponto é a banana de verdade, e não o centro da célula: é
        # sobre ela que o sensor do robô precisa passar.
        waypoints[-1] = point

        distance = stats['cost_cells'] * self.grid.resolution
        self.get_logger().info(
            f'caminho até {api_path or "o ponto pedido"}: {stats["cells"]} '
            f'células, {distance:.2f} m, {len(waypoints)} waypoints, '
            f'{stats["expanded"]} nós expandidos')
        if self.get_parameter('show_map').value:
            self.get_logger().info(
                '\n' + self.grid.render(cells, start_free, goal_free))

        self.target, self.target_path = point, api_path
        self.waypoints = waypoints
        self.index = 0
        self.publish_path(waypoints)
        self.send_goal(now)

    def choose_target(self, start_cell):
        """O alvo pedido no parâmetro `goal`, ou a banana de menor custo A*.

        Com um ponto "x,y" o alvo é esse ponto, mesmo que não haja banana lá —
        é como se testa um destino inalcançável. Sem parâmetro, roda o A* para
        cada banana pendente e fica com a de menor custo real: "mais próxima"
        aqui é pelo caminho que desvia dos poops, não em linha reta.
        """
        wanted = self.get_parameter('goal').value.strip()

        if ',' in wanted:
            x, y = (float(v) for v in wanted.split(',', 1))
            return (x, y), None

        pending = [(point, path) for point, path in self.pending_bananas()
                   if path not in self.skipped]
        if not pending:
            self.finish('não há mais banana para buscar')
            return None

        if wanted:
            for point, path in pending:
                if path == wanted or path.endswith('/' + wanted):
                    return point, path
            self.give_up(
                f'banana {wanted!r} não está na cena (ou já foi recolhida). '
                f'Opções: {", ".join(path for _, path in pending)}')
            return None

        diagonal = self.get_parameter('diagonal').value
        best, best_cost = None, math.inf
        for point, path in pending:
            cell = self.grid.to_cell(point)
            if cell is None:
                continue
            cell = self.grid.nearest_free(cell)
            if cell is None:
                continue
            _, stats = a_star.search(self.grid, start_cell, cell, diagonal)
            if stats['cost_cells'] is not None and stats['cost_cells'] < best_cost:
                best, best_cost = (point, path), stats['cost_cells']
        if best is None:
            self.give_up('nenhuma banana é alcançável a partir daqui')
            return None
        return best

    def pending_bananas(self):
        """Bananas ainda no chão, com o caminho da API quando ele é conhecido."""
        pairs = []
        for index, point in enumerate(self.bananas):
            path = self.banana_paths[index] if index < len(self.banana_paths) else ''
            pairs.append((point, path))
        return pairs

    # =========================================================================
    #  Execução do caminho
    # =========================================================================

    def follow(self, now):
        """Avança no caminho quando o robô chega ao waypoint atual."""
        last = self.index == len(self.waypoints) - 1
        tolerance = (self.get_parameter('goal_tolerance').value if last
                     else self.get_parameter('waypoint_tolerance').value)
        waypoint = self.waypoints[self.index]
        distance = math.hypot(waypoint[0] - self.robot[0],
                              waypoint[1] - self.robot[1])

        if last:
            if self.target_path is None:
                # Alvo é um ponto qualquer, não uma banana: chegar basta.
                if distance <= tolerance:
                    self.finish(f'ponto ({waypoint[0]:+.2f}, {waypoint[1]:+.2f}) '
                                'alcançado')
                elif now - self.since > self.get_parameter('replan_timeout').value:
                    self.retry('não cheguei ao ponto a tempo')
                return
            if self.collected_target():
                self.collected += 1
                self.get_logger().info(
                    f'banana recolhida pela cena ({self.collected} no total)')
                self.next_target()
                return
            # Chegou em cima dela e a cena não recolheu: espera um pouco e,
            # se não vier, tenta de novo de outro ângulo (replanejamento).
            # A espera conta a partir da CHEGADA, não da partida: a ponte relê
            # as bananas em rodízio, então a notícia de que a cena recolheu
            # esta leva alguns segundos para chegar.
            if distance <= tolerance:
                if self.arrived_at is None:
                    self.arrived_at = now
                    self.get_logger().info(
                        'em cima da banana, esperando a cena recolhê-la')
            else:
                self.arrived_at = None
            waiting = (self.arrived_at is not None
                       and now - self.arrived_at
                       > self.get_parameter('collect_timeout').value)
            if waiting:
                self.retry('cheguei na banana e a cena não a recolheu')
            elif now - self.since > self.get_parameter('replan_timeout').value:
                self.retry('não cheguei na banana a tempo')
            return

        if distance <= tolerance:
            self.index += 1
            self.send_goal(now)
            return
        if now - self.since > self.get_parameter('replan_timeout').value:
            self.retry(f'waypoint {self.index} não alcançado a tempo')

    def send_goal(self, now):
        """Publica o waypoint atual para o controlador de posição.

        O rumo pedido é a direção do trecho que começa nesse ponto (no último,
        a do trecho que chega nele): assim a lei de controle já alinha o robô
        com a curva seguinte, em vez de parar e girar a cada ponto.
        """
        x, y = self.waypoints[self.index]
        if self.index + 1 < len(self.waypoints):
            nxt = self.waypoints[self.index + 1]
            yaw = math.atan2(nxt[1] - y, nxt[0] - x)
        elif self.index > 0:
            previous = self.waypoints[self.index - 1]
            yaw = math.atan2(y - previous[1], x - previous[0])
        else:
            yaw = self.robot[2]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = yaw_quaternion(yaw)
        self.goal_pub.publish(msg)
        self.since = now
        self.arrived_at = None
        self.get_logger().info(
            f'waypoint {self.index + 1}/{len(self.waypoints)}: '
            f'({x:+.2f}, {y:+.2f})')

    def collected_target(self):
        """A banana alvo sumiu da lista da cena, ou seja, foi recolhida."""
        if self.target_path is None:
            return False      # alvo é um ponto qualquer: não há o que recolher
        return self.target_path not in self.banana_paths

    def retry(self, reason):
        """Replaneja o alvo atual; depois de tentar demais, pula para o próximo."""
        path = self.target_path or 'ponto pedido'
        self.attempts[path] = self.attempts.get(path, 0) + 1
        if self.attempts[path] >= self.get_parameter('attempts_per_banana').value:
            self.fail_target(self.target_path,
                             f'{reason} ({self.attempts[path]} tentativas)')
            return
        self.get_logger().warn(f'{reason}: replanejando de onde estou')
        self.waypoints = []
        self.grid = None      # as bananas e o robô mudaram de lugar

    def fail_target(self, api_path, reason):
        """Desiste deste alvo: encerra, ou passa ao próximo se collect_all."""
        name = api_path or 'o ponto pedido'
        self.get_logger().error(f'{name} não dá: {reason}')
        if self.get_parameter('show_map').value and self.grid is not None:
            start = self.grid.to_cell(self.robot[:2])
            goal = self.grid.to_cell(self.target) if self.target else None
            self.get_logger().info('\n' + self.grid.render(None, start, goal))
        if api_path:
            self.skipped.append(api_path)
        self.next_target(failed=True)

    def next_target(self, failed=False):
        """Fecha o alvo atual e procura o próximo, se for para coletar todas."""
        self.waypoints = []
        self.target = self.target_path = None
        self.grid = None
        # Alvo pedido na mão (uma banana ou um ponto): a tarefa era aquela.
        if self.get_parameter('goal').value.strip():
            self.finish('alvo encerrado' if failed else 'alvo pedido alcançado')
            return
        if not self.get_parameter('collect_all').value:
            self.finish('alvo encerrado' if failed else 'banana alcançada')
            return
        pending = [p for _, p in self.pending_bananas() if p not in self.skipped]
        if not pending:
            self.finish('não sobrou banana alcançável')

    def finish(self, reason):
        """Para o robô, avisa a ponte e encerra o nó."""
        if self.finished:
            return
        self.finished = True
        self.get_logger().info(
            f'{reason}: {self.collected} banana(s) alcançada(s) como alvo, '
            f'{self.scene_score} no placar da cena (as outras foram pegas de '
            'passagem)'
            + (f'; {len(self.skipped)} puladas' if self.skipped else ''))
        self.done_pub.publish(Bool(data=self.get_parameter('stop_on_done').value))

    def give_up(self, reason):
        """Erro que impede qualquer planejamento: explica e encerra."""
        self.get_logger().error(reason)
        if self.get_parameter('show_map').value and self.grid is not None:
            self.get_logger().info('\n' + self.grid.render())
        self.finish('planejamento impossível')

    # =========================================================================
    #  Publicações
    # =========================================================================

    def publish_path(self, waypoints):
        """O caminho planejado, para o rviz e para conferência no echo."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def publish_grid(self, grid):
        """O mapa de ocupação como nav_msgs/OccupancyGrid, para ver no rviz."""
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


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Alguns ciclos a mais para o `done` chegar à ponte antes de sair.
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
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
