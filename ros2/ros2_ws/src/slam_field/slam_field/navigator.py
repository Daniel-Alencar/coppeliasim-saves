"""Campo potencial sobre o mapa do SLAM Toolbox, com parada de emergência pelo laser.

O robô é atraído pela banana alvo e repelido pelas bordas dos obstáculos
inflados do /map, na vizinhança dele. A resultante dá o rumo; o laser, a cada
scan, pode vetar o avanço.

    F = F_atrativa(banana alvo) + Σ_setores F_repulsiva(borda inflada mais próxima)

    /map      (OccupancyGrid) --->  |           |  ---> /cmd_vel           Twist
    /scan     (LaserScan)     --->  |           |  ---> /inflated_map      OccupancyGrid
    /bananas  (PoseArray)     --->  | navigator |  ---> /target, /force    (rviz)
    TF map -> base_link       --->  |           |  ---> /collected_banana  PointStamped
    /ground_truth (opcional)  --->  |           |  ---> /done              Bool

Unknown (-1)
------------
Por padrão a célula desconhecida conta como livre para o campo potencial: no
começo quase todo o mapa é desconhecido, e tratá-lo como obstáculo prenderia o
robô onde está. A segurança diante do que o mapa ainda não mostra fica com a
parada de emergência, que só olha o scan atual. unknown_is_obstacle:=true
inverte essa escolha (útil com o mapa já completo).
"""

import math

from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, Twist, Vector3Stamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32
import tf2_ros

from slam_field.grid import InflatedMap, repulsive_force


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(q):
    """Yaw de um quaternion qualquer (só a rotação em torno de z importa)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SlamFieldNavigator(Node):

    def __init__(self):
        super().__init__('navigator')

        # --- Referenciais ----------------------------------------------------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('control_rate', 10.0)        # [Hz]

        # --- Mapa e inflação -------------------------------------------------
        self.declare_parameter('robot_radius', 0.17)        # medido na cena: 0,167 m
        self.declare_parameter('safety_margin', 0.08)       # [m]
        self.declare_parameter('occupied_threshold', 65)    # >= isto é obstáculo
        self.declare_parameter('unknown_is_obstacle', False)

        # --- Campo potencial -------------------------------------------------
        self.declare_parameter('k_att', 1.0)
        # Atração de módulo constante até 0,3 m do alvo. Decaindo desde 1 m
        # (como no exercício anterior), ela perdia para a repulsão e o robô
        # parava a meio metro de bananas perto de parede.
        self.declare_parameter('att_range', 0.3)            # atração satura acima [m]
        self.declare_parameter('k_rep', 0.01)
        # Vizinhança: só bordas infladas a menos disto repelem. É a folga a
        # partir da borda inflada, que já desconta raio e margem.
        self.declare_parameter('influence', 0.4)            # [m]
        self.declare_parameter('sectors', 12)               # um ponto por setor
        self.declare_parameter('d_min', 0.03)               # [m] na conta da repulsão
        self.declare_parameter('escape_gain', 2.0)          # dentro da região inflada
        self.declare_parameter('f_max', 3.0)

        # --- Controle --------------------------------------------------------
        self.declare_parameter('v_max', 0.15)               # devagar, como pede o teste
        self.declare_parameter('w_max', 1.2)
        self.declare_parameter('k_heading', 2.5)
        self.declare_parameter('turn_in_place', math.radians(70.0))
        self.declare_parameter('collect_radius', 0.12)      # [m]
        # Banana dentro da região inflada (encostada em parede ou móvel): o
        # centro do robô não pode chegar nela. Vale chegar o mais perto que a
        # margem permite: o campo segura o centro a ~0,4 m da parede.
        self.declare_parameter('collect_radius_blocked', 0.45)

        # --- Parada de emergência --------------------------------------------
        # Corredor à frente com a largura do robô mais estop_lateral_margin de
        # cada lado: ponto do scan dentro dele a menos de estop_distance (do
        # centro) para o avanço; a menos de slow_distance, freia.
        self.declare_parameter('estop_distance', 0.30)      # raio 0,17 + 13 cm
        self.declare_parameter('slow_distance', 0.55)
        self.declare_parameter('estop_lateral_margin', 0.06)
        # Qualquer ponto da metade da frente a menos disto também para (pega
        # obstáculo de lado, fora do corredor, quando o robô avança em curva).
        self.declare_parameter('estop_radius', 0.24)
        self.declare_parameter('estop_release', 0.10)       # histerese [m]
        self.declare_parameter('scan_timeout', 0.6)         # scan velho = parar [s]

        # --- Empates e alvos impossíveis -------------------------------------
        self.declare_parameter('stall_time', 3.0)
        self.declare_parameter('stall_progress', 0.05)
        # O contorno dura até o robô ficar mais perto do alvo do que estava no
        # empate (como um algoritmo bug), com este limite de tempo.
        self.declare_parameter('swirl_max_time', 60.0)
        self.declare_parameter('swirl_gain', 1.2)
        self.declare_parameter('max_swirls_per_target', 4)
        # Contornar uma parede de 4 m a 0,15 m/s leva ~2 min.
        self.declare_parameter('target_timeout', 180.0)
        self.declare_parameter('skip_time', 45.0)
        self.declare_parameter('max_attempts', 3)           # depois disso: inalcançável
        self.declare_parameter('blocked_penalty', 2.0)      # [m] por 100% do trecho bloqueado

        # --- Seleção de bananas, para os testes ------------------------------
        # "3" = só a banana 3; "3,7,12" = essas; vazio = todas. Os índices são
        # os da tabela que o nó imprime ao receber mapa e bananas.
        self.declare_parameter('banana_ids', '')
        self.declare_parameter('error_log_period', 5.0)     # [s] erro de pose x gabarito

        p = self.get_parameter
        self.map_frame = p('map_frame').value
        self.base_frame = p('base_frame').value

        # --- Estado ----------------------------------------------------------
        self.grid = None
        self.map_stamp = None
        self.scan = None
        self.scan_time = None
        self.bananas = []
        self.pending = []
        self.collected = []
        self.unreachable = []
        self.attempts = {}
        self.skipped = {}
        self.target = None
        self.finished = False
        self.table_printed = False
        self.truth = None
        self.last_error_log = 0.0
        self.swirl_sign = 1.0
        self.estop_active = False
        self.estop_turn = 1.0
        self.reset_progress()

        # --- ROS -------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # O SLAM Toolbox publica o /map com transient local e reliable.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, map_qos)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.create_subscription(PoseArray, 'bananas', self.bananas_callback, latched)
        self.create_subscription(PoseStamped, 'ground_truth', self.truth_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.inflated_pub = self.create_publisher(OccupancyGrid, 'inflated_map', latched)
        self.target_pub = self.create_publisher(PointStamped, 'target', 10)
        self.force_pub = self.create_publisher(Vector3Stamped, 'force', 10)
        self.collected_pub = self.create_publisher(PointStamped, 'collected_banana', 10)
        self.count_pub = self.create_publisher(Int32, 'collected', latched)
        self.done_pub = self.create_publisher(Bool, 'done', latched)

        self.create_timer(1.0 / p('control_rate').value, self.control_step)
        self.get_logger().info('navegador pronto: esperando /map, /scan, /bananas e TF')

    # =========================================================================
    #  Entradas
    # =========================================================================

    def map_callback(self, msg: OccupancyGrid):
        """Classifica e infla o mapa novo. Roda a cada atualização do SLAM."""
        p = self.get_parameter
        info = msg.info
        if info.width == 0 or info.height == 0:
            return
        if abs(yaw_of(info.origin.orientation)) > 1e-3:
            self.get_logger().warn('origem do /map girada: ignorando a rotação',
                                   throttle_duration_sec=30.0)
        radius = p('robot_radius').value + p('safety_margin').value
        self.grid = InflatedMap(
            msg.data, info.width, info.height, info.resolution,
            info.origin.position.x, info.origin.position.y, radius,
            occupied_threshold=p('occupied_threshold').value,
            unknown_is_obstacle=p('unknown_is_obstacle').value)
        self.map_stamp = msg.header.stamp
        g = self.grid
        self.get_logger().info(
            f'mapa {g.width}x{g.height} @ {g.resolution:.3f} m: '
            f'{int(g.occupied.sum())} ocupadas, {int(g.free.sum())} livres, '
            f'{int(g.unknown.sum())} desconhecidas; inflação {radius:.2f} m = '
            f'{g.inflation_cells} células', throttle_duration_sec=10.0)
        self.publish_inflated(msg)
        if self.bananas and not self.table_printed:
            self.print_table()

    def scan_callback(self, msg: LaserScan):
        self.scan = msg
        self.scan_time = self.now()

    def truth_callback(self, msg: PoseStamped):
        self.truth = msg

    def bananas_callback(self, msg: PoseArray):
        if self.bananas:
            return   # a lista é fixa nesta cena: só a primeira mensagem vale
        bananas = [(p.position.x, p.position.y) for p in msg.poses]
        self.bananas = bananas
        ids = self.get_parameter('banana_ids').value.strip()
        if ids:
            escolhidas = [int(i) for i in ids.replace(' ', '').split(',') if i]
            self.pending = [bananas[i] for i in escolhidas if 0 <= i < len(bananas)]
        else:
            self.pending = list(bananas)
        self.get_logger().info(
            f'{len(bananas)} bananas recebidas; {len(self.pending)} serão buscadas')
        if self.grid is not None:
            self.print_table()

    def print_table(self):
        """Lista as bananas com a folga até o obstáculo: ajuda a escolher os testes.

        'aberta' = longe de tudo; 'parede' = perto de um obstáculo mas fora da
        região inflada; 'inflada' = o centro do robô não chega nela.
        """
        self.table_printed = True
        linhas = ['banana   x       y     folga   situação']
        for i, (x, y) in enumerate(self.bananas):
            folga = self.grid.clearance(x, y)
            if self.grid.is_blocked(x, y):
                sit = 'inflada'
            elif folga < self.grid.inflation_radius + 0.3:
                sit = 'parede'
            else:
                sit = 'aberta' if self.grid.state_at(x, y) == 'free' else 'desconhecida'
            linhas.append(f'{i:4d}  {x:6.2f}  {y:6.2f}  {folga:5.2f}   {sit}')
        self.get_logger().info('bananas (use banana_ids:=N para testar uma):\n'
                               + '\n'.join(linhas))

    # =========================================================================
    #  Ciclo de controle
    # =========================================================================

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def robot_pose(self):
        """(x, y, yaw) do base_link no map, pela TF (map->odom do SLAM)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(), timeout=Duration(seconds=0.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = tf.transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def control_step(self):
        if self.finished:
            return
        now = self.now()
        pose = self.robot_pose()
        faltando = [n for n, ok in (('TF map->base_link', pose is not None),
                                    ('/map', self.grid is not None),
                                    ('/scan', self.scan is not None),
                                    ('/bananas', bool(self.bananas))) if not ok]
        if faltando:
            self.get_logger().info(f'esperando {", ".join(faltando)}',
                                   throttle_duration_sec=5.0)
            self.publish_cmd(0.0, 0.0)
            return
        if now - self.scan_time > self.get_parameter('scan_timeout').value:
            self.get_logger().warn('scan atrasado: parando', throttle_duration_sec=2.0)
            self.publish_cmd(0.0, 0.0)
            return

        x, y, theta = pose
        self.log_pose_error(x, y, theta, now)

        # --- Alvo ------------------------------------------------------------
        if self.target is not None and self.reached(x, y):
            self.collect(x, y)
        if self.target is None:
            self.target = self.choose_target(x, y, now)
            self.reset_progress()
            self.target_since = now
            if self.target is None:
                if self.pending:   # só sobraram bananas puladas: espera
                    self.publish_cmd(0.0, 0.0)
                    return
                self.finish()
                return
            self.get_logger().info(
                f'alvo: banana em ({self.target[0]:.2f}, {self.target[1]:.2f}), '
                f'{self.grid.state_at(*self.target)}')

        if self.swirls_on_target >= self.get_parameter('max_swirls_per_target').value:
            self.give_up(now, f'{self.swirls_on_target} empates')
            return
        if now - self.target_since > self.get_parameter('target_timeout').value:
            self.give_up(now, 'tempo esgotado')
            return
        self.publish_target()

        # --- Forças ----------------------------------------------------------
        p = self.get_parameter
        ax, ay, distance = self.attractive(x, y)
        rx, ry, inside, _ = repulsive_force(
            self.grid, x, y, p('influence').value, p('sectors').value,
            p('k_rep').value, p('d_min').value, p('escape_gain').value)
        if inside:
            # Dentro da região inflada só a fuga vale: a atração poderia
            # puxar o robô mais para dentro.
            ax = ay = 0.0
            self.get_logger().warn('centro do robô dentro da região inflada: saindo',
                                   throttle_duration_sec=3.0)
        sx, sy = self.swirl(now, distance, ax, ay, rx, ry)
        fx, fy = ax + rx + sx, ay + ry + sy
        magnitude = math.hypot(fx, fy)
        f_max = p('f_max').value
        if magnitude > f_max:
            fx, fy, magnitude = fx * f_max / magnitude, fy * f_max / magnitude, f_max
        self.publish_force(fx, fy)

        # --- Comando ---------------------------------------------------------
        w_max = p('w_max').value
        if magnitude < 1e-6:
            self.publish_cmd(0.0, 0.5 * w_max)
            return
        error = wrap(math.atan2(fy, fx) - theta)
        w = max(-w_max, min(w_max, p('k_heading').value * error))
        v = 0.0 if abs(error) > p('turn_in_place').value else p('v_max').value * math.cos(error)
        v = self.emergency_limit(v)
        if self.estop_active:
            # Parado pelo laser: gira para o lado que estava mais livre quando
            # parou, sempre o mesmo, até o corredor à frente liberar.
            w = 0.6 * w_max * self.estop_turn
        self.publish_cmd(v, w)

    # =========================================================================
    #  Forças e progresso
    # =========================================================================

    def attractive(self, x, y):
        k_att = self.get_parameter('k_att').value
        att_range = self.get_parameter('att_range').value
        dx, dy = self.target[0] - x, self.target[1] - y
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return 0.0, 0.0, d
        scale = k_att * min(1.0, d / att_range) / d
        return dx * scale, dy * scale, d

    def swirl(self, now, distance, ax, ay, rx, ry):
        """Tangente à repulsão para sair de um mínimo local.

        Empate = stall_time sem reduzir em stall_progress a menor distância já
        obtida até o alvo. Aí entra uma força perpendicular à repulsão (segue a
        borda do obstáculo), para o lado que gira a repulsão em direção ao
        alvo. Ela fica ligada até o robô ficar mais perto do alvo do que
        estava no empate — critério do algoritmo bug: contornar até sair da
        "sombra" do obstáculo —, ou até swirl_max_time.
        """
        p = self.get_parameter
        if self.swirling:
            if (distance < self.swirl_distance - p('stall_progress').value
                    or now - self.swirl_start > p('swirl_max_time').value):
                self.swirling = False
                self.best_distance = distance
                self.stall_since = None
        else:
            if distance < self.best_distance - p('stall_progress').value:
                self.best_distance = distance
                self.stall_since = None
            elif self.stall_since is None:
                self.stall_since = now
            if self.stall_since is not None and now - self.stall_since >= p('stall_time').value:
                bx, by = (rx, ry) if math.hypot(rx, ry) > 1e-6 else (ax, ay)
                cross = bx * ay - by * ax
                if abs(cross) > 1e-9:
                    self.swirl_sign = 1.0 if cross > 0.0 else -1.0
                self.swirling = True
                self.swirl_start = now
                self.swirl_distance = distance
                self.stall_since = None
                self.swirls_on_target += 1
                lado = 'esquerda' if self.swirl_sign > 0 else 'direita'
                self.get_logger().info(
                    f'campo empatado a {distance:.2f} m do alvo (empate '
                    f'{self.swirls_on_target}): contornando pela {lado}')
        if not self.swirling:
            return 0.0, 0.0
        bx, by = (rx, ry) if math.hypot(rx, ry) > 1e-6 else (ax, ay)
        base = math.hypot(bx, by)
        if base < 1e-6:
            return 0.0, 0.0
        gain = p('swirl_gain').value * max(math.hypot(ax, ay), 0.5) * self.swirl_sign / base
        return -by * gain, bx * gain

    def reset_progress(self):
        self.target_since = None
        self.swirls_on_target = 0
        self.best_distance = math.inf
        self.stall_since = None
        self.swirling = False
        self.swirl_start = 0.0
        self.swirl_distance = math.inf

    # =========================================================================
    #  Parada de emergência
    # =========================================================================

    def scan_points(self):
        """Pontos válidos do scan atual no referencial do robô (x à frente)."""
        scan = self.scan
        r = np.asarray(scan.ranges, dtype=float)
        ang = scan.angle_min + np.arange(r.size) * scan.angle_increment
        ok = np.isfinite(r) & (r >= scan.range_min)
        return r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])

    def emergency_limit(self, v):
        """Limita o avanço pelo scan atual, inclusive o que o mapa ainda não tem.

        Só o scan deste instante entra aqui: nada do mapa. O laser está no
        centro do robô (3 mm à frente), então os pontos já estão no base_link.
        """
        p = self.get_parameter
        px, py = self.scan_points()
        half_width = p('robot_radius').value + p('estop_lateral_margin').value
        corridor = (px > 0.0) & (np.abs(py) <= half_width)
        ahead = float(px[corridor].min()) if corridor.any() else math.inf
        front = px > -0.02
        close = float(np.hypot(px[front], py[front]).min()) if front.any() else math.inf
        stop, slow = p('estop_distance').value, p('slow_distance').value
        # Histerese: uma vez parado, só libera com estop_release a mais de
        # folga. Sem ela o robô oscilava na borda do limiar, sem sair do lugar.
        extra = p('estop_release').value if self.estop_active else 0.0
        if ahead <= stop + extra or close <= p('estop_radius').value + extra:
            if not self.estop_active:
                self.estop_turn = self.freer_side()
                self.get_logger().warn(
                    f'PARADA DE EMERGÊNCIA: obstáculo a {min(ahead, close):.2f} m '
                    'à frente do centro')
            self.estop_active = True
            return 0.0
        if self.estop_active:
            self.get_logger().info('caminho livre de novo')
        self.estop_active = False
        if ahead < slow:
            # Freia até 30%, sem zerar: zerado na borda do corredor, o robô
            # ficava parado diante de um obstáculo fora do mapa, sem conseguir
            # contorná-lo. Abaixo de estop_distance a parada é total.
            v *= 0.3 + 0.7 * (ahead - stop) / (slow - stop)
        return v

    def freer_side(self):
        """+1 se a esquerda do scan tem mais espaço que a direita, senão -1."""
        scan = self.scan
        r = np.asarray(scan.ranges, dtype=float)
        r = np.where(np.isfinite(r), r, scan.range_max)
        ang = scan.angle_min + np.arange(r.size) * scan.angle_increment
        left = r[(ang > 0.2) & (ang < 1.4)]
        right = r[(ang < -0.2) & (ang > -1.4)]
        if left.size == 0 or right.size == 0:
            return 1.0
        return 1.0 if left.mean() >= right.mean() else -1.0

    # =========================================================================
    #  Alvos
    # =========================================================================

    def choose_target(self, x, y, now):
        """Banana pendente de menor custo: distância + trecho bloqueado no caminho."""
        self.skipped = {b: t for b, t in self.skipped.items() if t > now}
        candidatas = [b for b in self.pending if b not in self.skipped]
        if not candidatas:
            return None
        penalty = self.get_parameter('blocked_penalty').value

        def custo(b):
            frac = self.grid.segment_blocked_fraction((x, y), b)
            return math.hypot(b[0] - x, b[1] - y) + penalty * frac
        return min(candidatas, key=custo)

    def reached(self, x, y):
        p = self.get_parameter
        raio = p('collect_radius').value
        if self.grid.is_blocked(*self.target):
            raio = p('collect_radius_blocked').value
        return math.hypot(self.target[0] - x, self.target[1] - y) <= raio

    def collect(self, x, y):
        b = self.target
        self.pending.remove(b)
        self.collected.append(b)
        self.count_pub.publish(Int32(data=len(self.collected)))
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.point.x, msg.point.y = b
        self.collected_pub.publish(msg)
        total = len(self.collected) + len(self.pending) + len(self.unreachable)
        self.get_logger().info(
            f'banana {len(self.collected)}/{total} alcançada em ({b[0]:.2f}, {b[1]:.2f})')
        self.target = None
        self.reset_progress()

    def give_up(self, now, motivo):
        b = self.target
        self.attempts[b] = self.attempts.get(b, 0) + 1
        if self.attempts[b] >= self.get_parameter('max_attempts').value:
            self.pending.remove(b)
            self.unreachable.append(b)
            self.get_logger().warn(
                f'banana em ({b[0]:.2f}, {b[1]:.2f}) inalcançável ({motivo}, '
                f'{self.attempts[b]} tentativas): descartada')
        else:
            self.skipped[b] = now + self.get_parameter('skip_time').value
            self.get_logger().info(
                f'desisti da banana em ({b[0]:.2f}, {b[1]:.2f}): {motivo}; '
                f'tentativa {self.attempts[b]}')
        self.target = None
        self.reset_progress()
        self.publish_cmd(0.0, 0.0)

    def finish(self):
        self.finished = True
        self.publish_cmd(0.0, 0.0)
        self.done_pub.publish(Bool(data=True))
        msg = f'fim: {len(self.collected)} bananas alcançadas'
        if self.unreachable:
            lista = ', '.join(f'({x:.2f}, {y:.2f})' for x, y in self.unreachable)
            msg += f'; {len(self.unreachable)} inalcançáveis: {lista}'
        self.get_logger().info(msg)

    # =========================================================================
    #  Saídas
    # =========================================================================

    def log_pose_error(self, x, y, theta, now):
        """Erro entre a pose do SLAM (TF) e a verdadeira, para comparar odometrias."""
        period = self.get_parameter('error_log_period').value
        if self.truth is None or period <= 0 or now - self.last_error_log < period:
            return
        self.last_error_log = now
        tp = self.truth.pose
        ex = math.hypot(tp.position.x - x, tp.position.y - y)
        eth = math.degrees(abs(wrap(yaw_of(tp.orientation) - theta)))
        self.get_logger().info(f'erro da pose no map: {ex * 100:.1f} cm, {eth:.1f}°')

    def publish_inflated(self, original: OccupancyGrid):
        """Mapa inflado para o rviz: 100 bloqueado, 0 livre, -1 desconhecido."""
        g = self.grid
        out = np.where(g.blocked, 100, np.where(g.unknown, -1, 0)).astype(np.int8)
        msg = OccupancyGrid()
        msg.header = original.header
        msg.info = original.info
        msg.data = out.ravel().tolist()
        self.inflated_pub.publish(msg)

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def publish_target(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.point.x, msg.point.y = self.target
        self.target_pub.publish(msg)

    def publish_force(self, fx, fy):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.vector.x, msg.vector.y = float(fx), float(fy)
        self.force_pub.publish(msg)

    def stop(self):
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = SlamFieldNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
