"""Navegação por campos potenciais: pega bananas, desvia de poops.

O robô é atraído pela banana alvo e repelido por cada poop. A soma das duas
contribuições dá uma força resultante, e é a direção dela que comanda o robô a
cada ciclo. Não existe caminho nem lista de pontos: a trajetória aparece
sozinha do campo.

    F = F_atrativa(banana alvo) + Σ F_repulsiva(poop)

Este nó não conhece o CoppeliaSim: ele só assina e publica tópicos. Quem
converte isso em simulação é a coppelia_bridge.

    bananas  (PoseArray)   --->  |            |
    poops    (PoseArray)   --->  | navigator  |  --->  cmd_vel  (Twist)
    pose     (PoseStamped) --->  |            |  --->  target   (PointStamped)
                                                --->  force    (Vector3Stamped)
                                                --->  collected (Int32)

O ciclo de controle é disparado por cada mensagem de pose, ou seja, roda na
frequência em que a ponte publica a pose do robô (parâmetro rate dela).
"""

import math

from geometry_msgs.msg import PointStamped, PoseArray, PoseStamped, Twist, Vector3Stamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32


def wrap(angle):
    """Normaliza um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(pose):
    """Rumo (rad) de uma geometry_msgs/Pose girada só em torno do eixo vertical."""
    return 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)


class PotentialFieldNavigator(Node):
    """Campo potencial: atrai para a banana alvo, repele dos poops."""

    def __init__(self):
        super().__init__('navigator')

        # --- Campo potencial -------------------------------------------------
        self.declare_parameter('k_att', 1.0)         # ganho da força atrativa
        self.declare_parameter('att_range', 1.0)     # acima disso a atração satura [m]
        self.declare_parameter('k_rep', 0.08)        # ganho da força repulsiva
        self.declare_parameter('d0_poop', 0.55)      # raio de influência do poop [m]
        self.declare_parameter('d_min', 0.08)        # distância mínima na conta [m]
        self.declare_parameter('f_max', 3.0)         # saturação da resultante

        # --- Controle --------------------------------------------------------
        self.declare_parameter('v_max', 0.25)        # velocidade de avanço [m/s]
        self.declare_parameter('w_max', 1.8)         # velocidade de giro [rad/s]
        self.declare_parameter('k_heading', 3.0)     # rad/s por rad de erro de rumo
        # Com erro de rumo maior que isto o robô gira parado, em vez de andar torto.
        self.declare_parameter('turn_in_place', math.radians(75.0))
        self.declare_parameter('collect_radius', 0.10)   # conta como alcançada [m]
        # O ciclo de controle é disparado pela pose. Se ela parar de chegar (a
        # ponte caiu, a simulação parou), o último cmd_vel continuaria valendo
        # e o robô seguiria às cegas. Passado este tempo, manda parar.
        self.declare_parameter('pose_timeout', 1.0)      # [s]

        # --- Busca -----------------------------------------------------------
        # Com o mapa vindo da câmera, "nenhuma banana conhecida" não quer dizer
        # que acabou: quer dizer que ainda não vimos nenhuma. Então o robô gira
        # para varrer o ambiente e, se não achar nada, anda um pouco e gira de
        # novo. Só desiste depois de search_timeout sem encontrar banana.
        self.declare_parameter('search', True)
        # Varredura devagar: girando rápido, a posição estimada pela câmera
        # fica ruim e o mapa enche de cópias do mesmo objeto.
        self.declare_parameter('search_w', 0.4)          # giro da varredura [rad/s]
        self.declare_parameter('search_v', 0.18)         # avanço da varredura [m/s]
        # 16 s a 0,4 rad/s dá uma volta inteira: a câmera vê o entorno todo.
        self.declare_parameter('search_spin_time', 16.0)  # tempo girando [s]
        self.declare_parameter('search_move_time', 3.0)  # tempo andando [s]
        self.declare_parameter('search_timeout', 120.0)  # desiste depois disso [s]
        # Distância para considerar que duas bananas do mapa são a mesma.
        self.declare_parameter('map_tolerance', 0.20)    # [m]
        # Banana que some do mapa com o robô a menos disto foi recolhida por
        # ele (o perception_map a retira quando o robô passa por cima).
        self.declare_parameter('vanish_collect_radius', 0.35)  # [m]

        # --- Alvo inalcançável -----------------------------------------------
        # Um alvo pode ser impossível: um poop (ou uma cópia dele no mapa) em
        # cima da banana, ou um poop classificado como banana. O campo empata
        # para sempre nesse ponto. Medido: sem esta regra o robô passou os
        # últimos 90 s de uma corrida de 4 min contornando o mesmo alvo, com 7
        # bananas conhecidas esperando.
        self.declare_parameter('max_swirls_per_target', 4)  # empates até desistir
        self.declare_parameter('target_timeout', 40.0)      # tempo máximo num alvo [s]
        self.declare_parameter('skip_time', 60.0)           # quanto tempo fica pulado [s]
        # Escolha do alvo: além da distância, cada poop a menos de d0_poop do
        # segmento robô-banana soma esta penalidade. Prefere bananas com o
        # caminho livre; a trajetória continua saindo das forças.
        self.declare_parameter('blocked_penalty', 1.0)      # [m por poop]

        # --- Mínimos locais --------------------------------------------------
        # O campo pode empatar: a repulsão dos poops cancela a atração e o robô
        # fica parado ou oscilando sem chegar na banana. Quando o progresso em
        # direção ao alvo para, entra uma componente tangencial (perpendicular à
        # atração) que contorna o empate. Continua sendo força, não caminho.
        self.declare_parameter('stall_time', 2.0)        # s sem progresso = empate
        self.declare_parameter('stall_progress', 0.05)   # aproximação mínima [m]
        self.declare_parameter('swirl_time', 2.5)        # duração da tangencial [s]
        self.declare_parameter('swirl_gain', 1.2)        # peso dela ante a atração

        # --- Mapa e estado ---------------------------------------------------
        self.bananas = []       # todas as bananas do mapa
        self.poops = []         # todos os poops do mapa
        self.pending = []       # bananas que ainda faltam
        self.target = None      # banana perseguida agora
        self.collected = 0
        self.finished = False
        self.reset_progress()
        self.swirl_sign = 1.0   # para que lado a tangencial desvia
        self.gone = []          # bananas que este nó já deu por coletadas
        self.robot_xy = None    # última posição do robô, para o mapa
        self.skipped = {}       # alvo -> até quando fica pulado
        self.target_since = None   # quando o alvo atual foi escolhido
        self.swirls_on_target = 0  # empates no alvo atual
        self.search_since = None   # quando a busca começou
        self.search_phase = None   # ('spin' | 'move', instante em que começou)

        # --- Comunicação ROS -------------------------------------------------
        # O mapa é publicado com transient local: mesmo subindo depois da ponte,
        # este nó recebe a última mensagem de bananas e poops.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseArray, 'bananas', self.bananas_callback, map_qos)
        self.create_subscription(PoseArray, 'poops', self.poops_callback, map_qos)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.target_pub = self.create_publisher(PointStamped, 'target', 10)
        self.force_pub = self.create_publisher(Vector3Stamped, 'force', 10)
        self.collected_pub = self.create_publisher(Int32, 'collected', 10)

        # Vigia da pose: roda em paralelo ao ciclo de controle, que só acontece
        # quando chega pose. Sem esse timer, nada notaria a ausência dela.
        self.last_pose_time = None
        self.pose_lost = False
        self.create_timer(0.2, self.check_pose_timeout)

        self.get_logger().info('navegador pronto, esperando o mapa e a pose')

    # =========================================================================
    #  Mapa
    # =========================================================================

    def bananas_callback(self, msg: PoseArray):
        """Atualiza o mapa de bananas sem perder o que já foi feito.

        O mapa pode chegar pronto (ponte lendo os sinais da cena) ou ir
        crescendo quando a câmera descobre bananas novas. Nos dois casos, as
        que aparecem são somadas às pendentes, e as que somem do mapa saem das
        pendentes: no caminho da percepção, sumir quer dizer que foi recolhida.
        """
        bananas = [(p.position.x, p.position.y) for p in msg.poses]
        if bananas == self.bananas:
            return
        tol = self.get_parameter('map_tolerance').value

        novas = [b for b in bananas
                 if not self.near_any(b, self.pending, tol)
                 and not self.near_any(b, self.gone, tol)]
        sumiram = [b for b in self.pending if not self.near_any(b, bananas, tol)]

        self.bananas = bananas
        self.pending = [b for b in self.pending if b not in sumiram] + novas
        # Sumiu com o robô do lado: foi ele que passou por cima e a recolheu.
        raio = self.get_parameter('vanish_collect_radius').value
        for b in sumiram:
            if self.robot_xy and math.hypot(b[0] - self.robot_xy[0],
                                            b[1] - self.robot_xy[1]) <= raio:
                self.gone.append(b)
                self.collected += 1
                self.collected_pub.publish(Int32(data=self.collected))
                self.get_logger().info(f'banana {self.collected} recolhida')
        if self.target is not None and self.target in sumiram:
            self.target = None
            self.reset_progress()
        if novas:
            self.finished = False
            self.search_phase = None
            self.search_since = None
        if novas or sumiram:
            self.get_logger().info(
                f'mapa: {len(bananas)} bananas conhecidas, {len(self.pending)} '
                f'pendentes (+{len(novas)} novas, -{len(sumiram)} que sumiram)'
            )

    @staticmethod
    def near_any(point, pontos, tol):
        """Diz se há algum ponto da lista a menos de tol do ponto dado."""
        return any(math.hypot(point[0] - p[0], point[1] - p[1]) <= tol for p in pontos)

    def poops_callback(self, msg: PoseArray):
        poops = [(p.position.x, p.position.y) for p in msg.poses]
        if poops != self.poops:
            self.poops = poops
            self.get_logger().info(f'{len(poops)} poops no mapa')

    # =========================================================================
    #  Alvo
    # =========================================================================

    def choose_target(self, x, y, now):
        """Banana pendente de menor custo. Só o alvo atual atrai o robô.

        Custo = distância + blocked_penalty por poop perto do segmento até ela.
        Bananas puladas (ver give_up) ficam de fora até skip_time passar.
        """
        self.skipped = {b: ate for b, ate in self.skipped.items() if ate > now}
        candidatas = [b for b in self.pending if b not in self.skipped]
        if not candidatas:
            return None
        d0 = self.get_parameter('d0_poop').value
        penalidade = self.get_parameter('blocked_penalty').value

        def custo(b):
            bloqueios = sum(1 for p in self.poops
                            if self.dist_to_segment(p, (x, y), b) < d0)
            return math.hypot(b[0] - x, b[1] - y) + penalidade * bloqueios
        return min(candidatas, key=custo)

    @staticmethod
    def dist_to_segment(p, a, b):
        """Distância do ponto p ao segmento ab."""
        ax, ay = a
        dx, dy = b[0] - ax, b[1] - ay
        comp = dx * dx + dy * dy
        t = 0.0
        if comp > 1e-9:
            t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / comp))
        return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))

    def give_up(self, now, motivo):
        """Pula o alvo atual por skip_time e libera a escolha de outro."""
        self.skipped[self.target] = now + self.get_parameter('skip_time').value
        self.get_logger().info(
            f'desisti da banana em ({self.target[0]:.2f}, {self.target[1]:.2f}): '
            f'{motivo}; tento de novo em {self.get_parameter("skip_time").value:.0f} s'
        )
        self.target = None
        self.reset_progress()

    def reset_progress(self):
        self.target_since = None
        self.swirls_on_target = 0
        self.best_distance = math.inf   # menor distância já obtida do alvo
        self.stall_since = None         # desde quando não há progresso
        self.swirl_until = -math.inf    # até quando a tangencial fica ligada

    # =========================================================================
    #  Forças
    # =========================================================================

    def attractive(self, x, y, target):
        """Puxa para a banana alvo, com módulo saturado a k_att."""
        k_att = self.get_parameter('k_att').value
        att_range = self.get_parameter('att_range').value

        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return 0.0, 0.0, distance
        # Longe do alvo o módulo é constante; perto, cai junto com a distância.
        scale = k_att * min(1.0, distance / att_range) / distance
        return dx * scale, dy * scale, distance

    def repulsive(self, x, y):
        """Soma das repulsões dos poops dentro do raio de influência.

        Cada poop usa a forma clássica do campo potencial:
        F = k_rep * (1/d - 1/d0) / d² , apontando do poop para o robô.
        """
        k_rep = self.get_parameter('k_rep').value
        d0 = self.get_parameter('d0_poop').value
        d_min = self.get_parameter('d_min').value

        fx = fy = 0.0
        for px, py in self.poops:
            dx, dy = x - px, y - py
            distance = math.hypot(dx, dy)
            if distance > d0:
                continue
            distance = max(distance, d_min)
            magnitude = k_rep * (1.0 / distance - 1.0 / d0) / (distance * distance)
            fx += magnitude * dx / distance
            fy += magnitude * dy / distance
        return fx, fy

    def swirl(self, now, distance, ax, ay, rx, ry):
        """Componente tangencial que desempata um mínimo local.

        Devolve (0, 0) enquanto o robô estiver se aproximando do alvo.

        A tangente é perpendicular à repulsão, ou seja, acompanha a borda do
        obstáculo, e aponta para o lado que mais se aproxima do alvo. Esse lado
        é recalculado a cada ativação, mas como ele vem da geometria (e não de
        uma inversão a cada vez), o robô mantém o mesmo sentido enquanto
        contorna, em vez de oscilar na frente do obstáculo.
        """
        stall_time = self.get_parameter('stall_time').value
        stall_progress = self.get_parameter('stall_progress').value
        swirl_time = self.get_parameter('swirl_time').value
        swirl_gain = self.get_parameter('swirl_gain').value

        # Progresso é reduzir a menor distância já alcançada até o alvo.
        if distance < self.best_distance - stall_progress:
            self.best_distance = distance
            self.stall_since = None
        elif self.stall_since is None:
            self.stall_since = now

        if self.stall_since is not None and now - self.stall_since >= stall_time:
            # Escolhe um lado e mantém por swirl_time, para não ficar alternando.
            if now > self.swirl_until:
                self.swirl_sign = self.pick_side(ax, ay, rx, ry)
                self.swirl_until = now + swirl_time
                self.swirls_on_target += 1
                self.get_logger().info('campo empatado, contornando pela tangente')
            self.stall_since = None
            self.best_distance = distance

        if now >= self.swirl_until:
            return 0.0, 0.0

        # Base da tangente: a repulsão, se houver algum poop por perto; sem
        # nenhum, a própria atração (empate sem obstáculo, raro mas possível).
        bx, by = (rx, ry) if math.hypot(rx, ry) > 1e-6 else (ax, ay)
        base = math.hypot(bx, by)
        if base < 1e-6:
            return 0.0, 0.0
        # Módulo proporcional à atração, para a tangente não explodir junto com
        # a repulsão quando o robô encosta no obstáculo.
        gain = swirl_gain * math.hypot(ax, ay) * self.swirl_sign / base
        return -by * gain, bx * gain

    def pick_side(self, ax, ay, rx, ry):
        """Lado do contorno: o que deixa a tangente apontando para o alvo."""
        bx, by = (rx, ry) if math.hypot(rx, ry) > 1e-6 else (ax, ay)
        # Produto vetorial entre a base e a atração: o sinal dele diz de que
        # lado girar a base para chegar mais perto da direção do alvo.
        cross = bx * ay - by * ax
        if abs(cross) < 1e-9:
            return self.swirl_sign   # empate perfeito: mantém o lado atual
        return 1.0 if cross > 0.0 else -1.0

    # =========================================================================
    #  Ciclo de controle
    # =========================================================================

    def pose_callback(self, msg: PoseStamped):
        """Um ciclo de controle, disparado por cada pose que chega da ponte."""
        self.last_pose_time = self.get_clock().now().nanoseconds / 1e9
        if self.finished:
            return
        # Sem mapa ainda não é o fim: com a câmera montando o mapa, é hora de
        # procurar. O choose_target devolve None e a busca assume lá embaixo.

        x = msg.pose.position.x
        y = msg.pose.position.y
        self.robot_xy = (x, y)
        theta = yaw_of(msg.pose)
        now = self.get_clock().now().nanoseconds / 1e9

        # --- Alvo alcançado? -------------------------------------------------
        # O /dirt_script da cena recolhe a banana quando ela passa sob o sensor;
        # aqui basta ter chegado perto.
        collect_radius = self.get_parameter('collect_radius').value
        if self.target is not None:
            if math.hypot(self.target[0] - x, self.target[1] - y) <= collect_radius:
                self.pending.remove(self.target)
                self.gone.append(self.target)
                self.collected += 1
                self.collected_pub.publish(Int32(data=self.collected))
                self.get_logger().info(
                    f'banana {self.collected}/{len(self.bananas)} alcançada'
                )
                self.target = None
                self.reset_progress()

        if self.target is None:
            self.target = self.choose_target(x, y, now)
            self.reset_progress()
            self.target_since = now
            if self.target is None:
                # Nenhuma banana conhecida. Com o mapa vindo dos sinais isso é
                # o fim; com o mapa vindo da câmera, é hora de procurar.
                if self.get_parameter('search').value:
                    self.do_search(x, y, theta, now)
                    return
                self.publish_cmd(0.0, 0.0)
                self.finished = True
                self.get_logger().info(
                    f'todas as {self.collected} bananas coletadas, parando'
                )
                return
        self.search_phase = None
        self.search_since = None

        # --- Desistir de alvo inalcançável ------------------------------------
        if self.swirls_on_target >= self.get_parameter('max_swirls_per_target').value:
            self.give_up(now, f'{self.swirls_on_target} empates seguidos')
            return
        if (self.target_since is not None and
                now - self.target_since > self.get_parameter('target_timeout').value):
            self.give_up(now, 'tempo esgotado')
            return

        self.publish_target(msg.header.frame_id)

        # --- Resultante ------------------------------------------------------
        # atração da banana + repulsão dos poops (+ tangente, se travado).
        ax, ay, distance = self.attractive(x, y, self.target)
        rx, ry = self.repulsive(x, y)
        sx, sy = self.swirl(now, distance, ax, ay, rx, ry)
        fx, fy = ax + rx + sx, ay + ry + sy

        f_max = self.get_parameter('f_max').value
        magnitude = math.hypot(fx, fy)
        if magnitude > f_max:
            fx, fy = fx * f_max / magnitude, fy * f_max / magnitude
            magnitude = f_max
        self.publish_force(msg.header.frame_id, fx, fy)

        w_max = self.get_parameter('w_max').value
        if magnitude < 1e-6:
            # Sem direção definida: gira no lugar até o campo voltar a apontar.
            self.publish_cmd(0.0, w_max * 0.5)
            return

        # --- Da força para o comando de roda ---------------------------------
        # A direção da resultante é o rumo desejado; o erro vira giro.
        v_max = self.get_parameter('v_max').value
        k_heading = self.get_parameter('k_heading').value
        turn_in_place = self.get_parameter('turn_in_place').value

        error = wrap(math.atan2(fy, fx) - theta)
        w = max(-w_max, min(w_max, k_heading * error))
        if abs(error) > turn_in_place:
            v = 0.0                    # muito torto: acerta o rumo primeiro
        else:
            v = v_max * math.cos(error)  # avança conforme aponta para a força
        self.publish_cmd(v, w)

    # =========================================================================
    #  Busca
    # =========================================================================

    def do_search(self, x, y, theta, now):
        """Varre o ambiente até a câmera achar uma banana.

        Alterna girar no lugar, que mostra todo o entorno à câmera, e andar,
        que leva a câmera para outro pedaço da sala. Enquanto anda, a repulsão
        dos poops continua valendo, então a busca também os evita.
        """
        if self.search_since is None:
            self.search_since = now
            self.get_logger().info('nenhuma banana conhecida: procurando')
        if now - self.search_since > self.get_parameter('search_timeout').value:
            self.publish_cmd(0.0, 0.0)
            self.finished = True
            self.get_logger().info(
                f'{self.get_parameter("search_timeout").value:.0f} s sem achar banana; '
                f'parando com {self.collected} coletadas'
            )
            return

        spin_time = self.get_parameter('search_spin_time').value
        move_time = self.get_parameter('search_move_time').value
        if self.search_phase is None:
            self.search_phase = ('spin', now)
        fase, desde = self.search_phase
        if fase == 'spin' and now - desde >= spin_time:
            self.search_phase = fase, desde = ('move', now)
        elif fase == 'move' and now - desde >= move_time:
            self.search_phase = fase, desde = ('spin', now)

        if fase == 'spin':
            self.publish_cmd(0.0, self.get_parameter('search_w').value)
            return

        # Andando: segue o rumo atual, desviado pela repulsão dos poops.
        rx, ry = self.repulsive(x, y)
        fx, fy = math.cos(theta) + rx, math.sin(theta) + ry
        erro = wrap(math.atan2(fy, fx) - theta)
        w_max = self.get_parameter('w_max').value
        w = max(-w_max, min(w_max, self.get_parameter('k_heading').value * erro))
        v = self.get_parameter('search_v').value * math.cos(erro)
        self.publish_cmd(max(0.0, v), w)

    # =========================================================================
    #  Publicações
    # =========================================================================

    def publish_cmd(self, v, w):
        """Comando de velocidade do corpo; a ponte faz a cinemática das rodas."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def publish_target(self, frame_id):
        """Banana perseguida agora, para acompanhar no rviz ou no echo."""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.point.x, msg.point.y = self.target
        self.target_pub.publish(msg)

    def publish_force(self, frame_id, fx, fy):
        """Resultante do campo, útil para entender o que o robô está 'sentindo'."""
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.vector.x = float(fx)
        msg.vector.y = float(fy)
        self.force_pub.publish(msg)

    def check_pose_timeout(self):
        """Para o robô quando a pose para de chegar."""
        if self.last_pose_time is None or self.finished:
            return
        agora = self.get_clock().now().nanoseconds / 1e9
        atraso = agora - self.last_pose_time
        if atraso > self.get_parameter('pose_timeout').value:
            if not self.pose_lost:
                self.pose_lost = True
                self.get_logger().warn(
                    f'sem pose há {atraso:.1f} s: parando o robô. '
                    'A ponte está rodando e a simulação em play?'
                )
            self.stop()
        elif self.pose_lost:
            self.pose_lost = False
            self.get_logger().info('pose voltou, retomando')

    def stop(self):
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        # O ros2 launch repassa um segundo Ctrl+C durante a limpeza; sem isto
        # ele interrompe o destroy_node e imprime um traceback inofensivo.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
