"""Controle de posição da base holonômica.

    goal (PoseStamped)  --->  |              |  --->  cmd_vel      Twist
    pose (PoseStamped)  --->  | base_control |  --->  goal_reached Bool

Por que não é a lei ρ-α-β do exercício anterior
-----------------------------------------------
O robô diferencial dos exercícios anteriores é NÃO-HOLONÔMICO: ele comanda duas
grandezas (v e ω) para controlar três coordenadas (x, y, θ), e não tem como
andar de lado. A lei ρ-α-β (Siegwart, cap. 3.6) existe para contornar isso: ela
mistura avanço e giro de modo que o robô descreva uma curva que acerta posição
e rumo ao mesmo tempo. Daí a necessidade de girar antes de andar, de andar de
ré quando o alvo está atrás, e todo o cuidado com a estabilidade dos ganhos.

A base de rodas suecas do youBot é HOLONÔMICA: as quatro rodas dão as três
liberdades do plano, e (vx, vy, ω) são independentes. Então o problema se
desfaz. Não há curva a planejar: o erro de posição é levado direto ao
referencial do robô e viaja como velocidade naquela direção, enquanto o erro de
rumo é corrigido em paralelo e sem interferir.

    e = (alvo - pose) no mundo               erro de posição
    e_robô = R(-θ) e                         o mesmo erro, visto pelo robô
    v = k_p * e_robô                         anda DIRETO para o alvo
    ω = k_θ * wrap(θ_alvo - θ)               e gira ao mesmo tempo

São três malhas proporcionais desacopladas em vez de uma lei acoplada — e isto
é exatamente o que a holonomia compra. Vale escrever no relatório: a mesma
tarefa, com uma base diferente, dispensa a parte mais elaborada do controlador
anterior.

O que a holonomia NÃO resolve
-----------------------------
1. ESCORREGAMENTO. A roda sueca perde: medido em `scene/calibrate_base.py`, o
   corpo faz 0,64 do que a cinemática pede, e só 0,36 quando o movimento é
   diagonal (duas rodas ficam paradas enquanto duas giram). É por isso que o
   controle é em malha fechada: o `cmd_gain` da ponte corrige a média, e a
   malha come o resto.
2. SATURAÇÃO. O motor de cada roda tem 2,5 N.m, e acima de ~0,25 m/s na roda a
   velocidade pedida deixa de ser entregue. Por isso `v_max` é baixo: pedir
   mais não anda mais rápido, só desacopla comando e movimento.
3. ZONA MORTA. Abaixo de uns 0,4 rad/s o giro quase não sai do atrito estático
   (ganho medido de 0,35 contra 0,65 em 0,6 rad/s). Então um erro de rumo
   pequeno recebe um comando MÍNIMO em vez de um comando proporcional
   minúsculo que não moveria o robô.
"""

import math

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from youbot_banana.kinematics import MAX_WHEEL_SPEED, WHEEL_RADIUS


def wrap(angle):
    """Normaliza um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(pose):
    """Rumo (rad) de uma geometry_msgs/Pose girada só em torno do eixo vertical."""
    return 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)


class BaseControl(Node):
    """Três malhas proporcionais desacopladas levando a base à pose pedida."""

    def __init__(self):
        super().__init__('base_control')

        # --- Ganhos ------------------------------------------------------------
        self.declare_parameter('k_position', 1.2)      # [1/s]
        self.declare_parameter('k_yaw', 2.0)           # [1/s]

        # --- Limites -----------------------------------------------------------
        # O teto vem do motor da roda: acima dele o comando deixa de virar
        # movimento. `MAX_WHEEL_SPEED * WHEEL_RADIUS` é a velocidade de roda no
        # limite do torque; metade dela é o teto de avanço, para sobrar margem
        # para o giro acontecer ao mesmo tempo.
        self.declare_parameter('v_max', 0.5 * MAX_WHEEL_SPEED * WHEEL_RADIUS)
        self.declare_parameter('w_max', 1.0)           # [rad/s]
        # Comandos mínimos, para vencer o atrito estático em vez de zumbir.
        self.declare_parameter('v_min', 0.03)          # [m/s]
        self.declare_parameter('w_min', 0.40)          # [rad/s]

        # --- Chegada -----------------------------------------------------------
        self.declare_parameter('position_tolerance', 0.03)       # [m]
        self.declare_parameter('yaw_tolerance', math.radians(4.0))
        # Alinhar o rumo importa quando o alvo é uma pose de preensão (o braço
        # precisa do objeto à frente), mas não nos waypoints do caminho: lá o
        # rumo pedido é só a direção do trecho, e exigi-lo faria o robô parar
        # para girar em cada curva — justamente o que uma base holonômica não
        # precisa fazer.
        self.declare_parameter('align_yaw', True)
        self.declare_parameter('pose_timeout', 2.0)              # [s]

        # --- Estado -------------------------------------------------------------
        self.goal = None              # (x, y, yaw)
        self.align = True             # o alvo atual pede rumo?
        self.reached = False
        self.last_pose_time = None
        self.pose_lost = False

        self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reached_pub = self.create_publisher(Bool, 'goal_reached', 10)

        self.create_timer(0.2, self.check_pose_timeout)
        self.get_logger().info(
            'controle da base pronto (holonômico), esperando goal. '
            f'v_max {self.get_parameter("v_max").value:.2f} m/s')

    # =========================================================================
    #  Entradas
    # =========================================================================

    def goal_callback(self, msg: PoseStamped):
        """Nova pose de referência, no frame do mundo.

        Um alvo cujo quaternion é exatamente nulo (todos os campos em zero, o
        que uma `Pose()` recém-criada tem) é lido como "não me importo com o
        rumo": é o caso dos waypoints do caminho. Um quaternion válido pede o
        rumo. Assim o mesmo tópico serve às duas necessidades, sem um segundo
        campo que a missão tivesse de preencher sempre.
        """
        quaternion = msg.pose.orientation
        degenerate = (abs(quaternion.w) < 1e-9 and abs(quaternion.z) < 1e-9
                      and abs(quaternion.x) < 1e-9 and abs(quaternion.y) < 1e-9)
        goal = (msg.pose.position.x, msg.pose.position.y,
                0.0 if degenerate else yaw_of(msg.pose))
        align = (not degenerate) and self.get_parameter('align_yaw').value
        if self.goal == goal and self.align == align:
            return
        self.goal, self.align, self.reached = goal, align, False
        self.get_logger().info(
            f'novo alvo: ({goal[0]:+.2f}, {goal[1]:+.2f})'
            + (f' rumo {math.degrees(goal[2]):+.0f}°' if align else ' (rumo livre)'))

    def pose_callback(self, msg: PoseStamped):
        """Um ciclo de controle, disparado por cada pose que chega da ponte."""
        self.last_pose_time = self.get_clock().now().nanoseconds / 1e9
        if self.goal is None:
            return

        x, y = msg.pose.position.x, msg.pose.position.y
        theta = yaw_of(msg.pose)
        forward, left, turn, distance, yaw_error = self.control(x, y, theta)

        if self.arrived(distance, yaw_error):
            self.publish(0.0, 0.0, 0.0)
            if not self.reached:
                self.reached = True
                self.reached_pub.publish(Bool(data=True))
                self.get_logger().info(
                    f'alvo alcançado a {distance * 100:.1f} cm'
                    + (f', rumo a {math.degrees(abs(yaw_error)):.1f}°'
                       if self.align else ''))
            return

        self.publish(forward, left, turn)

    # =========================================================================
    #  A lei de controle
    # =========================================================================

    def control(self, x, y, theta):
        """(frente, esquerda, giro, distância, erro de rumo) para a pose atual."""
        goal_x, goal_y, goal_yaw = self.goal
        error_x, error_y = goal_x - x, goal_y - y
        distance = math.hypot(error_x, error_y)
        yaw_error = wrap(goal_yaw - theta) if self.align else 0.0

        # O erro de posição, do mundo para o referencial do robô. É a única
        # conta que a holonomia exige: a partir daqui, "para onde" e "quanto
        # girar" são problemas separados.
        cos_theta, sin_theta = math.cos(theta), math.sin(theta)
        forward = cos_theta * error_x + sin_theta * error_y
        left = -sin_theta * error_x + cos_theta * error_y

        gain = self.get_parameter('k_position').value
        forward, left = forward * gain, left * gain
        turn = yaw_error * self.get_parameter('k_yaw').value

        forward, left = self.limit_translation(forward, left, distance)
        turn = self.limit_rotation(turn, yaw_error)
        return forward, left, turn, distance, yaw_error

    def limit_translation(self, forward, left, distance):
        """Satura o avanço mantendo a DIREÇÃO, e garante o mínimo útil.

        Reduzir as duas componentes no mesmo fator preserva a direção do
        movimento. Saturar uma só faria o robô andar para um lado diferente do
        que o erro pede — o mesmo erro que `scale_to_limit` evita nas rodas.
        """
        speed = math.hypot(forward, left)
        if speed < 1e-9:
            return 0.0, 0.0
        v_max = self.get_parameter('v_max').value
        v_min = self.get_parameter('v_min').value
        tolerance = self.get_parameter('position_tolerance').value

        wanted = min(speed, v_max)
        # Fora da tolerância, nunca menos que v_min: um comando proporcional
        # minúsculo não vence o atrito, e o robô ficaria parado a 5 cm do alvo
        # achando que está indo.
        if distance > tolerance:
            wanted = max(wanted, v_min)
        factor = wanted / speed
        return forward * factor, left * factor

    def limit_rotation(self, turn, yaw_error):
        """Satura o giro e garante o mínimo que vence o atrito estático."""
        if not self.align:
            return 0.0
        w_max = self.get_parameter('w_max').value
        w_min = self.get_parameter('w_min').value
        tolerance = self.get_parameter('yaw_tolerance').value
        if abs(yaw_error) <= tolerance:
            return 0.0
        magnitude = min(abs(turn), w_max)
        magnitude = max(magnitude, w_min)
        return math.copysign(magnitude, yaw_error)

    def arrived(self, distance, yaw_error):
        if distance > self.get_parameter('position_tolerance').value:
            return False
        if not self.align:
            return True
        return abs(yaw_error) <= self.get_parameter('yaw_tolerance').value

    # =========================================================================
    #  Saídas
    # =========================================================================

    def publish(self, forward, left, turn):
        """Twist do corpo; a ponte faz a cinemática das quatro rodas."""
        msg = Twist()
        msg.linear.x = float(forward)
        msg.linear.y = float(left)
        msg.angular.z = float(turn)
        self.cmd_pub.publish(msg)

    def check_pose_timeout(self):
        """Para o robô quando a pose para de chegar (ponte caiu, simulação parada)."""
        if self.last_pose_time is None:
            return
        late = self.get_clock().now().nanoseconds / 1e9 - self.last_pose_time
        if late > self.get_parameter('pose_timeout').value:
            if not self.pose_lost:
                self.pose_lost = True
                self.get_logger().warn(
                    f'sem pose há {late:.1f} s: parando o robô. '
                    'A ponte está rodando e a simulação em play?')
            self.publish(0.0, 0.0, 0.0)
        elif self.pose_lost:
            self.pose_lost = False
            self.get_logger().info('pose voltou, retomando')


def main(args=None):
    rclpy.init(args=args)
    node = BaseControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish(0.0, 0.0, 0.0)
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
