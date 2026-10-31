"""Controle de posição em malha fechada: leva o robô a uma pose de referência.

É o controlador do pacote a_star_banana (Siegwart, *Introduction to Autonomous
Mobile Robots*, cap. 3.6), reaproveitado aqui sem mudança de lei: o
house_navigator só escolhe para onde mandar o robô; quem o move é este nó. Ele
não sabe de casa, de cômodo nem de voz — recebe uma pose e persegue.

    goal (PoseStamped)  --->  |                  |  --->  cmd_vel      Twist
    pose (PoseStamped)  --->  | position_control |  --->  goal_reached Bool

A lei, no referencial do erro (Δx, Δy do robô até o alvo, θ o rumo do robô e
θ_g o rumo pedido no alvo):

    ρ = √(Δx² + Δy²)              distância que falta
    α = atan2(Δy, Δx) − θ         para onde girar para apontar ao alvo
    β = θ_g − θ − α               quanto falta do rumo final

    v = k_ρ ρ                     (3)
    ω = k_α α + k_β β             (4)

Estabilidade: k_ρ > 0, k_β < 0 e k_α + 5/3·k_β − 2/π·k_ρ > 0. Os padrões
daqui (3, 8, −1.5) satisfazem as três.

As duas melhorias que vêm com ele
---------------------------------
1. Marcha a ré. A lei crua só gera v positivo, e com o alvo atrás o robô dá uma
   volta inteira para chegar. Se |α| > π/2 o alvo está atrás: o erro é medido
   com o robô "de costas" (α é refletido por π) e v sai negativo. A trajetória
   fica a mais curta das duas.
2. Velocidade constante. Como v e ω são proporcionais a ρ, o robô desacelera
   exponencialmente e a chegada demora. A trajetória não muda se v e ω forem
   escalados juntos (o que importa é a razão v/ω, que dá a curvatura): então o
   par é reescalado para |v| = v_const. Só na aproximação final, dentro de
   approach_radius, a velocidade volta a cair com ρ, para o robô parar em vez
   de passar do ponto.
"""

import math

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


def wrap(angle):
    """Normaliza um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_of(pose):
    """Rumo (rad) de uma geometry_msgs/Pose girada só em torno do eixo vertical."""
    return 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)


class PositionControl(Node):
    """Lei de controle de posição sobre uma pose de referência."""

    def __init__(self):
        super().__init__('position_control')

        # --- Ganhos da lei (eqs. 3 e 4) --------------------------------------
        self.declare_parameter('k_rho', 3.0)
        self.declare_parameter('k_alpha', 8.0)
        self.declare_parameter('k_beta', -1.5)

        # --- Limites e velocidade constante ----------------------------------
        self.declare_parameter('v_const', 0.22)   # velocidade de cruzeiro [m/s]
        self.declare_parameter('v_max', 0.30)     # teto de avanço [m/s]
        self.declare_parameter('w_max', 2.0)      # teto de giro [rad/s]
        # Dentro deste raio a velocidade volta a cair com ρ, para parar no ponto.
        self.declare_parameter('approach_radius', 0.25)   # [m]
        self.declare_parameter('v_min', 0.05)     # avanço mínimo na chegada [m/s]
        # Com erro de rumo maior que isto o robô gira parado: a lei sozinha
        # faria uma curva larga, e aqui o caminho já foi escolhido pelo A*.
        self.declare_parameter('turn_in_place', math.radians(75.0))
        self.declare_parameter('allow_reverse', True)

        # --- Chegada ---------------------------------------------------------
        self.declare_parameter('position_tolerance', 0.05)   # [m]
        # Alinhar o rumo final só quando ele importa. Nos waypoints do A* não
        # importa: o rumo pedido é a direção do próximo trecho, que o robô já
        # está seguindo.
        self.declare_parameter('align_final_yaw', False)
        self.declare_parameter('yaw_tolerance', math.radians(10.0))
        # Sem pose o robô andaria às cegas com o último comando.
        self.declare_parameter('pose_timeout', 2.0)          # [s]

        self.goal = None          # (x, y, yaw)
        self.reached = False      # já avisou que chegou neste alvo?
        self.last_pose_time = None
        self.pose_lost = False

        self.create_subscription(PoseStamped, 'goal', self.goal_callback, 10)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.reached_pub = self.create_publisher(Bool, 'goal_reached', 10)

        self.create_timer(0.2, self.check_pose_timeout)
        self.get_logger().info('controlador de posição pronto, esperando goal')

    # =========================================================================
    #  Entradas
    # =========================================================================

    def goal_callback(self, msg: PoseStamped):
        goal = (msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose))
        if self.goal is not None and goal == self.goal:
            return
        self.goal = goal
        self.reached = False
        self.get_logger().info(
            f'novo alvo: ({goal[0]:+.2f}, {goal[1]:+.2f}) '
            f'rumo {math.degrees(goal[2]):+.0f}°')

    def pose_callback(self, msg: PoseStamped):
        """Um ciclo de controle, disparado por cada pose que chega da ponte."""
        self.last_pose_time = self.get_clock().now().nanoseconds / 1e9
        if self.goal is None:
            return

        x, y = msg.pose.position.x, msg.pose.position.y
        theta = yaw_of(msg.pose)
        v, w, rho, yaw_error = self.control(x, y, theta)

        if self.arrived(rho, yaw_error):
            self.publish_cmd(0.0, 0.0)
            if not self.reached:
                self.reached = True
                self.reached_pub.publish(Bool(data=True))
                self.get_logger().info(
                    f'alvo alcançado a {rho:.3f} m '
                    f'(erro de rumo {math.degrees(yaw_error):+.0f}°)')
            return

        self.publish_cmd(v, w)

    # =========================================================================
    #  A lei de controle
    # =========================================================================

    def control(self, x, y, theta):
        """(v, ω, ρ, erro de rumo) para a pose atual, pelas eqs. (3) e (4)."""
        goal_x, goal_y, goal_yaw = self.goal
        dx, dy = goal_x - x, goal_y - y
        rho = math.hypot(dx, dy)
        yaw_error = wrap(goal_yaw - theta)

        if rho < 1e-6:
            # Em cima do ponto: só resta girar para o rumo pedido, se ele importa.
            w = self.clamp(self.get_parameter('k_alpha').value * yaw_error,
                           self.get_parameter('w_max').value)
            return 0.0, w, rho, yaw_error

        alpha = wrap(math.atan2(dy, dx) - theta)
        direction = 1.0
        if self.get_parameter('allow_reverse').value and abs(alpha) > math.pi / 2.0:
            # Alvo atrás: mede o erro com o robô de costas e anda de ré.
            alpha = wrap(alpha - math.pi)
            direction = -1.0
        beta = wrap(goal_yaw - theta - alpha)

        k_rho = self.get_parameter('k_rho').value
        v = direction * k_rho * rho
        w = (self.get_parameter('k_alpha').value * alpha
             + self.get_parameter('k_beta').value * beta)

        v, w = self.constant_speed(v, w, rho)

        if abs(alpha) > self.get_parameter('turn_in_place').value:
            v = 0.0   # muito torto: acerta o rumo antes de andar
        return v, w, rho, yaw_error

    def constant_speed(self, v, w, rho):
        """Escala o par (v, ω) sem mudar a trajetória.

        A curvatura do movimento é ω/v: multiplicar os dois pelo mesmo fator
        mantém o mesmo caminho, só percorrido mais rápido. É isso que tira a
        desaceleração exponencial da lei crua.
        """
        v_const = self.get_parameter('v_const').value
        approach = self.get_parameter('approach_radius').value
        v_min = self.get_parameter('v_min').value

        target = v_const
        if rho < approach:
            # Perto do alvo, volta a cair com ρ para não passar do ponto.
            target = max(v_min, v_const * rho / approach)

        if abs(v) > 1e-6:
            scale = target / abs(v)
            v, w = v * scale, w * scale

        v = self.clamp(v, self.get_parameter('v_max').value)
        w_max = self.get_parameter('w_max').value
        if abs(w) > w_max:
            # Saturar só ω mudaria a curvatura; reduzir os dois mantém a curva.
            v *= w_max / abs(w)
            w = self.clamp(w, w_max)
        return v, w

    def arrived(self, rho, yaw_error):
        if rho > self.get_parameter('position_tolerance').value:
            return False
        if not self.get_parameter('align_final_yaw').value:
            return True
        return abs(yaw_error) <= self.get_parameter('yaw_tolerance').value

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value))

    # =========================================================================
    #  Saídas
    # =========================================================================

    def publish_cmd(self, v, w):
        """Comando do corpo; a ponte faz a cinemática das rodas."""
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
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
            self.publish_cmd(0.0, 0.0)
        elif self.pose_lost:
            self.pose_lost = False
            self.get_logger().info('pose voltou, retomando')


def main(args=None):
    rclpy.init(args=args)
    node = PositionControl()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_cmd(0.0, 0.0)
        # O ros2 launch repassa um segundo Ctrl+C durante a limpeza; sem isto
        # ele interrompe o destroy_node e imprime um traceback inofensivo.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
