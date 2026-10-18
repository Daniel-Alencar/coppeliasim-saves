"""Controle do braço: cinemática inversa, trajetória cartesiana e garra.

Este nó é o que o exercício anterior não tinha. Lá o robô só precisava CHEGAR
num ponto, e o controlador de posição (a lei ρ-α-β) bastava. Aqui ele precisa
PEGAR, e isso traz a cinemática de manipulador: dado um ponto no espaço, achar
os ângulos das juntas que levam a garra até ele.

    arm_goal    (PoseStamped) --->  |             |  ---> arm_joints  Float64MultiArray
    arm_posture (String)      --->  | arm_control |  ---> arm_result  String
    gripper     (Float64)     --->  |             |  ---> grasped     Bool
    arm_state   (JointState)  --->  |             |
    gripper_gap (Float64)     --->  |             |

Não conhece o CoppeliaSim: recebe o estado das juntas pela ponte e devolve
posições de junta. A cinemática em si está em `kinematics.py`, testável sem
simulador (ver test/).

Por que a trajetória é INTERPOLADA
----------------------------------
A inversa dá o ponto de chegada, não o caminho até ele. Mandar direto a
configuração final faz cada junta correr para o alvo na velocidade que
conseguir, cada uma chegando num instante diferente: a garra descreve uma curva
qualquer pelo meio do caminho, e o movimento é brusco. Medido nesta cena: com
um salto de posição, a banana já presa ESCAPAVA da garra no levantamento — as
juntas do modelo vão a 6,28 rad/s, e o tranco vence o atrito dos dedos.

A correção é a de qualquer manipulador: interpolar. O nó divide o percurso em
passos de `cartesian_speed` metros por segundo, resolve a inversa em CADA passo
e manda a configuração correspondente. A ponta anda então em LINHA RETA, devagar
e com a orientação da garra mantida — e a banana fica na mão.

As posturas com nome
--------------------
Nem todo movimento é melhor descrito por um ponto. Recolher o braço, por
exemplo, quer dizer "fique compacto e longe do chão", e não um ponto no espaço:
exprimir isso como alvo cartesiano dá um ponto fora do envelope da garra
apontada para baixo. Por isso há posturas definidas em ESPAÇO DE JUNTAS, pedidas
por nome em `arm_posture`.
"""

import json
import math

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

from youbot_banana import kinematics
from youbot_banana.kinematics import ARM, Unreachable

#: Posturas em espaço de juntas (q0..q4), em radianos.
#:
#: - `home`  braço recolhido e erguido, que é a postura de deslocamento: a
#:           garra fica dentro da silhueta da base, sem varrer os obstáculos
#:           enquanto o robô atravessa a arena.
#: - `carry` igual, mas um pouco mais aberta, para a banana presa não bater na
#:           própria base durante o transporte.
#: - `zero`  todas as juntas em zero, com o braço esticado para cima. Só serve
#:           para conferir a cinemática: é nesta postura que a geometria foi
#:           medida.
POSTURES = {
    # Ponta a 0,18 m do eixo da junta 0 e 0,24 m de altura, garra apontada
    # para baixo. Os valores saem da PRÓPRIA inversa (ver o cabeçalho de
    # kinematics), e não de tentativa e erro: assim a postura de repouso está
    # DENTRO do envelope de preensão, e a reta cartesiana daqui até uma banana
    # no chão não atravessa região sem solução.
    'home': (-0.004, -0.089, -1.325, -1.727, 0.0),
    # Para andar com a banana presa: mais baixa e um pouco mais estendida, com
    # a garra ainda para baixo, para a banana não bater na própria base.
    'carry': (-0.004, -0.240, -1.594, -1.307, 0.0),
    # Todas as juntas em zero, braço esticado para cima. Só serve para conferir
    # a cinemática: é nesta postura que a geometria foi medida.
    'zero': (0.0, 0.0, 0.0, 0.0, 0.0),
}


def yaw_of(pose):
    """Rumo (rad) de uma geometry_msgs/Pose girada só em torno do eixo vertical."""
    return 2.0 * math.atan2(pose.orientation.z, pose.orientation.w)


class ArmControl(Node):
    """Leva a ponta da garra a pontos pedidos, e abre e fecha a garra."""

    def __init__(self):
        super().__init__('arm_control')

        # --- Trajetória -------------------------------------------------------
        # Velocidade da PONTA, não das juntas. Foi o valor que, nos ensaios de
        # preensão, levantou a banana sem arrancá-la da garra.
        self.declare_parameter('cartesian_speed', 0.08)      # [m/s]
        # Duração de um ciclo deste nó, que em modo stepping é o passo de
        # simulação: é o que converte velocidade da ponta em tamanho de passo
        # da interpolação.
        self.declare_parameter('cycle_time', 0.05)           # [s]
        # Tolerância para considerar a junta assentada no alvo.
        self.declare_parameter('joint_tolerance', math.radians(2.0))
        # Quantos ciclos o braço precisa ficar dentro da tolerância. Um ciclo
        # só pode ser coincidência na passagem.
        self.declare_parameter('settle_cycles', 5)
        # Sem assentar neste tempo, desiste e avisa: melhor a missão saber que
        # o braço empacou do que ela esperar para sempre.
        self.declare_parameter('move_timeout', 25.0)         # [s]

        # --- Preensão ---------------------------------------------------------
        self.declare_parameter('approach', math.pi)   # garra apontada para baixo
        # Vão abaixo do qual se considera que a garra fechou NO VAZIO. A garra
        # vazia chega a 26 mm; com algo entre os dedos, para antes.
        self.declare_parameter('empty_gap', 0.032)           # [m]
        # Quantos ciclos o vão tem de ficar parado para dizer que travou.
        self.declare_parameter('stall_cycles', 8)
        self.declare_parameter('stall_tolerance', 0.0005)    # [m]

        # --- Estado -----------------------------------------------------------
        self.joints = None            # últimas posições das 5 juntas do braço
        self.gap = None               # vão atual da garra [m]
        self.path = []                # pontos que faltam da trajetória
        self.goal = None              # (x, y, z) final, no frame da base (ROS)
        self.goal_yaw = 0.0           # azimute do eixo maior do objeto
        self.posture = None           # alvo em espaço de juntas, quando houver
        self.command = None           # última configuração publicada
        self.settled = 0
        self.started_at = None
        self.announced = False        # já publicou o resultado deste alvo?
        self.gripper_goal = 1.0
        self.gap_history = []
        self.grasped = False
        # O relógio da cena, vindo da ponte. Os prazos são medidos NELE, e não
        # no do computador: com a simulação a 0,3x do tempo real — o que
        # acontece numa máquina carregada — um prazo de 25 s de relógio viraria
        # 7 s de simulação, e o braço seria declarado empacado no meio de um
        # movimento que estava indo bem. É a mesma lição da ponte do exercício
        # anterior.
        self.sim_clock = None

        # --- Comunicação ROS ---------------------------------------------------
        self.create_subscription(PoseStamped, 'arm_goal', self.goal_callback, 10)
        self.create_subscription(String, 'arm_posture', self.posture_callback, 10)
        self.create_subscription(Float64, 'gripper', self.gripper_callback, 10)
        self.create_subscription(JointState, 'arm_state', self.state_callback, 10)
        self.create_subscription(Float64, 'gripper_gap', self.gap_callback, 10)
        self.create_subscription(Float64, 'sim_time', self.clock_callback, 10)

        self.joints_pub = self.create_publisher(Float64MultiArray, 'arm_joints', 10)
        self.result_pub = self.create_publisher(String, 'arm_result', 10)
        self.grasped_pub = self.create_publisher(Bool, 'grasped', 10)

        self.get_logger().info(
            'controle do braço pronto. envelope de preensão por cima: '
            f'{self.envelope()[0]:.2f} a {self.envelope()[1]:.2f} m do eixo da '
            'junta 0')

    # =========================================================================
    #  Entradas
    # =========================================================================

    def goal_callback(self, msg: PoseStamped):
        """Alvo cartesiano para a PONTA da garra.

        A mensagem é interpretada assim:

            position     onde a ponta tem de ficar, no frame da base (base_link,
                         convenção do ROS: +x para a frente do robô)
            orientation  só o yaw é usado, e ele é o azimute do EIXO MAIOR do
                         objeto a pegar. Não é a orientação da garra: quem a
                         calcula é `kinematics.grasp_roll`, que gira a junta 4
                         para os dedos fecharem atravessados nesse eixo.

        Escolher o yaw do objeto em vez da orientação da garra é deliberado: a
        missão sabe como a banana está deitada, mas não tem por que saber como
        a garra precisa girar para pegá-la — isso é cinemática, e mora aqui.
        """
        self.goal = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.goal_yaw = yaw_of(msg.pose)
        self.posture = None
        self.announced = False
        self.settled = 0
        self.started_at = None
        self.path = []        # a trajetória é traçada no próximo state_callback
        self.get_logger().info(
            f'alvo da garra: ({self.goal[0]:+.3f}, {self.goal[1]:+.3f}, '
            f'{self.goal[2]:+.3f}) no frame da base, '
            f'objeto a {math.degrees(self.goal_yaw):+.0f}°')

    def posture_callback(self, msg: String):
        """Postura com nome, em espaço de juntas (ver POSTURES)."""
        name = msg.data.strip()
        if name not in POSTURES:
            self.fail(f'postura desconhecida: "{name}". '
                      f'conhecidas: {", ".join(sorted(POSTURES))}')
            return
        self.posture = POSTURES[name]
        self.goal = None
        self.path = []
        self.announced = False
        self.settled = 0
        self.started_at = None
        self.get_logger().info(f'postura pedida: {name}')

    def gripper_callback(self, msg: Float64):
        """Abertura pedida: 1 = aberta, 0 = fechar até travar no objeto."""
        goal = max(0.0, min(1.0, msg.data))
        if abs(goal - self.gripper_goal) > 1e-6:
            self.gripper_goal = goal
            self.gap_history = []
            if goal > 0.5:
                # Abrir solta o que estava preso.
                self.set_grasped(False)
            self.get_logger().info(
                f'garra: {"abrir" if goal > 0.5 else "fechar"} (alvo {goal:.2f})')

    def gap_callback(self, msg: Float64):
        self.gap = msg.data
        self.check_grasp()

    def clock_callback(self, msg: Float64):
        self.sim_clock = msg.data

    def state_callback(self, msg: JointState):
        """Um ciclo de controle, disparado por cada estado que chega da ponte.

        Mesmo desenho do controlador de posição do exercício anterior: quem dá
        o ritmo é o estado que chega, e não um timer deste nó. Em modo stepping
        isso faz um comando por passo de simulação, exatamente.
        """
        if len(msg.position) < 5:
            return
        self.joints = list(msg.position[:5])

        if self.posture is not None:
            self.follow_joints(self.posture)
            return
        if self.goal is None:
            return
        if not self.path:
            if not self.plan_path():
                return
        self.follow_path()

    # =========================================================================
    #  Trajetória
    # =========================================================================

    def envelope(self, height=0.03):
        """Faixa de raio em que a preensão por cima tem solução, nesta altura.

        Vale para o log e para a mensagem de erro: dizer "fora do envelope"
        ajuda menos que dizer entre que raios ele existe.
        """
        approach = self.get_parameter('approach').value
        radii = []
        radius = 0.05
        while radius <= 0.45:
            try:
                kinematics.arm_ik_best((0.0, ARM.axis_y + radius, height),
                                       approach=approach)
                radii.append(radius)
            except Unreachable:
                pass
            radius += 0.005
        return (min(radii), max(radii)) if radii else (0.0, 0.0)

    def plan_path(self):
        """Traça o caminho até o alvo, em linha reta se der, por juntas se não.

        A conferência é feita ANTES de começar a andar. Descobrir no meio do
        percurso que um ponto não tem solução deixaria o braço parado numa
        postura qualquer, com a banana na mão e a missão sem saber o que houve.

        DOIS MODOS, e a escolha não é estética:

        CARTESIANO (preferido) — a ponta anda em linha reta. É o que se quer
        perto do objeto: descer em volta da banana e levantar com ela exigem
        que a garra não passeie pelos lados no caminho.

        ESPAÇO DE JUNTAS (reserva) — interpola de configuração a configuração.
        Não mantém a ponta em linha reta, mas tem uma garantia que o cartesiano
        não tem: o caminho é SEMPRE válido. Cada limite de junta é um
        intervalo, e a interpolação entre dois pontos de um intervalo fica
        dentro dele; então, se as duas pontas cabem nos limites, tudo entre
        elas também cabe.

        É o que resolve o caso que travou a primeira versão: o braço começa na
        postura dobrada para trás do modelo, cuja ponta fica do lado OPOSTO ao
        eixo da junta 0. A reta cartesiana de lá até a banana atravessa o eixo,
        e sobre o eixo o azimute pede q0 = 180°, fora do limite de ±169°. O
        alvo era perfeitamente alcançável; o caminho até ele não era.
        """
        start = kinematics.ref_to_ros(kinematics.arm_fk(self.joints))
        step = (self.get_parameter('cartesian_speed').value
                * self.get_parameter('cycle_time').value)
        steps = max(1, int(math.dist(start, self.goal) / step))

        # O alvo tem de ter solução de qualquer modo: sem ele não há o que fazer.
        try:
            final = self.solve(self.goal)
        except Unreachable as error:
            low, high = self.envelope(self.goal[2])
            self.fail(
                f'o alvo não tem solução: {error}. '
                f'Nesta altura o braço pega entre {low:.2f} e {high:.2f} m do '
                'eixo da junta 0 — reposicione a base.')
            self.goal = None
            return False

        path = []
        for index in range(1, steps + 1):
            fraction = index / steps
            point = tuple(start[i] + fraction * (self.goal[i] - start[i])
                          for i in range(3))
            try:
                path.append(self.solve(point))
            except Unreachable as error:
                self.get_logger().info(
                    f'a reta até o alvo sai do envelope a {fraction:.0%} do '
                    f'caminho ({error}); indo por espaço de juntas, que é '
                    'sempre válido mas não mantém a ponta em linha reta.')
                self.path = self.joint_path(final, steps)
                self.started_at = self.now()
                return True
        self.path = path
        self.started_at = self.now()
        return True

    def joint_path(self, final, steps):
        """Interpolação de configuração a configuração, junta por junta."""
        start = list(self.joints)
        return [[start[j] + (index / steps) * (final[j] - start[j])
                 for j in range(5)]
                for index in range(1, steps + 1)]

    def solve(self, point):
        """Inversa de um ponto do frame da base (ROS) para as cinco juntas.

        Aqui ficam juntas as duas conversões que o resto do código não precisa
        conhecer: do frame do ROS para o da cena, e do yaw do objeto para o
        giro da junta 4.
        """
        approach = self.get_parameter('approach').value
        target = kinematics.ros_to_ref(point)
        configuration = list(kinematics.arm_ik_best(target, approach=approach))
        # O yaw do objeto também é dado na convenção do ROS; no frame da cena
        # ele aparece girado de 90°, porque o +x do ROS é o +y da cena.
        object_yaw_ref = self.goal_yaw + math.pi / 2.0
        configuration[4] = kinematics.grasp_roll(object_yaw_ref, configuration[0])
        return configuration

    def follow_path(self):
        """Publica o próximo ponto da trajetória, e o alvo quando ela acaba."""
        if len(self.path) > 1:
            self.command = self.path.pop(0)
            self.publish(self.command)
            return
        # No último ponto o nó deixa de avançar e passa a esperar o braço
        # assentar: é só então que a missão pode fechar a garra.
        self.command = self.path[0]
        self.publish(self.command)
        self.check_settled(self.command)

    def follow_joints(self, target):
        """Vai para uma postura em espaço de juntas, com a mesma espera."""
        if self.started_at is None:
            self.started_at = self.now()
        self.command = list(target)
        self.publish(self.command)
        self.check_settled(self.command)

    def check_settled(self, target):
        """Declara o movimento terminado quando as juntas param no alvo."""
        if self.announced:
            return
        tolerance = self.get_parameter('joint_tolerance').value
        if all(abs(actual - wanted) <= tolerance
               for actual, wanted in zip(self.joints, target)):
            self.settled += 1
            if self.settled >= self.get_parameter('settle_cycles').value:
                self.announced = True
                self.succeed()
            return
        self.settled = 0
        if (self.started_at is not None
                and self.now() - self.started_at
                > self.get_parameter('move_timeout').value):
            self.announced = True
            worst = max(range(5),
                        key=lambda i: abs(self.joints[i] - target[i]))
            self.fail(
                f'o braço não assentou em '
                f'{self.get_parameter("move_timeout").value:.0f} s. '
                f'junta {worst} parou a '
                f'{math.degrees(abs(self.joints[worst] - target[worst])):.1f}° '
                'do alvo — algo está no caminho, ou a junta bateu no limite.')

    def publish(self, configuration):
        self.joints_pub.publish(Float64MultiArray(data=list(configuration)))

    # =========================================================================
    #  Preensão
    # =========================================================================

    def check_grasp(self):
        """Decide se a garra travou em algo, só pela posição dos dedos.

        Fechando no vazio, os dedos vão até o fim (vão de 26 mm). Com um objeto
        entre eles, o motor TRAVA antes — e é essa parada, acima do vão vazio,
        que diz que pegou. Não é preciso sensor de força: a própria cinemática
        da garra conta.
        """
        if self.gripper_goal > 0.5 or self.gap is None:
            return
        history = self.gap_history
        history.append(self.gap)
        needed = self.get_parameter('stall_cycles').value
        if len(history) > needed:
            history.pop(0)
        if len(history) < needed:
            return
        if max(history) - min(history) > self.get_parameter('stall_tolerance').value:
            return          # ainda está fechando
        stalled_at = sum(history) / len(history)
        self.set_grasped(stalled_at > self.get_parameter('empty_gap').value)

    def set_grasped(self, value):
        if value == self.grasped:
            return
        self.grasped = value
        self.grasped_pub.publish(Bool(data=value))
        if value:
            self.get_logger().info(
                f'garra travou com {self.gap * 1000:.0f} mm entre os dedos: '
                'há algo preso')
        else:
            self.get_logger().info('garra solta')

    # =========================================================================
    #  Resultados
    # =========================================================================

    def succeed(self):
        self.result_pub.publish(String(data=json.dumps({'ok': True})))
        self.get_logger().info('braço no alvo')

    def fail(self, reason):
        self.result_pub.publish(String(data=json.dumps(
            {'ok': False, 'reason': reason})))
        self.get_logger().warn(reason)

    def now(self):
        """O relógio da simulação quando a ponte o publica; senão, o do sistema.

        Cair para o relógio do sistema deixa o nó utilizável sem a ponte (nos
        testes, por exemplo), sem perder a medida certa quando ela está lá.
        """
        if self.sim_clock is not None:
            return self.sim_clock
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = ArmControl()
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
