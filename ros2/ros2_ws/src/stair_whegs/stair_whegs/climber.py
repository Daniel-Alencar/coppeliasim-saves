"""A percepção e a decisão: achar a escada, alinhar, subir e parar no topo.

    probes   (Float64MultiArray) --->  |         |  ---> drive      (Twist)  -> gait
    attitude (Vector3Stamped)    --->  | climber |  ---> gait       (String) -> gait
    pose     (PoseStamped)       --->  |         |  ---> wheel_mode (String) -> ponte
    sim_time (Float64)           --->  |         |  ---> status     (String)
                                       |         |  ---> done       (Bool)   -> ponte

Este nó é a resposta à pergunta que dá sentido ao trabalho: subir escada não é
só girar as rodas. Girar as rodas o rover já faz sozinho — e, de olhos
fechados, ele sobe e depois atravessa o patamar e cai do outro lado, que foi
exatamente o que os primeiros ensaios registraram. O que falta é a camada que
olha o terreno e decide.

    CALIBRA ──> PROCURA ──> ALINHA ──> SOBE ──> TOPO ──> FIM
     roda        roda        roda      GARRA    roda
                   ^                     │
                   └──── RECUA ◀─────────┘   (empacou no degrau)
                         garra

ESTE NÓ É O GATILHO DA RODA
---------------------------
A linha de baixo do diagrama é o estado da roda transformável, e é este nó
quem o escolhe. Na roda original não há nó nenhum: a perna gatilho esbarra no
obstáculo e o atrito abre as três. Aqui o gatilho é a SONDA — o mesmo sinal
que já dizia "há degrau à frente" passa a dizer também "abre a garra".

A troca não é de graça, e por isso ela tem um tempo próprio (`transform_time`):
abrir as pernas LEVANTA o rover de um raio para quase dois, e fazer isso
girando, encostado na face do espelho, é o jeito de prender a ponta de uma
perna embaixo da quina. O rover para, abre, e só então ataca.

TRÊS SINAIS, E O QUE CADA UM RESOLVE
------------------------------------
1. ENCURTAMENTO DA SONDA. As duas sondas apontam 30° para baixo; sobre piso
   plano, cada uma percorre uma distância previsível até o chão. Com um
   espelho à frente, o raio bate na face ANTES disso. Não se usa limiar
   absoluto: usa-se a razão entre a leitura e a referência de piso plano, que
   o nó MEDE no início (estado CALIBRA) em vez de receber pronta. Assim o
   mesmo código serve a um rover de outra altura.

2. DIFERENÇA ENTRE AS SONDAS. Com o rover torto em relação à face do degrau,
   uma sonda a encontra antes da outra. A diferença dá o ângulo, e o ângulo
   vira comando de giro. Chegar torto é o jeito mais fácil de falhar numa
   escada: um lado engancha primeiro, o corpo gira e o rover desce de lado.

3. PITCH DO IMU. Medido nesta cena: ~3° no plano, 21° a 29° durante a
   escalada. É o sinal mais limpo dos três, e é o que diz quando a escalada
   começou e quando acabou.

POR QUE TUDO PASSA POR UMA JANELA
---------------------------------
O wheg de três raios faz o corpo subir e descer 25 mm e balançar ±15° a cada
passo. Nenhuma leitura instantânea significa coisa alguma: a mesma sonda, no
mesmo lugar, lê 0,04 m ou 0,18 m conforme a fase da marcha. Toda decisão deste
nó é tomada sobre a MEDIANA de uma janela de aproximadamente um passo — a
mediana, e não a média, porque as leituras perdidas (raio no vazio) aparecem
como o alcance máximo e puxariam uma média para longe.
"""

import math
import statistics

from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray, String

from stair_whegs import geometry

CALIBRA = 'calibra'
PROCURA = 'procura'
ALINHA = 'alinha'
SOBE = 'sobe'
RECUA = 'recua'
TOPO = 'topo'
FIM = 'fim'

#: Inclinação das sondas abaixo da horizontal [rad]. Tem de bater com a cena
#: (scene/build_scene.add_sensors); é usada para converter distância ao longo
#: do raio em distância horizontal.
PROBE_TILT = math.radians(30.0)


class Climber(Node):
    """Olha o terreno e decide o que a marcha deve fazer."""

    def __init__(self):
        super().__init__('climber')

        # --- Velocidades --------------------------------------------------
        self.declare_parameter('search_speed', 3.0)     # [rad/s] no plano
        self.declare_parameter('climb_speed', 3.0)      # [rad/s] na escada
        self.declare_parameter('align_turn', 1.2)       # [rad/s] girando
        self.declare_parameter('back_speed', 2.0)       # [rad/s] recuando

        # --- Percepção ------------------------------------------------------
        # Janela de decisão, em segundos de simulação. Aproximadamente um passo
        # da marcha: curta demais e o balanço passa, longa demais e o rover
        # reage tarde.
        self.declare_parameter('window', 0.7)
        self.declare_parameter('calibration_time', 1.5)   # [s] medindo o plano
        # A sonda acusa degrau quando lê menos que esta fração da referência.
        self.declare_parameter('step_ratio', 0.70)
        # E acusa vazio quando lê mais que esta fração do ALCANCE do sensor.
        self.declare_parameter('void_ratio', 0.90)
        # Desalinhamento tolerado antes de girar [rad].
        self.declare_parameter('align_tolerance', math.radians(5.0))
        # Pitch acima do qual se considera que está subindo [rad].
        self.declare_parameter('climb_pitch', math.radians(12.0))
        # E abaixo do qual se considera plano.
        self.declare_parameter('flat_pitch', math.radians(7.0))
        # Rolagem acima da qual o rover está deitado e não há o que fazer.
        # Ver `rolled_over`.
        self.declare_parameter('roll_limit', math.radians(60.0))

        # --- Progresso e desistência ----------------------------------------
        # Sem ganhar esta altura dentro de `stall_time`, considera-se empacado.
        self.declare_parameter('progress_height', 0.015)   # [m]
        self.declare_parameter('stall_time', 6.0)          # [s de simulação]
        self.declare_parameter('back_time', 2.0)           # [s] recuando
        self.declare_parameter('max_retries', 3)
        # Duas marchas, e não uma: `alternada` anda melhor, `junta` sobe
        # melhor, e a razão medida é a ROLAGEM (ver `geometry.CLIMB_GAIT`).
        self.declare_parameter('climb_gait', geometry.CLIMB_GAIT)
        self.declare_parameter('search_gait', geometry.DEFAULT_GAIT)
        # Tempo parado enquanto as pernas abrem ou fecham [s de simulação].
        self.declare_parameter('transform_time', 0.8)
        self.declare_parameter('stop_when_done', True)

        # --- Estado ----------------------------------------------------------
        self.state = CALIBRA
        # A roda começa fechada, como a cena a constrói.
        self.wheel = geometry.RODA
        self.clock = None
        self.since = None
        self.probe_l, self.probe_r, self.edge = [], [], []
        self.pitch_window = []
        self.roll_window = []
        self.probe_range = None     # alcance do sensor, descoberto na calibração
        self.flat_reference = None  # leitura de piso plano, medida
        self.pose = None
        self.best_z = None
        self.progress_at = None
        self.retries = 0
        self.climb_started_z = None

        self.create_subscription(Float64MultiArray, 'probes',
                                 self.probes_callback, 10)
        self.create_subscription(Vector3Stamped, 'attitude',
                                 self.attitude_callback, 10)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.create_subscription(Float64, 'sim_time', self.clock_callback, 10)

        self.drive_pub = self.create_publisher(Twist, 'drive', 10)
        self.gait_pub = self.create_publisher(String, 'gait', 10)
        self.wheel_pub = self.create_publisher(String, 'wheel_mode', 10)
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.done_pub = self.create_publisher(Bool, 'done', 10)

        self.get_logger().info('escalador pronto: medindo o piso plano')

    # =========================================================================
    #  Entradas
    # =========================================================================

    def probes_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            return
        self.probe_l.append(msg.data[0])
        self.probe_r.append(msg.data[1])
        self.edge.append(msg.data[2])
        if self.probe_range is None:
            # A ponte publica o ALCANCE quando nada é detectado, então o maior
            # valor que aparece é o alcance do sensor.
            self.probe_range = max(msg.data)
        else:
            self.probe_range = max(self.probe_range, max(msg.data))
        self.trim()

    def attitude_callback(self, msg: Vector3Stamped):
        self.pitch_window.append(msg.vector.y)
        self.roll_window.append(msg.vector.x)
        self.trim()

    def pose_callback(self, msg: PoseStamped):
        self.pose = (msg.pose.position.x, msg.pose.position.y,
                     msg.pose.position.z)

    def clock_callback(self, msg: Float64):
        if self.since is None:
            self.since = msg.data
        self.clock = msg.data
        self.tick()

    def trim(self):
        """Mantém as janelas do tamanho pedido, em amostras."""
        # A ponte publica a 20 Hz em stepping (um passo de 50 ms).
        size = max(3, int(self.get_parameter('window').value / 0.05))
        for window in (self.probe_l, self.probe_r, self.edge,
                       self.pitch_window, self.roll_window):
            del window[:-size]

    # =========================================================================
    #  Percepção
    # =========================================================================

    def ready(self):
        """Há janela cheia o bastante para decidir?"""
        return (self.clock is not None and self.pose is not None
                and len(self.probe_l) >= 3 and len(self.pitch_window) >= 3)

    @staticmethod
    def median(window):
        return statistics.median(window) if window else None

    def pitch(self):
        """Pitch mediano da janela [rad]: positivo é nariz para cima."""
        return self.median(self.pitch_window)

    def rolled_over(self):
        """O rover está deitado de lado?

        Modo de falha que só existe desde a roda transformável, e que nenhuma
        medida de altura denuncia. Abrir a garra leva o centro de massa de um
        raio para quase dois e não mexe na bitola: o ângulo de tombamento
        lateral cai de 56° para 48° (ver `scene/characterize.py bitola`), e o
        que sobra de margem a marcha consome balançando o corpo em rolagem.

        Deitado, o rover continua girando as rodas e continua ANDANDO — para
        trás, escada abaixo. Sem este teste a máquina de estados fica em SOBE
        esperando um ganho de altura que não vem, gasta as três tentativas de
        RECUA e só então desiste, com o rover longe de onde empacou.
        """
        roll = self.median(self.roll_window)
        return (roll is not None
                and abs(roll) > self.get_parameter('roll_limit').value)

    def ground_ahead(self):
        """Distância mediana das duas sondas [m]."""
        return (self.median(self.probe_l) + self.median(self.probe_r)) / 2.0

    def step_ahead(self):
        """Há um espelho à frente?

        Pela RAZÃO entre a leitura e a referência de piso plano medida na
        calibração, e não por um limiar absoluto — assim o critério não depende
        da altura em que os sensores foram montados.
        """
        if self.flat_reference is None:
            return False
        ratio = self.ground_ahead() / self.flat_reference
        return ratio < self.get_parameter('step_ratio').value

    def void_ahead(self):
        """O terreno sumiu à frente? (a beirada do patamar)

        O sensal `edge` aponta mais para baixo que as sondas: enquanto há piso
        ele lê curto, e na beirada o raio mergulha no vazio e a leitura vai ao
        alcance máximo.
        """
        if self.probe_range is None:
            return False
        return (self.median(self.edge)
                > self.get_parameter('void_ratio').value * self.probe_range)

    def misalignment(self):
        """Ângulo entre a frente do rover e a face do degrau [rad].

        As duas sondas estão separadas lateralmente; se o rover está torto, uma
        encontra a face antes da outra. A diferença das leituras, projetada na
        horizontal, sobre a separação, dá a tangente do ângulo.

        Positivo quer dizer que a sonda ESQUERDA está mais longe — o rover está
        virado para a esquerda e precisa girar para a direita.
        """
        left, right = self.median(self.probe_l), self.median(self.probe_r)
        separation = 2.0 * geometry.TRACK / 3.0
        return math.atan2((left - right) * math.cos(PROBE_TILT), separation)

    # =========================================================================
    #  A máquina de estados
    # =========================================================================

    def tick(self):
        if self.state == FIM or not self.ready():
            return
        if self.state != CALIBRA and self.rolled_over():
            roll = math.degrees(self.median(self.roll_window))
            self.get_logger().error(
                f'rover deitado ({roll:+.0f}° de rolagem). Com a garra aberta '
                f'o centro de massa vai a {geometry.claw_reach() * 1000:.0f} mm '
                f'e a bitola é de {geometry.TRACK * 1000:.0f} mm: o ângulo de '
                f'tombamento lateral é de '
                f'{math.degrees(math.atan2(geometry.TRACK / 2.0, geometry.claw_reach())):.0f}°.')
            self.finish(False)
            return
        handler = {
            CALIBRA: self.do_calibra,
            PROCURA: self.do_procura,
            ALINHA: self.do_alinha,
            SOBE: self.do_sobe,
            RECUA: self.do_recua,
            TOPO: self.do_topo,
        }.get(self.state)
        if handler:
            handler()
        self.status_pub.publish(String(data=self.state))
        # Republicado a cada ciclo, e não só na troca: o tópico não é latched,
        # e uma ponte que suba depois do climber perderia o único anúncio.
        self.wheel_pub.publish(String(data=self.wheel))

    #: Em que estado a roda tem de estar em cada estado da máquina. É a linha
    #: de baixo do diagrama do cabeçalho, escrita uma vez só: andar e procurar
    #: é trabalho de RODA, atacar degrau é trabalho de GARRA.
    WHEEL_FOR = {
        CALIBRA: geometry.RODA,
        PROCURA: geometry.RODA,
        ALINHA: geometry.RODA,
        SOBE: geometry.GARRA,
        RECUA: geometry.GARRA,
        TOPO: geometry.RODA,
    }

    def enter(self, state):
        self.state = state
        self.since = self.clock
        wheel = self.WHEEL_FOR.get(state, self.wheel)
        if wheel != self.wheel:
            self.wheel = wheel
            self.get_logger().info(
                f'-> {state}  (roda {wheel}: a garra alcança '
                f'{geometry.claw_reach() * 1000:.0f} mm contra '
                f'{geometry.WHEG_RADIUS * 1000:.0f} mm do aro)')
        else:
            self.get_logger().info(f'-> {state}')

    def transforming(self):
        """Verdadeiro enquanto as pernas ainda estão indo para o lugar.

        Quem acabou de entrar num estado que muda a roda tem de ESPERAR. O
        rover sobe de um raio para quase dois ao abrir, e começar a girar no
        meio disso é prender uma ponta de perna debaixo da quina do degrau.
        """
        return self.elapsed() < self.get_parameter('transform_time').value

    def elapsed(self):
        return 0.0 if self.since is None else self.clock - self.since

    def do_calibra(self):
        """Mede a leitura das sondas sobre piso plano, parado.

        É a referência de tudo o que vem depois. Medir em vez de receber um
        número fixo faz o nó funcionar com outra altura de chassi ou outra
        inclinação de sensor sem mexer em código — e o valor medido já inclui
        o balanço da marcha, que um cálculo geométrico não teria.
        """
        self.drive(0.0)
        if self.elapsed() < self.get_parameter('calibration_time').value:
            return
        self.flat_reference = self.ground_ahead()
        expected = (geometry.WHEG_RADIUS + 0.004 + 0.015) / math.sin(PROBE_TILT)
        self.get_logger().info(
            f'piso plano: sondas a {self.flat_reference * 1000:.0f} mm '
            f'(geometria diria {expected * 1000:.0f} mm; a diferença é o '
            f'balanço do corpo). Degrau será acusado abaixo de '
            f'{self.flat_reference * self.get_parameter("step_ratio").value * 1000:.0f} mm')
        self.gait_pub.publish(String(data=self.get_parameter('search_gait').value))
        self.enter(PROCURA)

    def do_procura(self):
        """Anda para a frente no plano até achar um degrau."""
        self.drive(self.get_parameter('search_speed').value)
        if self.step_ahead():
            self.get_logger().info(
                f'degrau à frente: sondas a {self.ground_ahead() * 1000:.0f} mm '
                f'contra {self.flat_reference * 1000:.0f} mm do piso plano')
            self.enter(ALINHA)

    def do_alinha(self):
        """Gira no lugar até ficar de frente para a face do degrau."""
        error = self.misalignment()
        tolerance = self.get_parameter('align_tolerance').value
        if abs(error) <= tolerance:
            self.get_logger().info(
                f'alinhado a {math.degrees(error):+.1f}° da face do degrau')
            self.gait_pub.publish(
                String(data=self.get_parameter('climb_gait').value))
            self.best_z = self.pose[2]
            self.progress_at = self.clock
            self.climb_started_z = self.pose[2]
            self.enter(SOBE)
            return
        # Girar no lugar: velocidade de marcha zero, só diferença entre os lados.
        turn = math.copysign(self.get_parameter('align_turn').value, -error)
        self.drive(0.0, turn)
        if self.elapsed() > 8.0:
            self.get_logger().warn(
                f'não consegui alinhar em 8 s (erro de {math.degrees(error):+.1f}°); '
                'subindo assim mesmo')
            self.best_z = self.pose[2]
            self.progress_at = self.clock
            self.climb_started_z = self.pose[2]
            self.enter(SOBE)

    def do_sobe(self):
        """Abre a garra, sobe, e vigia o progresso em altura."""
        if self.transforming():
            self.drive(0.0)
            return
        self.drive(self.get_parameter('climb_speed').value)

        # Progresso: ganhou altura desde a última vez que olhei?
        if self.pose[2] > self.best_z + self.get_parameter('progress_height').value:
            self.best_z = self.pose[2]
            self.progress_at = self.clock
            self.retries = 0

        # Chegou ao plano? Pitch baixo, sustentado, e sem degrau à frente.
        if (abs(self.pitch()) < self.get_parameter('flat_pitch').value
                and not self.step_ahead()
                and self.pose[2] > (self.climb_started_z or 0.0) + 0.02):
            self.get_logger().info(
                f'topo: subiu {(self.pose[2] - self.climb_started_z) * 100:.1f} cm, '
                f'pitch de volta a {math.degrees(self.pitch()):+.1f}°')
            self.enter(TOPO)
            return

        # Empacou? Sem ganhar altura dentro do prazo, recua e tenta de novo.
        if (self.clock - self.progress_at
                > self.get_parameter('stall_time').value):
            self.retries += 1
            if self.retries > self.get_parameter('max_retries').value:
                self.get_logger().error(
                    f'empacado a {self.pose[2] * 100:.1f} cm depois de '
                    f'{self.retries - 1} tentativas. Um wheg de '
                    f'{geometry.WHEG_RADIUS * 1000:.0f} mm vence degraus de '
                    f'até ~{geometry.max_riser() * 1000:.0f} mm '
                    f'({geometry.CLIMB_RATIO:.1f} raios, medido em '
                    'scene/characterize.py): este pode estar fora do alcance.')
                self.finish(False)
                return
            self.get_logger().warn(
                f'sem ganhar altura há '
                f'{self.get_parameter("stall_time").value:.0f} s: recuando para '
                f'tentar de novo ({self.retries}/'
                f'{self.get_parameter("max_retries").value})')
            self.enter(RECUA)

    def do_recua(self):
        """Recua um pouco e volta a atacar o degrau.

        Recuar funciona porque o insucesso quase nunca é "não dá": é a ponta de
        um raio tendo parado encostada na face do espelho em vez de acima da
        quina. Afastar e voltar entrega o degrau a outra fase da marcha.
        """
        self.drive(-self.get_parameter('back_speed').value)
        if self.elapsed() > self.get_parameter('back_time').value:
            self.progress_at = self.clock
            self.enter(SOBE)

    def do_topo(self):
        """Fecha a roda e anda sobre o patamar, parando antes da beirada."""
        if self.transforming():
            self.drive(0.0)
            return
        self.drive(self.get_parameter('search_speed').value)
        if self.void_ahead():
            self.get_logger().info(
                f'beirada à frente ({self.median(self.edge) * 1000:.0f} mm, '
                f'alcance {self.probe_range * 1000:.0f} mm): parando')
            self.finish(True)
            return
        if self.step_ahead():
            # Outro lance de escada: volta a subir.
            self.get_logger().info('mais degraus à frente')
            self.enter(ALINHA)

    # =========================================================================
    #  Saídas
    # =========================================================================

    def drive(self, speed, turn=0.0):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(turn)
        self.drive_pub.publish(msg)

    def finish(self, success):
        self.drive(0.0)
        self.state = FIM
        height = (self.pose[2] - (self.climb_started_z or self.pose[2]))
        self.get_logger().info(
            ('SUCESSO' if success else 'FALHA')
            + f': subiu {height * 100:.1f} cm, parou em x={self.pose[0]:+.2f} m, '
              f'z={self.pose[2]:+.2f} m')
        self.status_pub.publish(String(data=FIM))
        if self.get_parameter('stop_when_done').value:
            self.done_pub.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = Climber()
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
