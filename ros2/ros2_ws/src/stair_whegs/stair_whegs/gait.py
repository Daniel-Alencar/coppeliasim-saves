"""Gerador de marcha: transforma um comando de velocidade em fases de wheg.

    drive (Twist)      --->  |      |  ---> wheg_joints  Float64MultiArray
    gait  (String)     --->  | gait |  ---> gait_state   std_msgs/String
    sim_time (Float64) --->  |      |

Este nó é o equivalente, para um robô de pernas, do que a cinemática inversa
das rodas era para os robôs anteriores: a camada que converte "quero andar
para a frente a tanto" no que cada atuador tem de fazer. A diferença é que,
com pernas, o comando não é só uma velocidade — é uma velocidade MAIS uma
coordenação de fases.

    θ_i(t) = φ_i + (ω ± Δ/2)·t

`φ_i` é a fase da marcha (geometry.GAITS), `ω` é a velocidade de marcha e `Δ` a
diferença entre os lados, que faz o rover virar por derrapagem — um wheg não
esterça nem rola de lado, então derrapar é a única forma de curvar.

POSIÇÃO, E NÃO VELOCIDADE
-------------------------
As juntas são comandadas em POSIÇÃO, com o alvo andando no tempo, e não em
velocidade. Parece um rodeio — mandar velocidade seria mais direto — mas foi
medido nesta cena:

    controle de velocidade   as fases DERIVAM: ao fim de 14 s de escalada, as
                             diagonais estavam 8° fora, e crescendo
    controle de posição      deriva zero, e o rover sobe os mesmos seis
                             degraus cerca de um segundo mais cedo

A razão é que cada wheg encontra uma carga diferente — um está enganchando a
quina enquanto o outro gira no ar — e, em malha de velocidade, quem está sob
carga fica para trás e não recupera o atraso. Em posição, o erro é corrigido.
Fase coordenada é o ponto inteiro de uma marcha; deixá-la derivar é desfazer a
marcha aos poucos.

O relógio é o DA SIMULAÇÃO
--------------------------
`θ(t)` precisa de um `t`, e ele vem do tópico `sim_time`, publicado pela ponte
— não do relógio do computador. Com a simulação a 0,3x do tempo real, um `t`
de parede faria o alvo das juntas correr três vezes mais rápido que o mundo, e
as juntas viveriam saturadas, perseguindo um alvo que foge.
"""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String

from stair_whegs import geometry
from stair_whegs.geometry import WHEGS


class Gait(Node):
    """Gera as posições das quatro juntas a partir de um comando de marcha."""

    def __init__(self):
        super().__init__('gait')

        self.declare_parameter('gait', geometry.DEFAULT_GAIT)
        self.declare_parameter('spokes', geometry.SPOKES)
        # Teto de velocidade dos whegs. Mais rápido não sobe melhor: o corpo
        # salta e perde contato justo quando precisaria de apoio.
        self.declare_parameter('max_speed', 6.0)          # [rad/s]
        self.declare_parameter('cmd_timeout', 1.0)        # [s de simulação]

        self.gait = self.get_parameter('gait').value
        self.spokes = self.get_parameter('spokes').value
        self.speed = 0.0
        self.turn = 0.0
        self.clock = None            # relógio da simulação
        self.started = None          # quando a marcha atual começou
        self.last_cmd = None         # quando chegou o último comando
        self.offset = {name: 0.0 for name in WHEGS}
        self.position = {}           # última posição medida de cada junta

        self.create_subscription(Twist, 'drive', self.drive_callback, 10)
        self.create_subscription(String, 'gait', self.gait_callback, 10)
        self.create_subscription(Float64, 'sim_time', self.clock_callback, 10)
        self.create_subscription(JointState, 'wheg_state', self.state_callback, 10)
        self.joints_pub = self.create_publisher(Float64MultiArray,
                                                'wheg_joints', 10)
        self.state_pub = self.create_publisher(String, 'gait_state', 10)

        self.get_logger().info(
            f'marcha pronta: "{self.gait}" com {self.spokes} raios '
            f'(passo de {math.degrees(geometry.step_angle(self.spokes)):.0f}°, '
            f'maior vão sem apoio {geometry.longest_gap(self.gait):.0%} do passo)')

    # =========================================================================
    #  Entradas
    # =========================================================================

    def drive_callback(self, msg: Twist):
        """Comando de marcha.

        linear.x   velocidade dos whegs [rad/s]; POSITIVO anda para a frente
        angular.z  diferença entre os lados [rad/s]; positivo vira à esquerda

        O sinal de `linear.x` é invertido aqui, uma vez só: as juntas estão
        montadas na cena de modo que girar no sentido negativo leva o rover
        para a frente. Quem comanda pensa em "para a frente"; a convenção da
        cena fica contida neste nó.
        """
        limit = self.get_parameter('max_speed').value
        self.speed = -max(-limit, min(limit, msg.linear.x))
        self.turn = -max(-limit, min(limit, msg.angular.z))
        self.last_cmd = self.clock

    def gait_callback(self, msg: String):
        """Troca de marcha, sem salto nas juntas."""
        name = msg.data.strip()
        if name == self.gait:
            return
        if name not in geometry.GAITS:
            self.get_logger().warn(
                f'marcha "{name}" não existe. conhecidas: '
                f'{", ".join(sorted(geometry.GAITS))}')
            return
        # Guarda a fase onde os whegs ESTÃO, para a marcha nova começar daí.
        # Sem isso, as juntas correriam para a fase nova na velocidade máxima
        # do motor e o rover levaria um tranco — justo o tipo de movimento que
        # faz perder o apoio no meio de um degrau.
        if self.position and self.elapsed() is not None:
            self.offset = geometry.rephase(
                self.position, self.elapsed(), self.speed, self.turn,
                name, self.spokes)
        self.gait = name
        self.get_logger().info(f'marcha -> {name}')

    def clock_callback(self, msg: Float64):
        if self.started is None:
            self.started = msg.data
        self.clock = msg.data
        self.tick()

    def state_callback(self, msg: JointState):
        self.position = dict(zip(msg.name, msg.position))

    # =========================================================================
    #  Geração
    # =========================================================================

    def elapsed(self):
        if self.clock is None or self.started is None:
            return None
        return self.clock - self.started

    def tick(self):
        """Um ciclo: publica a posição alvo de cada wheg para este instante."""
        time = self.elapsed()
        if time is None:
            return

        # Sem comando recente, para de andar. O alvo deixa de avançar e as
        # juntas ficam onde estão — num robô de pernas, voltar as juntas à
        # origem faria o corpo desabar.
        if (self.last_cmd is not None
                and self.clock - self.last_cmd
                > self.get_parameter('cmd_timeout').value):
            self.speed = 0.0
            self.turn = 0.0

        targets = geometry.gait_targets(time, self.speed, self.turn,
                                        self.gait, self.spokes, self.offset)
        self.joints_pub.publish(
            Float64MultiArray(data=[targets[name] for name in WHEGS]))
        self.state_pub.publish(String(data=self.gait))


def main(args=None):
    rclpy.init(args=args)
    node = Gait()
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
