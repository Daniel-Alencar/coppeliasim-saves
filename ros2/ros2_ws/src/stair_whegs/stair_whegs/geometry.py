"""A geometria do rover de whegs e a matemática da marcha.

Python puro: não conhece ROS nem CoppeliaSim, e por isso dá para testar a
marcha inteira sem abrir o simulador (ver test/).

O que é um wheg
---------------
Uma roda de raios — *wheel* + *legs*. Em vez do aro contínuo, `SPOKES` raios
curvos saindo do cubo. No plano, ela rola como uma roda poligonal; contra um
degrau, cada raio funciona como uma perna que ENGANCHA na quina e puxa o corpo
para cima. É o que uma roda lisa não faz: a roda precisa de atrito na face
vertical do espelho, que não existe, enquanto o raio se apoia na QUINA.

A regra prática que sai disso — e que os ensaios deste projeto CORRIGIRAM para
cima:

    altura de degrau vencível  ≈  1,3 x raio do wheg

A estimativa de livro é um raio: a ponta do raio tem de alcançar acima da
quina. Medido em `scene/characterize.py`, este rover sobe degraus de 65 mm com
whegs de 50 mm e falha nos de 68 mm. O que compra os 30 % a mais é a CURVATURA
do raio — a garra alcança a quina de um ponto em que um raio reto ainda estaria
batendo na face do espelho.

Com três raios a 120°, o corpo sobe e desce `R(1 - cos 60°) = R/2` a cada
terço de volta — é o balanço característico da marcha, e é o preço de não ter
aro. Mais raios suavizam o andar e pioram a escalada (o vão entre raios fica
curto demais para abraçar a quina); menos raios fazem o contrário.

A marcha
--------
Com quatro whegs, o que define a marcha é a FASE de cada um:

    θ_i(t) = φ_i + ω·t

Os pares DIAGONAIS andam juntos (φ_fl = φ_rr e φ_fr = φ_rl), e as duas
diagonais ficam defasadas de meio passo — 60° para três raios, que é metade dos
120° entre raios. Assim, enquanto uma diagonal está no meio do apoio, a outra
está entrando em contato: nunca há instante sem pelo menos dois pontos de
apoio em diagonal, que é o mínimo para o corpo não cair de nariz.

É a marcha do Mini-Whegs, e é o análogo com quatro pernas do tripé alternado
dos insetos.
"""

import math

# =============================================================================
#  O rover
# =============================================================================

#: Raio do wheg, do centro à ponta do raio [m].
WHEG_RADIUS = 0.050
#: Quantos raios cada wheg tem.
SPOKES = 3
#: Quanto o raio se curva para trás, do cubo à ponta [rad]. Zero daria um raio
#: reto; a curvatura faz a ponta "varrer" a quina do degrau em vez de bater
#: nela de frente.
SPOKE_CURVE = 0.55

#: Distância entre os eixos dianteiro e traseiro [m].
WHEELBASE = 0.150
#: Distância entre os whegs esquerdo e direito [m].
TRACK = 0.150

#: Comprimento da cauda [m] e sua inclinação abaixo da horizontal [rad].
#:
#: A cauda é o que impede o capotamento para trás. Sem ela, medido nos ensaios,
#: o rover subia três degraus e tombava de costas: ao enganchar o degrau, a
#: reação empurra o nariz para cima, e com o centro de massa atrás do ponto de
#: apoio não há o que o segure. A cauda devolve um terceiro ponto de apoio lá
#: atrás, longe do eixo, e o momento se resolve.
TAIL_LENGTH = 0.090
TAIL_ANGLE = math.radians(-20.0)

#: Massas [kg], da cena.
CHASSIS_MASS = 0.80
WHEG_MASS = 0.06
TAIL_MASS = 0.05

#: Torque máximo de cada motor de wheg [N.m].
WHEG_TORQUE = 2.0
#: Velocidade angular de marcha usada por padrão [rad/s]. Negativa é para a
#: frente, pelo jeito como as juntas ficam montadas na cena.
CRUISE = 3.0


# =============================================================================
#  A escada
# =============================================================================

#: Degrau padrão da cena: espelho (altura) e piso (profundidade), em metros.
RISER = 0.040
TREAD = 0.120
STEPS = 6
STAIR_WIDTH = 0.70
#: Onde começa o primeiro espelho, e o comprimento do patamar no topo [m].
STAIR_X0 = 0.40
LANDING = 0.60


def stair_height(x, riser=RISER, tread=TREAD, steps=STEPS, x0=STAIR_X0):
    """Altura do piso da escada em x [m]. Zero antes dela, topo depois."""
    if x < x0:
        return 0.0
    index = int((x - x0) // tread) + 1
    return min(index, steps) * riser


# =============================================================================
#  A marcha
# =============================================================================

#: Nomes dos quatro whegs, na ordem usada em todo o pacote.
WHEGS = ('fl', 'fr', 'rl', 'rr')

#: Padrões de marcha: fase de cada wheg, em frações de UM PASSO.
#:
#: Um passo é o ângulo entre dois raios (2π/SPOKES = 120° para três raios), e
#: não uma volta inteira: é ele que se repete no contato com o chão. Dar as
#: fases em frações do passo deixa os padrões independentes do número de raios.
#:
#: - `alternada`  as diagonais em oposição (meio passo de diferença). É a
#:                marcha de marcha e de escalada: sempre dois apoios em
#:                diagonal, nunca um balanço só.
#: - `junta`      os quatro em fase. Os quatro raios batem no chão ao mesmo
#:                tempo: o rover salta em vez de andar, e o maior vão sem apoio
#:                é o passo inteiro. Serve de comparação — e o que o ensaio diz
#:                dela é menos óbvio do que parece (ver o README).
#: - `dianteira`  frente e trás em oposição, esquerda e direita em fase. O
#:                corpo galopa; útil para mostrar o efeito da fase no balanço.
GAITS = {
    'alternada': {'fl': 0.0, 'fr': 0.5, 'rl': 0.5, 'rr': 0.0},
    'junta': {'fl': 0.0, 'fr': 0.0, 'rl': 0.0, 'rr': 0.0},
    'dianteira': {'fl': 0.0, 'fr': 0.0, 'rl': 0.5, 'rr': 0.5},
}

#: Marcha usada por padrão.
DEFAULT_GAIT = 'alternada'


def step_angle(spokes=SPOKES):
    """Ângulo entre dois raios [rad]: o período real da marcha."""
    return 2.0 * math.pi / spokes


def phases(gait=DEFAULT_GAIT, spokes=SPOKES):
    """Fase inicial de cada wheg [rad], a partir do nome da marcha."""
    if gait not in GAITS:
        raise KeyError(
            f'marcha "{gait}" não existe. conhecidas: {", ".join(sorted(GAITS))}')
    step = step_angle(spokes)
    return {name: fraction * step for name, fraction in GAITS[gait].items()}


def gait_targets(time, speed, turn=0.0, gait=DEFAULT_GAIT, spokes=SPOKES,
                 offset=None):
    """Posição alvo de cada wheg [rad] no instante `time`.

    time    tempo decorrido de marcha [s] — o relógio da SIMULAÇÃO
    speed   velocidade angular dos whegs [rad/s]; negativa anda para a frente
    turn    diferença entre os lados [rad/s]: positivo gira para a esquerda
    offset  fase acumulada de cada wheg, para trocar de marcha sem salto

    É a geração de marcha inteira, e é só isto:

        θ_i(t) = φ_i + (ω ± Δ/2)·t

    O giro é por DERRAPAGEM, como num tanque: o lado de dentro da curva anda
    mais devagar que o de fora. Um wheg não tem como esterçar — e, ao
    contrário da roda, ele nem sequer rola de lado, então derrapar é a única
    forma de virar.

    `offset` existe porque trocar de marcha no meio do movimento mudaria φ_i
    de repente, e o wheg saltaria para a nova posição. Quem troca de marcha
    guarda a fase onde estava e a passa aqui (ver `rephase`).
    """
    base = phases(gait, spokes)
    offset = offset or {name: 0.0 for name in WHEGS}
    targets = {}
    for name in WHEGS:
        side = +1.0 if name in ('fl', 'rl') else -1.0
        # Lado esquerdo mais devagar => o rover vira para a esquerda.
        rate = speed + side * turn / 2.0
        targets[name] = base[name] + offset[name] + rate * time
    return targets


def rephase(current, time, speed, turn=0.0, gait=DEFAULT_GAIT, spokes=SPOKES):
    """Offsets que fazem a nova marcha COMEÇAR onde os whegs já estão.

    Sem isto, trocar de marcha dá um salto: as juntas em controle de posição
    correriam para a fase nova na velocidade máxima que o motor permite, e o
    rover daria um tranco — exatamente o tipo de movimento brusco que faz
    perder o apoio no meio de um degrau.
    """
    wanted = gait_targets(time, speed, turn, gait, spokes)
    return {name: current.get(name, 0.0) - wanted[name] for name in WHEGS}


def longest_gap(gait=DEFAULT_GAIT):
    """Maior intervalo do passo sem nenhum raio entrando em contato, de 0 a 1.

    Mede, por geometria pura, o que a marcha alternada compra. Se as fases dos
    quatro whegs estão espalhadas pelo passo, há sempre uma perna chegando ao
    chão; se estão todas juntas, existe um trecho longo em que nenhuma chega —
    e nesse trecho o corpo está caindo.

        alternada   fases {0, ½}       -> maior vão = 0,5
        dianteira   fases {0, ½}       -> maior vão = 0,5
        junta       fases {0}          -> maior vão = 1,0  (o passo inteiro)

    Quanto menor, mais suave. Não substitui o ensaio — ele é que diz se sobe —
    mas explica por que `junta` salta em vez de andar.
    """
    fractions = sorted(set(GAITS[gait].values()))
    if len(fractions) == 1:
        return 1.0
    gaps = [fractions[index + 1] - fractions[index]
            for index in range(len(fractions) - 1)]
    gaps.append(1.0 - fractions[-1] + fractions[0])
    return max(gaps)


# =============================================================================
#  Limites
# =============================================================================

#: Razão entre o maior degrau vencido e o raio do wheg, MEDIDA.
#:
#: `scene/characterize.py espelho`: com whegs de 50 mm o rover sobe degraus de
#: 65 mm e falha nos de 68 mm. A estimativa geométrica simples — a ponta tem de
#: alcançar a quina, logo o degrau vale um raio — subestima, porque o raio é
#: CURVO: a garra alcança a quina de um ponto em que um raio reto ainda bateria
#: na face.
CLIMB_RATIO = 1.3


def max_riser(radius=WHEG_RADIUS):
    """Maior espelho que o rover vence, em metros.

    Vem da razão MEDIDA (CLIMB_RATIO), e não de um argumento geométrico. Serve
    para o escalador avisar, quando empaca, que o degrau pode simplesmente
    estar fora do alcance deste wheg.
    """
    return CLIMB_RATIO * radius


def body_bounce(radius=WHEG_RADIUS, spokes=SPOKES):
    """Sobe-e-desce do corpo a cada passo, em metros.

    Entre um raio e o seguinte, o centro do wheg passa do apoio na ponta
    (altura R) ao apoio na "quina" entre dois raios (altura R·cos(π/n)). A
    diferença é o balanço, e é o custo de não ter aro.
    """
    return radius * (1.0 - math.cos(math.pi / spokes))
