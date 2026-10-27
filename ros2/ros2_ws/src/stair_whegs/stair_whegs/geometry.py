"""A geometria do rover de whegs e a matemática da marcha.

Python puro: não conhece ROS nem CoppeliaSim, e por isso dá para testar a
marcha inteira sem abrir o simulador (ver test/).

O que é um wheg, e o que é uma RODA TRANSFORMÁVEL
-------------------------------------------------
Um wheg comum — *wheel* + *legs* — troca o aro por raios fixos: anda mal e sobe
bem, e não há como escolher. A roda deste rover não é essa. Ela é a roda
transformável do **Wheel Transformer** (Kim, Jung, Kim, Cho e Chu, IEEE T-RO
2014; patente US 9 073 587), e tem DOIS estados:

    FECHADA ("roda")   as `SPOKES` pernas encostam umas nas outras e os seus
                       dorsos curvos completam um ARO de raio `WHEG_RADIUS`.
                       É uma roda de verdade: rola sem balanço nenhum.

    ABERTA ("garra")   cada perna gira `LEG_OPEN` em torno do seu eixo, que
                       fica SOBRE o aro, a `LEG_PIVOT` do raio. As pernas
                       abrem em catavento e a ponta passa a alcançar
                       `claw_reach()` ≈ 2 raios do centro.

O ganho é que não se escolhe mais entre andar e subir: a roda fechada rola
rápido e lisa no plano, e a aberta engancha a quina do degrau como perna. É a
diferença entre as duas colunas desta conta:

                            fechada          aberta
    alcance do contato      1,00 R           1,96 R    (`claw_reach`)
    balanço do corpo/passo  0 mm             R(1-cos 60°) = R/2

NO ROBÔ ORIGINAL A TROCA É PASSIVA
----------------------------------
No Wheel Transformer não existe motor de transformação. Uma das três pernas é
a PERNA GATILHO: quando a ponta dela esfrega num obstáculo, o atrito a faz
girar, e um disco transmissor acoplado leva as outras duas junto — as três
abrem em bloco. Uma mola de torção fecha tudo de volta quando o atrito some.

Aqui a transformação é COMANDADA, e a diferença é honesta: o acoplamento
mecânico das três pernas pelo disco vira um acoplamento de software (as três
juntas de uma roda recebem sempre o mesmo alvo), e o gatilho de atrito vira o
gatilho de PERCEPÇÃO do `climber` — ele já sabe, pelas sondas, quando há
degrau à frente. Simular o disco e o atrito do gatilho daria um mecanismo
instável no motor de física e não acrescentaria nada ao que o trabalho mede.
O limite de junta em zero é que continua sendo mecânico: é o batente contra o
qual a perna fechada apoia o peso do rover, como no original.

O balanço do corpo
------------------
Com três pernas ABERTAS a 120°, o corpo sobe e desce `R(1 - cos 60°) = R/2` a
cada terço de volta — o balanço característico da marcha, e o preço de abrir o
aro. FECHADA não há balanço algum, e é por isso que a calibração das sondas e
a busca pela escada acontecem com a roda fechada.

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
#: Quantas pernas cada roda tem. Fechadas, os dorsos delas formam o aro: cada
#: uma cobre 360/SPOKES graus de circunferência. Abertas, são os raios da
#: marcha, e `step_angle` continua sendo o ângulo entre duas.
SPOKES = 3

#: Distância entre os eixos dianteiro e traseiro [m].
#:
#: 200 mm, e não os 150 mm da versão de raios fixos. Quem mandou alongar foi a
#: garra: o que empina o rover ao atacar o degrau é a perna agarrando a quina
#: ACIMA do eixo, e o que resiste é o entre-eixos. A razão
#: `claw_reach() / WHEELBASE` era 0,33 com o wheg fixo de 50 mm; a garra de
#: 98 mm contra 150 mm de base a levou a 0,65, e o rover passou a empinar até
#: 90° e cair. Medido em `scene/characterize.py`: com a razão de volta a ~0,32
#: o pitch máximo cai de 90° para 27° e o rover sobe a escada inteira.
WHEELBASE = 0.200
#: Distância entre as rodas esquerda e direita [m].
#:
#: 180 mm. A garra aberta levanta o centro de massa e o ângulo de tombamento
#: lateral — `atan(meia-bitola / altura)` — cai junto. Com 150 mm de bitola e
#: a garra padrão ele vale 51°; com 180 mm, 54°. A margem importa porque a
#: marcha `alternada` põe os dois lados em fases opostas e balança o corpo em
#: rolagem o tempo todo: medido em piso plano, ±14°.
TRACK = 0.180

#: Comprimento da cauda [m] e sua inclinação abaixo da horizontal [rad].
#:
#: A cauda é o que impede o capotamento para trás. Sem ela, medido nos ensaios,
#: o rover subia três degraus e tombava de costas: ao enganchar o degrau, a
#: reação empurra o nariz para cima, e com o centro de massa atrás do ponto de
#: apoio não há o que o segure. A cauda devolve um terceiro ponto de apoio lá
#: atrás, longe do eixo, e o momento se resolve.
TAIL_LENGTH = 0.120
TAIL_ANGLE = math.radians(-20.0)

#: Massas [kg], da cena.
#:
#: A roda pesa o dobro do wheg maciço que ela substituiu, e é de propósito.
#: O que ela carrega agora é um mecanismo — quadro, três eixos, três pernas —
#: e, no simulador, massa de roda pequena demais tem um custo concreto: o
#: contato com o chão passa por UMA perna, e com 60 g de roda essa perna
#: pesava 12 g contra um corpo de 850 g. Motor de física nenhum trata bem uma
#: razão de 1:70 no ponto de contato. Com 120 g por roda a perna vai a 24 g e
#: a razão cai para 1:35.
CHASSIS_MASS = 0.80
WHEG_MASS = 0.12
TAIL_MASS = 0.05

#: Torque máximo de cada motor de wheg [N.m].
WHEG_TORQUE = 2.0
#: Velocidade angular de marcha usada por padrão [rad/s]. Negativa é para a
#: frente, pelo jeito como as juntas ficam montadas na cena.
CRUISE = 3.0


# =============================================================================
#  A roda transformável
# =============================================================================

#: Onde fica o eixo de cada perna, em frações do raio da roda.
#:
#: Ele fica SOBRE o aro, e não no cubo — é o que distingue esta roda de um
#: wheg comum. Fechada, o dorso da perna é um arco de raio `LEG_PIVOT x R`
#: que começa no próprio eixo e vai até o eixo da perna seguinte: os três
#: arcos emendam e fecham a circunferência. O valor é o raio da LINHA MÉDIA da
#: casca, isto é, o raio externo menos meia espessura.
LEG_PIVOT = 0.86

#: Para que lado o dorso da perna sai do eixo dela: -1 ou +1.
#:
#: É o que decide se a PONTA da perna chega ao chão antes ou depois do eixo
#: dela, e isso muda tudo contra um degrau. Rodando para a frente, a roda gira
#: no sentido horário no plano x-z:
#:
#:     -1  o dorso sai no sentido horário  ->  a ponta chega ANTES: ela varre
#:         a quina e engancha, e a rotação puxa o corpo por cima
#:     +1  o dorso sai no anti-horário     ->  a ponta chega DEPOIS: quem bate
#:         na face do espelho é o lombo da perna, que empurra o rover para
#:         trás e o empina
#:
#: Medido: com +1 o rover empina 72° no primeiro degrau e não sobe. É a mesma
#: razão pela qual os raios do wheg clássico são curvados para trás.
LEG_RAKE = -1.0

#: Quanto cada perna gira para abrir [rad].
#:
#: `claw_reach` é monótona: cada grau a mais alcança mais longe, e pelo
#: alcance a perna deveria abrir o máximo possível. O ensaio diz o contrário,
#: e a razão é que o alcance da garra não é só alcance — é também a ALTURA EM
#: QUE O ROVER ANDA quando está aberto, e três coisas pioram com ela de uma
#: vez:
#:
#:     - o que empina o rover na quina do degrau cresce, contra um
#:       entre-eixos que não muda (ver `WHEELBASE`);
#:     - o ângulo de tombamento lateral cai (ver `TRACK`);
#:     - a cauda, que é rígida, passa a pegar o chão tarde demais
#:       (ver `tail_catch_angle`).
#:
#: Com 80° de abertura — 1,96 raios de alcance — o rover empina até 90° no
#: primeiro degrau e capota, e NENHUMA bitola o salva: 150, 180 e 220 mm
#: capotam igual, nas duas marchas. Com 30° — 1,30 raio, que ainda é 1,6 vez o
#: degrau da cena — ele sobe os seis degraus com pitch máximo de 27°.
#:
#: O número, então, não sai de maximizar o alcance. Sai de manter o rover
#: aberto dentro da mesma faixa de proporções em que o wheg de raios fixos já
#: funcionava.
LEG_OPEN = math.radians(30.0)

#: Nomes dos dois estados da roda, como viajam no tópico `wheel_mode`.
RODA = 'roda'
GARRA = 'garra'

#: Ângulo de perna de cada estado [rad]. Fechada é zero PORQUE zero é o
#: batente: a perna fechada descansa contra o limite da junta, e o peso do
#: rover vai para o batente e não para o motor — como no mecanismo original.
#:
#: Aberta tem o sinal CONTRÁRIO ao de `LEG_RAKE`, que é o único que afasta a
#: ponta do centro (ver `claw_tip`); girar para o outro lado faria a roda
#: encolher. A junta é montada com os dois estados como os seus dois
#: batentes.
WHEEL_MODES = {RODA: 0.0, GARRA: -LEG_RAKE * LEG_OPEN}

#: Torque máximo de cada junta de perna [N.m].
#:
#: Alto de propósito, e não é um motorzinho exagerado: na roda de verdade a
#: perna ABERTA não é segurada por motor nenhum. Ela trava contra o fim do
#: rasgo do disco transmissor, e a carga vai para o quadro da roda. Aqui não
#: há disco, então quem faz esse papel é o torque da junta — ele é a RIGIDEZ
#: do mecanismo, não a força de um atuador.
#:
#: O piso é o rover inteiro apoiado na ponta de UMA perna aberta:
#:
#:     peso x braço  =  13 N x 0,064 m  ~  0,8 N.m
#:
#: e o resto é margem para o impacto de a garra bater na quina, que é
#: justamente o instante em que a perna não pode ceder. Com a roda FECHADA
#: este torque não faz diferença nenhuma, porque ali quem segura é o batente
#: (ver `WHEEL_MODES`).
LEG_TORQUE = 4.0


def leg_span(spokes=SPOKES):
    """Quanto de circunferência o dorso de uma perna cobre [rad].

    É o que faz a roda FECHAR: com `spokes` pernas iguais, cada dorso tem de
    cobrir exatamente a fatia que falta para a volta inteira.
    """
    return 2.0 * math.pi / spokes


def claw_tip(radius=WHEG_RADIUS, open_angle=LEG_OPEN, spokes=SPOKES,
             pivot=LEG_PIVOT):
    """Onde a ponta de uma perna aberta vai parar, no plano da roda [m, m].

    Origem no centro da roda, perna cujo eixo está em ângulo zero. A ponta
    nasce na outra extremidade do arco — a `leg_span` adiante — e gira em
    torno do eixo da perna, que NÃO é o centro da roda. Daí o alcance crescer
    muito mais rápido que o ângulo de abertura.

    O sentido da abertura é o que AFASTA a ponta do centro, e sai de
    `LEG_RAKE`: é sempre o contrário do lado para o qual o dorso se estende.
    Isso não é convenção — girando para o outro lado a ponta passa por dentro
    do aro, a roda encolhe, e o mecanismo não serviria para nada.
    """
    px, pz = pivot * radius, 0.0
    tip = LEG_RAKE * leg_span(spokes)
    ex, ez = pivot * radius * math.cos(tip), pivot * radius * math.sin(tip)
    dx, dz = ex - px, ez - pz
    turn = -LEG_RAKE * open_angle
    cos, sin = math.cos(turn), math.sin(turn)
    return px + dx * cos - dz * sin, pz + dx * sin + dz * cos


def claw_reach(radius=WHEG_RADIUS, open_angle=LEG_OPEN, spokes=SPOKES,
               pivot=LEG_PIVOT):
    """Raio efetivo da roda aberta [m]: a distância do centro à ponta.

    É o número que diz o que a transformação compra. Com os valores padrão dá
    1,96 x `WHEG_RADIUS` — a roda fechada alcança um raio, a aberta alcança
    quase dois, e é sobre o SEGUNDO que vale a regra do degrau vencível.

    O valor é medido na LINHA MÉDIA da casca da perna; a superfície externa
    fica meia espessura mais longe, e é ela que toca a quina.
    """
    return math.hypot(*claw_tip(radius, open_angle, spokes, pivot))


def wheel_angle(mode):
    """Ângulo de perna [rad] a partir do nome do estado."""
    if mode not in WHEEL_MODES:
        raise KeyError(
            f'estado de roda "{mode}" não existe. conhecidos: '
            f'{", ".join(sorted(WHEEL_MODES))}')
    return WHEEL_MODES[mode]


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

#: Marcha usada por padrão: a de ANDAR.
DEFAULT_GAIT = 'alternada'

#: Marcha usada para SUBIR, que não é a mesma — e a razão medida não é a que
#: a intuição de marcha sugere.
#:
#: `alternada` põe os dois lados em fases opostas, e com a garra ABERTA isso
#: balança o corpo em ROLAGEM: medido em piso plano, ±14°. No plano não faz
#: diferença; na quina do degrau, onde o rover empina e a margem lateral já
#: está consumida, esses ±14° são a diferença entre subir e deitar. Medido em
#: `scene/characterize.py`, mesmo rover, só trocando a marcha:
#:
#:     alternada   rolagem máxima 180°  ->  CAPOTOU
#:     junta       rolagem máxima   1°  ->  subiu os seis degraus
#:
#: Com os quatro em fase os dois lados sobem e descem JUNTOS: o balanço vai
#: todo para o pitch, que a cauda e o entre-eixos aguentam, e some da rolagem,
#: que não tem quem aguente. É por isso que trocar de marcha em movimento, sem
#: salto nas juntas (`rephase`), é função do nó `gait` e não um detalhe.
CLIMB_GAIT = 'junta'


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

#: Razão entre o maior degrau vencido e o ALCANCE do contato, MEDIDA.
#:
#: O denominador é `claw_reach()`, e não o raio da roda. Com a roda
#: transformável as duas coisas deixaram de ser a mesma: fechada, o contato
#: está a um raio do centro; aberta, a 1,30 raios. Quem vence o degrau difícil
#: é a perna ABERTA, então é o alcance dela que entra na conta.
#:
#: O valor vem de `scene/characterize.py roda`, que monta a cena de novo para
#: cada altura de espelho e observa. Medido com a garra padrão de 65 mm e a
#: marcha de escalada:
#:
#:     70 mm (1,08 garra)  sobe os seis degraus
#:     75 mm (1,16 garra)  CAPOTA
#:
#: e o número ficou com o maior que sobe. Repare que ele é bem menor que o 1,3
#: do wheg de raios fixos contra o próprio raio, e não por a garra ser pior: o
#: denominador aqui é o ALCANCE da garra, que já é 1,30 vez o aro. Em degrau
#: absoluto a roda transformável vai mais longe — 70 mm contra os 65 mm do
#: wheg fixo, com um aro do mesmo tamanho.
#:
#: MEDIR DE NOVO depois de mexer na geometria da perna: a razão não é uma
#: constante da natureza, e muda com `LEG_OPEN`, `LEG_PIVOT`, o entre-eixos e
#: a cauda.
CLIMB_RATIO = 1.08


def max_riser(radius=WHEG_RADIUS, open_angle=LEG_OPEN, spokes=SPOKES,
              pivot=LEG_PIVOT):
    """Maior espelho que o rover vence com a roda ABERTA, em metros.

    Vem da razão MEDIDA (CLIMB_RATIO) aplicada ao alcance da garra. Serve para
    o escalador avisar, quando empaca, que o degrau pode simplesmente estar
    fora do alcance desta roda.

    A roda FECHADA tem o seu próprio limite, e ele não é zero: um aro de raio
    R se apoia na QUINA do degrau como qualquer roda sobe um meio-fio, e com
    tração nas quatro rodas isso leva longe. Medido em
    `scene/characterize.py roda`, e foi uma surpresa — a primeira versão deste
    arquivo afirmava que roda lisa não sobe degrau nenhum, o que vale para uma
    roda só puxando contra a face vertical, e não para este rover.
    """
    return CLIMB_RATIO * claw_reach(radius, open_angle, spokes, pivot)


def tail_catch_angle(ride_height, tail=TAIL_LENGTH, angle=TAIL_ANGLE,
                     wheelbase=WHEELBASE, body_length=None):
    """Com quanto de pitch a cauda encosta no chão [rad].

    `ride_height` é a altura do eixo — e é AÍ que mora o problema que a roda
    transformável criou. A cauda é uma peça rígida, dimensionada para uma
    altura de eixo; o rover agora tem DUAS, e elas diferem por um fator de
    quase dois.

    A conta: a ponta da cauda fica `d` atrás do eixo traseiro e `h` abaixo
    dele. Empinando de `phi`, ela desce, e toca quando

        d sen(phi) + h cos(phi) = ride_height

    Quanto MENOR o ângulo devolvido, mais cedo a cauda entra e mais segura ela
    é. Com o eixo a 50 mm (roda fechada) a cauda padrão pega aos 10°: o rover
    mal começa a empinar e já tem apoio. Com o eixo a 98 mm (garra aberta) ela
    só pega aos 43°, e aos 43° o rover já tem energia demais — foi o que os
    ensaios de bitola registraram, com pitch máximo de 81° a 90° e o rover
    acabando deitado.

    Devolve `None` se a cauda não alcança o chão em pitch nenhum.
    """
    if tail <= 0.0:
        return None
    if body_length is None:
        body_length = wheelbase + 0.05
    # A cauda sai da traseira do chassi, que fica atrás do eixo traseiro.
    overhang = body_length / 2.0 - wheelbase / 2.0
    depth = overhang + tail * math.cos(angle)
    drop = tail * math.sin(abs(angle))
    amplitude = math.hypot(depth, drop)
    if amplitude < ride_height:
        return None
    phase = math.atan2(drop, depth)
    return math.asin(ride_height / amplitude) - phase


def body_bounce(radius=WHEG_RADIUS, spokes=SPOKES, mode=GARRA):
    """Sobe-e-desce do corpo a cada passo, em metros.

    Com a roda ABERTA, entre uma perna e a seguinte o centro passa do apoio na
    ponta (altura R) ao apoio no vão entre duas pontas (altura R·cos(π/n)). A
    diferença é o balanço, e é o custo de abrir o aro.

    FECHADA o balanço é ZERO, e não "pequeno": os dorsos das pernas formam uma
    circunferência de verdade. É a vantagem que o mecanismo transformável
    compra, e é por isso que a calibração das sondas e a busca pela escada
    acontecem com a roda fechada — nessa fase não há balanço contaminando a
    leitura.
    """
    if mode == RODA:
        return 0.0
    return radius * (1.0 - math.cos(math.pi / spokes))
