#!/usr/bin/env python3
"""Monta a cena do rover de whegs e da escada, e a salva em .ttt.

Não há modelo de rover de whegs no CoppeliaSim: este script o CONSTRÓI, peça
por peça, pela ZeroMQ Remote API. É a parte do trabalho que normalmente se
faria na interface gráfica, e fazê-la em código tem três consequências boas:
a cena é reproduzível a partir de uma semente de parâmetros, a geometria do
robô vira variável de experimento (`--wheg-radius`, `--spokes`, `--leg-open`,
`--tail`), e cada decisão de projeto fica escrita ao lado do código que a
executa.

    python3 scene/build_scene.py                      # rover e escada padrão
    python3 scene/build_scene.py --riser 0.05         # degrau mais alto
    python3 scene/build_scene.py --wheg-radius 0.065 --spokes 4
    python3 scene/build_scene.py --leg-open 60        # garra que abre menos
    python3 scene/build_scene.py --no-tail            # para ver o capotamento
    python3 scene/build_scene.py --no-save

O que a cena tem
----------------
    /rover/chassis            o corpo, com a cauda incorporada
    /rover/chassis/imu        dummy alinhado ao mundo: é por ele que a atitude
                              é medida (ver abaixo)
    /rover/chassis/<w>_joint  as quatro juntas de TRAÇÃO (fl, fr, rl, rr)
    .../<w>_joint/<w>_hub     o quadro em Y de cada roda transformável
    .../<w>_leg<k>_joint      as três juntas de PERNA de cada roda, k = 0..2.
                              Zero fecha a roda num aro; -LEG_OPEN a abre em
                              catavento (ver `add_wheel`)
    /rover/chassis/front_l    sensores de proximidade à frente, à altura do
    /rover/chassis/front_r    espelho: detectam a escada e o desalinhamento
    /rover/chassis/edge       sensor apontado para baixo e à frente: detecta a
                              beirada do patamar
    /stairs/step              os degraus
    /stairs/landing           o patamar no topo

POR QUE O IMU É UM DUMMY SEPARADO
---------------------------------
A atitude do rover não pode ser lida do frame do chassi. O chassi é montado
com `sim.groupShapes` (corpo + cauda), e o CoppeliaSim realinha o frame do
shape resultante com os eixos principais de inércia: com a cauda, o frame
nasce inclinado 20° em relação ao mundo. Ler `getObjectOrientation` dele daria
um rover permanentemente empinado.

O dummy `imu` é criado DEPOIS, alinhado ao mundo, e preso ao chassi. Ele é o
que um IMU de verdade seria: um referencial solidário ao corpo, montado
nivelado. A ponte lê a atitude dele.
"""

import argparse
import math
import os
import sys

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..', '..'))
sys.path.insert(0, os.path.join(HERE, '..'))

from stair_whegs import geometry  # noqa: E402  (precisa do sys.path acima)

COPPELIA = os.environ.get(
    'COPPELIASIM_ROOT',
    os.path.expanduser('~/Softwares/CoppeliaSim/CoppeliaSim_Edu'))
FLOOR_MODEL = os.path.join(COPPELIA,
                           'models/infrastructure/floors/5mX5m wooden floor.ttm')

SCENE_TARGETS = (
    os.path.join(REPO, 'projects/10 - stair_whegs/stair_whegs.ttt'),
    os.path.join(HERE, '..', 'coppeliasim', 'stair_whegs.ttt'),
)


# =============================================================================
#  Utilidades
# =============================================================================

def purge(sim, alias):
    """Apaga tudo que já existe com esse nome, modelo ou não."""
    handle = sim.getObject(alias, {'noError': True})
    while handle != -1:
        try:
            sim.removeModel(handle)
        except Exception:
            sim.removeObjects(sim.getObjectsInTree(handle, sim.handle_all))
        handle = sim.getObject(alias, {'noError': True})


def group(sim, name):
    """Dummy para agrupar objetos na hierarquia."""
    dummy = sim.createDummy(0.01)
    sim.setObjectAlias(dummy, name)
    sim.setObjectPosition(dummy, sim.handle_world, [0.0, 0.0, 0.0])
    return dummy


def add_floor(sim):
    """Garante um chão respondable sob a cena."""
    if sim.getObject('/Floor', {'noError': True}) != -1:
        return
    if os.path.exists(FLOOR_MODEL):
        sim.loadModel(FLOOR_MODEL)
        return
    floor = sim.createPrimitiveShape(sim.primitiveshape_cuboid, [6.0, 6.0, 0.2], 0)
    sim.setObjectAlias(floor, 'Floor')
    sim.setObjectPosition(floor, sim.handle_world, [0.0, 0.0, -0.1])
    sim.setObjectInt32Param(floor, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(floor, sim.shapeintparam_static, 1)


# =============================================================================
#  A escada
# =============================================================================

def add_stairs(sim, riser, tread, steps, width, x0, landing):
    """Degraus encaixados, mais um patamar no topo.

    Cada degrau é uma caixa que vai da própria quina até o fim da escada, uma
    dentro da outra. Sai mais barato que modelar o perfil em L — para o
    contato só importa a superfície de cima e a face da frente — e evita vãos
    entre peças, onde a ponta de um raio poderia prender.

    O PATAMAR não é decoração. Sem ele, "chegou ao topo" e "empinou na
    beirada" dão a mesma altura, e não há como distinguir sucesso de
    capotamento iminente — foi o que atrapalhou a primeira rodada de ensaios
    deste projeto.
    """
    parent = group(sim, 'stairs')
    for index in range(steps):
        height = riser * (index + 1)
        depth = tread * (steps - index)
        step = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [depth, width, height], 0)
        sim.setObjectAlias(step, 'step')
        sim.setObjectPosition(step, sim.handle_world,
                              [x0 + tread * index + depth / 2.0, 0.0, height / 2.0])
        sim.setObjectInt32Param(step, sim.shapeintparam_respondable, 1)
        sim.setObjectInt32Param(step, sim.shapeintparam_static, 1)
        sim.setShapeColor(step, None, sim.colorcomponent_ambient_diffuse,
                          [0.72, 0.58, 0.42])
        sim.setObjectParent(step, parent, True)

    top_x = x0 + tread * steps
    top_z = riser * steps
    pad = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid, [landing, width, top_z], 0)
    sim.setObjectAlias(pad, 'landing')
    sim.setObjectPosition(pad, sim.handle_world,
                          [top_x + landing / 2.0, 0.0, top_z / 2.0])
    sim.setObjectInt32Param(pad, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(pad, sim.shapeintparam_static, 1)
    sim.setShapeColor(pad, None, sim.colorcomponent_ambient_diffuse,
                      [0.62, 0.70, 0.52])
    sim.setObjectParent(pad, parent, True)
    return top_x, top_z


# =============================================================================
#  O rover
# =============================================================================

#: Máscara de resposta a colisão: só o byte ALTO ligado.
#:
#: O CoppeliaSim só faz dois shapes colidirem se as máscaras deles tiverem um
#: bit em comum, e o byte BAIXO vale apenas entre peças da mesma árvore
#: dinâmica. Zerá-lo faz o rover não colidir consigo mesmo — e aqui isso virou
#: obrigatório: as três pernas de uma roda emendam umas nas outras quando
#: fechadas, e com a máscara cheia o motor de física passa a vida resolvendo o
#: contato entre elas, o que trava a roda em vez de fechá-la.
NO_SELF_COLLISION = 0xff00


def arc_parts(sim, center, radius, start, span, width, thick, segments=8):
    """Um arco de circunferência, aproximado por `segments` cuboides.

    `center` é o centro do arco no MUNDO, `radius` o raio da linha média,
    `start` o ângulo inicial e `span` quanto de ângulo o arco cobre. O arco
    vive no plano x-z do mundo, que é o plano em que as rodas giram.

    São cuboides, e não uma malha curva, porque cada cuboide é CONVEXO e o
    CoppeliaSim trata o shape agrupado como um conjunto de convexos. Uma casca
    curva única seria côncava, e o motor de física produz contatos errados com
    ela — o mesmo problema que a banana do trabalho anterior deu.

    `segments` é o que decide o quanto o aro FECHADO é redondo de verdade, e é
    um compromisso com o custo: cada cuboide é um corpo convexo que o motor de
    física testa a cada passo, e são `4 x spokes x segments` deles no rover
    inteiro. Com 8 por perna a flecha de cada corda vale
    `radius(1 - cos(span/2n))`, uns 0,1 mm na roda padrão — menos de um décimo
    do que o aro balança por outras razões — e o rover fica com 96 convexos,
    que a simulação aguenta em tempo útil.
    """
    x, y, z = center
    parts = []
    for piece in range(segments):
        a0 = start + span * piece / segments
        a1 = start + span * (piece + 1) / segments
        p0 = (radius * math.cos(a0), radius * math.sin(a0))
        p1 = (radius * math.cos(a1), radius * math.sin(a1))
        middle = ((p0[0] + p1[0]) / 2.0, (p0[1] + p1[1]) / 2.0)
        length = math.dist(p0, p1)
        angle = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        # 1,15 de sobra no comprimento para os cuboides se sobreporem: sem
        # isso ficam frestas entre eles, e a quina do degrau entra nelas.
        segment = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [length * 1.15, width, thick], 0)
        sim.setObjectPosition(segment, sim.handle_world,
                              [x + middle[0], y, z + middle[1]])
        # O wheg gira no plano x-z do mundo; o eixo da junta é o y.
        sim.setObjectOrientation(segment, sim.handle_world,
                                 [math.pi / 2.0, -angle, 0.0])
        parts.append(segment)
    return parts


def hub_parts(sim, center, pivot_radius, spokes, width=0.007, thick=0.007):
    """O quadro da roda: o cubo central e um braço até o eixo de cada perna.

    É a peça em Y da roda original. Ela não toca o chão em nenhum dos dois
    estados — quem toca é sempre a perna — e por isso é fina: o que ela faz é
    segurar os eixos das pernas no lugar certo do aro.
    """
    x, y, z = center
    parts = []
    for index in range(spokes):
        angle = 2.0 * math.pi * index / spokes
        arm = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [pivot_radius, width, thick], 0)
        sim.setObjectPosition(
            arm, sim.handle_world,
            [x + pivot_radius / 2.0 * math.cos(angle), y,
             z + pivot_radius / 2.0 * math.sin(angle)])
        sim.setObjectOrientation(arm, sim.handle_world,
                                 [math.pi / 2.0, -angle, 0.0])
        parts.append(arm)
    core = sim.createPrimitiveShape(
        sim.primitiveshape_cylinder, [0.020, 0.020, 0.014], 0)
    sim.setObjectPosition(core, sim.handle_world, [x, y, z])
    sim.setObjectOrientation(core, sim.handle_world, [math.pi / 2.0, 0.0, 0.0])
    parts.append(core)
    return parts


def add_wheel(sim, name, center, drive_joint, args):
    """Uma roda transformável inteira, presa à junta de tração.

    A árvore dinâmica que sai daqui:

        <junta de tração>  ->  cubo  ->  <junta de perna> x3  ->  perna x3

    FECHADA (perna em 0°) os três dorsos emendam e formam um aro de raio
    `args.wheg_radius`: rola como roda, sem balanço. ABERTA (perna no outro
    extremo do intervalo) as pontas saem para fora em catavento e alcançam
    `geometry.claw_reach()`, quase o dobro.

    O ZERO É UM BATENTE, NÃO UMA POSIÇÃO DE CONTROLE
    ------------------------------------------------
    O intervalo da junta de perna vai de zero ao ângulo de abertura, do lado
    que `geometry.LEG_RAKE` mandar. Com a roda fechada, o peso do rover
    empurra a perna CONTRA o batente em zero, e quem segura é o batente — o
    motor da perna fica sem carga.

    Que seja sempre assim é geometria, não sorte: o ponto de contato no chão
    fica sempre adiante do eixo da perna, no mesmo sentido em que ela fecha, e
    por isso qualquer carga vertical fecha a roda em vez de abri-la. É o
    equivalente, aqui, de as pernas fechadas do mecanismo original se apoiarem
    umas nas outras — e é o que evita ter de pôr num motorzinho de perna o
    torque que sustenta o robô.
    """
    radius = args.wheg_radius
    spokes = args.spokes
    pivot_radius = geometry.LEG_PIVOT * radius
    # A espessura da casca é o que falta para a linha média virar o raio
    # externo: assim a roda fechada tem exatamente `radius` de raio.
    thick = 2.0 * (radius - pivot_radius)
    span = geometry.leg_span(spokes)
    # Uma fresta entre o fim de um dorso e o eixo da perna seguinte. Sem ela as
    # pernas fechadas se tocam e não há como abrir sem raspar. Ela é um furo
    # no aro, então quanto menor melhor: 2° valem 1,5 mm de arco na roda
    # padrão, menos que a espessura da casca.
    gap = math.radians(2.0)

    hub = sim.groupShapes(hub_parts(sim, center, pivot_radius, spokes))
    sim.setObjectAlias(hub, f'{name}_hub')
    sim.setObjectInt32Param(hub, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(hub, sim.shapeintparam_respondable_mask,
                            NO_SELF_COLLISION)
    sim.setObjectInt32Param(hub, sim.shapeintparam_static, 0)
    sim.setShapeMass(hub, geometry.WHEG_MASS * 0.4)
    sim.setShapeColor(hub, None, sim.colorcomponent_ambient_diffuse,
                      [0.28, 0.30, 0.34])
    sim.setObjectParent(hub, drive_joint, True)

    legs = []
    for index in range(spokes):
        base = 2.0 * math.pi * index / spokes
        # `LEG_RAKE` decide para que lado o dorso sai do eixo da perna, e com
        # isso se a PONTA chega ao degrau antes ou depois do eixo. Ver o
        # comentário dele em geometry.py: com o sinal trocado o rover empina
        # 72° no primeiro degrau e não sobe.
        rake = geometry.LEG_RAKE
        leg = sim.groupShapes(arc_parts(
            sim, center, pivot_radius, base + rake * gap / 2.0,
            rake * (span - gap), width=args.leg_width, thick=thick))
        sim.setObjectAlias(leg, f'{name}_leg{index}')
        sim.setObjectInt32Param(leg, sim.shapeintparam_respondable, 1)
        sim.setObjectInt32Param(leg, sim.shapeintparam_respondable_mask,
                                NO_SELF_COLLISION)
        sim.setObjectInt32Param(leg, sim.shapeintparam_static, 0)
        sim.setShapeMass(leg, geometry.WHEG_MASS * 0.6 / spokes)
        sim.setShapeColor(leg, None, sim.colorcomponent_ambient_diffuse,
                          [0.52, 0.53, 0.56])

        joint = sim.createJoint(sim.joint_revolute, sim.jointmode_dynamic, 0)
        sim.setObjectAlias(joint, f'{name}_leg{index}_joint')
        sim.setObjectPosition(
            joint, sim.handle_world,
            [center[0] + pivot_radius * math.cos(base), center[1],
             center[2] + pivot_radius * math.sin(base)])
        sim.setObjectOrientation(joint, sim.handle_world,
                                 [math.pi / 2.0, 0.0, 0.0])
        # A API recebe (mínimo, AMPLITUDE), e não (mínimo, máximo). O
        # intervalo vai de zero ao ângulo de abertura, do lado que `LEG_RAKE`
        # mandar: os dois estados da roda são os dois extremos dele, e o zero
        # — a roda fechada — é um batente de verdade.
        sim.setJointInterval(
            joint, False,
            [min(0.0, -rake * args.leg_open), args.leg_open])
        sim.setObjectParent(joint, hub, True)
        sim.setObjectParent(leg, joint, True)
        sim.setObjectInt32Param(joint, sim.jointintparam_dynctrlmode,
                                sim.jointdynctrl_position)
        sim.setJointTargetForce(joint, geometry.LEG_TORQUE)
        sim.setJointTargetPosition(joint, 0.0)
        legs.append(joint)
    return hub, legs


def add_rover(sim, args):
    """O rover inteiro: chassi, cauda, IMU, quatro whegs e os sensores."""
    radius = args.wheg_radius
    # Altura do eixo: o raio do wheg mais uma folga, para ele nascer apoiado
    # na ponta de um raio e não enterrado no chão.
    axle_z = radius + 0.004
    half_base = args.wheelbase / 2.0
    half_track = args.track / 2.0
    body_length = args.wheelbase + 0.05

    parent = group(sim, 'rover')

    chassis = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid,
        [body_length, args.track - 0.03, args.body_height], 0)
    sim.setObjectAlias(chassis, 'chassis')
    sim.setObjectPosition(chassis, sim.handle_world,
                          [args.start_x, 0.0, axle_z])
    sim.setObjectInt32Param(chassis, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(chassis, sim.shapeintparam_respondable_mask,
                            NO_SELF_COLLISION)
    sim.setObjectInt32Param(chassis, sim.shapeintparam_static, 0)
    sim.setShapeMass(chassis, geometry.CHASSIS_MASS)
    sim.setShapeColor(chassis, None, sim.colorcomponent_ambient_diffuse,
                      [0.30, 0.35, 0.40])

    if args.tail > 0.0:
        angle = geometry.TAIL_ANGLE
        tail = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [args.tail, 0.012, 0.008], 0)
        sim.setObjectPosition(
            tail, sim.handle_world,
            [args.start_x - body_length / 2.0 - args.tail / 2.0 * math.cos(angle),
             0.0, axle_z + args.tail / 2.0 * math.sin(angle)])
        sim.setObjectOrientation(tail, sim.handle_world, [0.0, -angle, 0.0])
        sim.setObjectInt32Param(tail, sim.shapeintparam_respondable, 1)
        sim.setObjectInt32Param(tail, sim.shapeintparam_static, 0)
        sim.setShapeMass(tail, geometry.TAIL_MASS)
        chassis = sim.groupShapes([chassis, tail])
        sim.setObjectAlias(chassis, 'chassis')
        sim.setObjectInt32Param(chassis, sim.shapeintparam_respondable_mask,
                                NO_SELF_COLLISION)
        sim.setShapeColor(chassis, None, sim.colorcomponent_ambient_diffuse,
                          [0.30, 0.35, 0.40])
    sim.setObjectParent(chassis, parent, True)

    # O IMU: criado depois do agrupamento e alinhado ao MUNDO. Ver o cabeçalho
    # deste arquivo para o porquê de não se poder usar o frame do chassi.
    imu = sim.createDummy(0.015)
    sim.setObjectAlias(imu, 'imu')
    sim.setObjectPosition(imu, sim.handle_world, [args.start_x, 0.0, axle_z])
    sim.setObjectOrientation(imu, sim.handle_world, [0.0, 0.0, 0.0])
    sim.setObjectParent(imu, chassis, True)

    joints = {}
    for name, x, y in (('fl', +half_base, +half_track),
                       ('fr', +half_base, -half_track),
                       ('rl', -half_base, +half_track),
                       ('rr', -half_base, -half_track)):
        joint = sim.createJoint(sim.joint_revolute, sim.jointmode_dynamic, 0)
        sim.setObjectAlias(joint, f'{name}_joint')
        sim.setObjectPosition(joint, sim.handle_world,
                              [args.start_x + x, y, axle_z])
        sim.setObjectOrientation(joint, sim.handle_world,
                                 [math.pi / 2.0, 0.0, 0.0])
        sim.setObjectParent(joint, chassis, True)
        sim.setObjectInt32Param(joint, sim.jointintparam_dynctrlmode,
                                sim.jointdynctrl_position)
        sim.setJointTargetForce(joint, geometry.WHEG_TORQUE)
        add_wheel(sim, name, (args.start_x + x, y, axle_z), joint, args)
        joints[name] = joint

    add_sensors(sim, chassis, args, axle_z, body_length)

    # O rover não pode ser visto pelos próprios sensores. Os raios dos whegs
    # varrem até 0,125 m à frente do centro — à FRENTE dos sensores — e sem
    # isto eles acusam obstáculo a 0,000 m o tempo todo, que foi o que os
    # primeiros ensaios mediram. Num robô de verdade o problema é o mesmo, e a
    # solução também: montar o sensor de modo que ele não enxergue o corpo.
    for shape in sim.getObjectsInTree(parent, sim.sceneobject_shape):
        sim.setBoolProperty(shape, 'detectable', False)
    return chassis, joints, axle_z


def ray_params(range_m, min_m=0.0, point_size=0.005):
    """Os 15 floats que `sim.createProximitySensor` espera para um sensor raio.

    A API recebe os parâmetros numa lista posicional sem nomes, e só os três
    primeiros importam aqui: deslocamento da origem, ALCANCE e tamanho do
    volume de detecção (zero, que é o que faz dele um raio fino em vez de um
    cone). Os demais descrevem cones e pirâmides, e ficam em zero.
    """
    return [min_m, range_m, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0,
            0.0, point_size, 0.0, 0.0]


def add_sensors(sim, chassis, args, axle_z, body_length):
    """Os três sensores de terreno, e por que todos apontam para BAIXO.

    A primeira montagem foi a óbvia: raios horizontais na frente do chassi,
    para "ver o degrau". Ela não funciona neste robô, e a medição mostra o
    porquê. O wheg de três raios faz o corpo subir e descer 25 mm a cada passo
    e balançar ±15° — contra um espelho de 40 mm. Um raio horizontal baixo
    passa metade do tempo enfiado no chão e a outra metade por cima do degrau;
    medido, ele acusava o PISO em 50 % a 100 % das leituras, com a distância
    governada pelo balanço e não pelo degrau.

    Os três sensores daqui apontam para baixo e para a frente, e o que eles
    informam não é "detectei" e sim A DISTÂNCIA:

    `probe_l`, `probe_r` — 30° abaixo da horizontal, um de cada lado. Sobre
        piso plano o raio percorre uma distância previsível até o chão
        (altura / sen 30°). Com um espelho à frente, ele bate na face ANTES
        disso: a leitura ENCURTA. É esse encurtamento, e não um limiar
        absoluto, que denuncia o degrau — e a diferença entre os dois lados
        mede o desalinhamento do rover com a face do degrau, que é o jeito
        mais comum de falhar numa escada.

    `edge` — 55°, bem à frente. Enquanto há piso, lê curto; na beirada do
        patamar, o raio mergulha no vazio e a leitura ESTOURA (ou some). É o
        que faz o rover parar no topo em vez de atravessar o patamar e cair do
        outro lado, que foi o que ele fez nos primeiros ensaios.

    Os três ficam na aresta SUPERIOR da frente. Montados em cima, o balanço do
    corpo muda menos o ponto em que o raio toca o chão do que se estivessem
    rentes a ele.
    """
    half = body_length / 2.0
    sensor_z = axle_z + args.body_height / 2.0
    for name, y, tilt, reach in (
            ('probe_l', +args.track / 3.0, 30.0, 0.35),
            ('probe_r', -args.track / 3.0, 30.0, 0.35),
            ('edge', 0.0, 55.0, 0.35)):
        sensor = sim.createProximitySensor(
            sim.proximitysensor_ray, 16, 0, [0] * 8, ray_params(range_m=reach))
        sim.setObjectAlias(sensor, name)
        sim.setObjectPosition(sensor, sim.handle_world,
                              [args.start_x + half, y, sensor_z])
        # O raio sai pelo +z do sensor: 90° em y o aponta para o +x do mundo
        # (a frente do rover), e o resto o inclina para baixo.
        sim.setObjectOrientation(
            sensor, sim.handle_world,
            [0.0, math.pi / 2.0 + math.radians(tilt), 0.0])
        sim.setObjectParent(sensor, chassis, True)


def flat_ground_range(axle_z, body_height, tilt_deg):
    """Distância que um raio inclinado percorre até o piso plano, em metros.

    É a leitura de referência: tudo mais curto que isto é alguma coisa
    subindo à frente. Quem usa é o nó `climber`, que recebe o valor como
    parâmetro em vez de um limiar mágico.
    """
    height = axle_z + body_height / 2.0
    return height / math.sin(math.radians(tilt_deg))


# =============================================================================
#  Programa
# =============================================================================

def build(sim, args):
    for alias in ('/rover', '/stairs', '/chassis', '/wheg'):
        purge(sim, alias)
    add_floor(sim)
    top_x, top_z = add_stairs(sim, args.riser, args.tread, args.steps,
                              args.width, args.stair_x, args.landing)
    _, _, axle_z = add_rover(sim, args)
    return {'top_x': top_x, 'top_z': top_z, 'axle_z': axle_z}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=23000)

    rover = parser.add_argument_group('rover')
    rover.add_argument('--wheg-radius', type=float, default=geometry.WHEG_RADIUS,
                       help='raio da roda FECHADA [m]; aberta, a garra alcança '
                            'quase o dobro')
    rover.add_argument('--spokes', type=int, default=geometry.SPOKES,
                       help='pernas por roda; menos sobe melhor e anda pior')
    rover.add_argument('--leg-open', type=float,
                       default=math.degrees(geometry.LEG_OPEN),
                       help='quanto a perna abre [graus]; mais alcança mais '
                            'alto e apoia pior')
    rover.add_argument('--leg-width', type=float, default=0.012,
                       help='largura da perna [m]')
    rover.add_argument('--wheelbase', type=float, default=geometry.WHEELBASE)
    rover.add_argument('--track', type=float, default=geometry.TRACK)
    rover.add_argument('--body-height', type=float, default=0.030)
    rover.add_argument('--tail', type=float, default=geometry.TAIL_LENGTH,
                       help='comprimento da cauda [m]; sem ela o rover capota '
                            'de costas no terceiro degrau')
    rover.add_argument('--no-tail', dest='tail', action='store_const', const=0.0)
    rover.add_argument('--start-x', type=float, default=0.0,
                       help='onde o rover nasce [m]')

    stairs = parser.add_argument_group('escada')
    stairs.add_argument('--riser', type=float, default=geometry.RISER,
                        help='altura do degrau [m]')
    stairs.add_argument('--tread', type=float, default=geometry.TREAD,
                        help='profundidade do degrau [m]')
    stairs.add_argument('--steps', type=int, default=geometry.STEPS)
    stairs.add_argument('--width', type=float, default=geometry.STAIR_WIDTH)
    stairs.add_argument('--stair-x', type=float, default=geometry.STAIR_X0,
                        help='onde começa o primeiro espelho [m]')
    stairs.add_argument('--landing', type=float, default=geometry.LANDING,
                        help='comprimento do patamar no topo [m]')

    parser.add_argument('--no-save', action='store_true')
    args = parser.parse_args()
    # A linha de comando fala em graus porque é como se pensa num ângulo de
    # abertura; o resto do programa fala em radianos, como todo o pacote.
    args.leg_open = math.radians(args.leg_open)

    try:
        client = RemoteAPIClient(args.host, args.port)
        sim = client.require('sim')
    except Exception:
        raise SystemExit(f'CoppeliaSim não responde em {args.host}:{args.port}.')

    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            pass

    info = build(sim, args)

    reach = geometry.claw_reach(args.wheg_radius, args.leg_open, args.spokes)
    print(f'rover: roda de {args.wheg_radius * 1000:.0f} mm com {args.spokes} '
          f'pernas, entre-eixos {args.wheelbase * 1000:.0f} mm, '
          + (f'cauda de {args.tail * 1000:.0f} mm' if args.tail else 'SEM CAUDA'))
    print(f'  fechada: aro de {args.wheg_radius * 1000:.0f} mm, balanço 0 mm')
    print(f'  aberta ({math.degrees(args.leg_open):.0f}°): garra alcança '
          f'{reach * 1000:.0f} mm ({reach / args.wheg_radius:.2f} raios), '
          f'balanço do corpo por passo '
          f'{geometry.body_bounce(reach, args.spokes) * 1000:.1f} mm')
    catch = geometry.tail_catch_angle(reach, tail=args.tail,
                                      wheelbase=args.wheelbase)
    print(f'  eixo a {info["axle_z"] * 1000:.0f} mm do chão fechada, '
          f'{reach * 1000:.0f} mm aberta')
    print(f'  cauda pega aos '
          f'{math.degrees(catch) if catch else float("nan"):.0f}° de pitch, '
          f'razão garra/entre-eixos {reach / args.wheelbase:.2f}')
    print(f'escada: {args.steps} degraus de {args.riser * 1000:.0f} x '
          f'{args.tread * 1000:.0f} mm, de x={args.stair_x:.2f} a '
          f'x={info["top_x"]:.2f}, topo a {info["top_z"] * 1000:.0f} mm')
    print(f'  patamar de {args.landing * 1000:.0f} mm')
    ratio = args.riser / reach
    print(f'  degrau / alcance da garra = {ratio:.2f}'
          + ('  (acima de 1,0 a ponta bate na face em vez de enganchar a quina)'
             if ratio > 1.0 else ''))
    print(f'  degrau / raio da roda fechada = '
          f'{args.riser / args.wheg_radius:.2f}'
          + ('  (acima de ~1 raio o aro fechado não alcança a quina)'
             if args.riser > args.wheg_radius else ''))

    if args.no_save:
        print('\n--no-save: a cena ficou na memória, sem gravar.')
        return 0
    for target in SCENE_TARGETS:
        target = os.path.abspath(target)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        sim.saveScene(target)
        print(f'salva em {target}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
