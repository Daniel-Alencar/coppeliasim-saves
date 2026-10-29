#!/usr/bin/env python3
"""Monta o corpo do Microduck no CoppeliaSim a partir do MJCF do próprio robô.

O Microduck é um bípede de 25 cm da Pollen Robotics, governado por políticas de
aprendizado por reforço. O repositório dele (`microduck/`) é o cérebro: os
daemons em Rust, o laço de controle a 50 Hz e as políticas ONNX. O CORPO vive
noutro repositório (`microduck_rl`), em MuJoCo, e é com ele que o `robotd`
conversa quando roda em simulação.

Este script põe o CoppeliaSim nesse lugar. Ele lê a cinemática do arquivo que
ESTE repositório já carrega —

    microduck/kinematics/assets/alpha/robot_walk.xml

— e constrói o pato peça por peça pela ZeroMQ Remote API, como o rover de whegs
do trabalho 10 é construído.

    python3 build_duck.py                  # monta e salva
    python3 build_duck.py --no-save
    python3 build_duck.py --mjcf <caminho>

O MJCF NÃO TEM GEOMETRIA
------------------------
Ele tem a árvore cinemática, as juntas com os seus limites e a INÉRCIA de cada
corpo — e nenhum `<geom>`. As malhas vivem no outro repositório. Montar o pato
aqui exige, então, inventar a forma de cada peça, e a escolha feita é a única
que não inventa física junto: para cada corpo, a caixa de densidade uniforme
cuja inércia é a que o MJCF declara. Dados `m` e a diagonal de inércia,

    a² = (Iyy + Izz − Ixx)·12/m ... / 2

e assim por diante. A caixa que sai tem a massa certa, a inércia certa e o
centro de massa no lugar certo; o que ela não tem é a aparência do pato. Para o
que se quer medir — o laço de controle fechando contra um corpo com a dinâmica
do robô — é a parte que importa.

A exceção são os PÉS. O MJCF marca a sola com um `site`, e a caixa de inércia
do tornozelo não chega lá: sem uma sola explícita o pato não tem em que se
apoiar. Então cada pé ganha uma placa fina na posição do site.

A ORDEM DAS JUNTAS NÃO É A DO ARQUIVO
--------------------------------------
O protocolo indexa as juntas POSICIONALMENTE, e a ordem canônica está em
`duck_ipc_proto::JOINT_NAMES`, com quinze entradas. O MJCF tem catorze: falta
`mouth`, que as políticas alfa não acionam. O mapeamento está em
`JOINT_ORDER` abaixo e é o mesmo que `duck_body.py` usa — mudá-lo num lugar só
desalinha o vetor inteiro de observação, que é o tipo de erro que não aparece
como erro.
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
DEFAULT_MJCF = os.path.join(
    REPO, 'microduck', 'kinematics', 'assets', 'alpha', 'robot_walk.xml')
SCENE = os.path.join(HERE, 'microduck.ttt')

#: A ordem canônica do protocolo (`duck_ipc_proto::JOINT_NAMES`), quinze nomes.
#: `mouth` não existe no MJCF e fica como `None` no modelo: o lugar dela no
#: vetor continua existindo, e é isso que importa para os índices baterem.
JOINT_ORDER = (
    'left_hip_yaw', 'left_hip_roll', 'left_hip_pitch', 'left_knee',
    'left_ankle', 'neck_pitch', 'head_pitch', 'head_yaw', 'head_roll',
    'mouth', 'right_hip_yaw', 'right_hip_roll', 'right_hip_pitch',
    'right_knee', 'right_ankle',
)

#: Pose de repouso, de `duck_control::model::DEFAULT_POSITION` [rad]. É a pose
#: contra a qual as políticas foram treinadas: elas observam a posição de cada
#: junta RELATIVA a ela, então o pato nasce aqui e não em zeros.
DEFAULT_POSITION = (
    0.0, -0.0873, -0.4579, -0.0049, 0.4530,
    0.3491, 0.3491, 0.0, 0.0, 0.0,
    0.0, 0.0873, 0.4579, 0.0049, -0.4530,
)

#: Espessura mínima de uma peça [m]. A caixa de inércia de uma peça pequena e
#: alongada sai com uma dimensão quase nula, e o motor de física trata mal
#: casca fina: isto é o piso.
MIN_SIZE = 0.004

#: A sola de cada pé [m]: comprimento, largura, espessura.
FOOT_PAD = (0.055, 0.035, 0.006)

#: Ganho proporcional do servo de posição de cada junta.
#:
#: O padrão do CoppeliaSim é 0,1, e ele é MOLE demais para uma política de
#: 50 Hz: um degrau de 0,3 rad leva 205 ms para assentar, dez tiques de
#: controle. A política não é um gerador de marcha com relógio — ela é
#: reativa, e a marcha nasce do laço fechado com o corpo. Se a junta não chega
#: ao alvo dentro do tique, a realimentação nunca se desenvolve e o pato fica
#: de pé parado, que foi exatamente o que a primeira corrida fez.
#:
#: Medido, com degrau de 0,3 rad no joelho:
#:
#:     P = 0,1    32 % em 20 ms, assenta em 205 ms
#:     P = 1,0    78 % em 20 ms, assenta em  25 ms, sem overshoot
#:     P = 3,0    78 % em 20 ms, assenta em  25 ms, overshoot de 0,032 rad
#:     P = 10     idem, overshoot de 0,053 rad
#:
#: Acima de 1,0 não se ganha velocidade, só oscilação.
#:
#: A propriedade é POR MOTOR DE FÍSICA (`bullet.posPid`, `ode.posPid`, ...), e
#: o `sim.setObjectFloatParam(h, sim.jointfloatparam_pid_p, ...)` — o caminho
#: antigo e óbvio — não a alcança: ele aceita a escrita e o valor lido de volta
#: continua 0,1. Custou uma varredura inteira de ganho que não mudava nada.
SERVO_GAIN = 1.0

#: Os motores de física cujo PID recebe `SERVO_GAIN`. Escrever em todos custa
#: nada e faz a cena continuar valendo se alguém trocar de motor.
ENGINES = ('bullet', 'ode', 'vortex', 'newton', 'mujoco')

#: Só o byte alto da máscara de colisão: as peças do pato não colidem entre si.
#: Num bípede de quinze corpos encostados isto não é otimização, é requisito —
#: com a máscara cheia a coxa e a canela brigam a cada passo.
NO_SELF_COLLISION = 0xff00


# =============================================================================
#  Do MJCF para caixas
# =============================================================================

def numbers(text, count=None):
    """Lê os floats de um atributo do MJCF."""
    values = [float(piece) for piece in text.split()]
    if count is not None and len(values) != count:
        raise ValueError(f'esperava {count} números em "{text}"')
    return values


def box_from_inertia(mass, inertia):
    """Dimensões da caixa uniforme com essa massa e essa inércia [m, m, m].

    `inertia` é a diagonal (Ixx, Iyy, Izz). Para uma caixa de lados (a, b, c):

        Ixx = m(b² + c²)/12      =>  b² + c² = 12·Ixx/m = A
        Iyy = m(a² + c²)/12      =>  a² + c² = 12·Iyy/m = B
        Izz = m(a² + b²)/12      =>  a² + b² = 12·Izz/m = C

    e daí `a² = (B + C − A)/2`. A desigualdade triangular das inércias garante
    que os três saiam positivos para uma inércia fisicamente possível — mas o
    MJCF vem de um CAD, e uma peça achatada chega aqui com um valor na casa do
    arredondamento, então o resultado é truncado em `MIN_SIZE`.
    """
    ixx, iyy, izz = inertia
    a_sq = 6.0 * (iyy + izz - ixx) / mass
    b_sq = 6.0 * (ixx + izz - iyy) / mass
    c_sq = 6.0 * (ixx + iyy - izz) / mass
    return tuple(max(MIN_SIZE, math.sqrt(value) if value > 0.0 else 0.0)
                 for value in (a_sq, b_sq, c_sq))


def mjcf_quat(text):
    """Quaternion do MJCF (w, x, y, z) na ordem do CoppeliaSim (x, y, z, w)."""
    w, x, y, z = numbers(text, 4)
    norm = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    return [x / norm, y / norm, z / norm, w / norm]


def parse_bodies(path):
    """A árvore do MJCF como uma lista de dicionários, pais antes dos filhos."""
    root = ET.parse(path).getroot()
    world = root.find('worldbody')
    bodies = []

    def walk(node, parent):
        for element in node.findall('body'):
            inertial = element.find('inertial')
            joint = element.find('joint')
            body = {
                'name': element.get('name'),
                'parent': parent,
                'pos': numbers(element.get('pos', '0 0 0'), 3),
                'quat': mjcf_quat(element.get('quat', '1 0 0 0')),
                'mass': float(inertial.get('mass')) if inertial is not None else 0.01,
                'com': (numbers(inertial.get('pos'), 3)
                        if inertial is not None else [0.0, 0.0, 0.0]),
                'inertia': (numbers(inertial.get('fullinertia'), 6)[:3]
                            if inertial is not None else [1e-6] * 3),
                'joint': None if joint is None else {
                    'name': joint.get('name'),
                    'axis': numbers(joint.get('axis', '0 0 1'), 3),
                    'range': (numbers(joint.get('range'), 2)
                              if joint.get('range') else None),
                },
                'sites': {site.get('name'): numbers(site.get('pos'), 3)
                          for site in element.findall('site')},
                'free': element.find('freejoint') is not None,
            }
            bodies.append(body)
            walk(element, body['name'])

    walk(world, None)
    return bodies


# =============================================================================
#  A cena
# =============================================================================

def purge(sim, alias):
    """Apaga o que já existir com esse nome."""
    handle = sim.getObject(alias, {'noError': True})
    while handle != -1:
        try:
            sim.removeModel(handle)
        except Exception:
            sim.removeObjects(sim.getObjectsInTree(handle, sim.handle_all))
        handle = sim.getObject(alias, {'noError': True})


def add_floor(sim):
    """Um chão respondable sob o pato."""
    if sim.getObject('/Floor', {'noError': True}) != -1:
        return
    floor = sim.createPrimitiveShape(sim.primitiveshape_cuboid,
                                     [4.0, 4.0, 0.1], 0)
    sim.setObjectAlias(floor, 'Floor')
    sim.setObjectPosition(floor, sim.handle_world, [0.0, 0.0, -0.05])
    sim.setObjectInt32Param(floor, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(floor, sim.shapeintparam_static, 1)
    sim.setShapeColor(floor, None, sim.colorcomponent_ambient_diffuse,
                      [0.55, 0.55, 0.58])


def make_link(sim, body, parent, colour):
    """A caixa de um corpo, com a massa onde o MJCF diz que ela está.

    Essa distinção é o detalhe que decide se o pato fica de pé. No MJCF cada
    corpo tem um frame — onde a junta mora e onde a pose do filho é medida — e
    um `inertial pos`, que é onde está o centro de massa, quase nunca na
    origem: o tronco tem o dele 18 mm atrás, a casca da cabeça 33 mm abaixo.
    Pôr a caixa na origem do frame move o centro de massa do robô inteiro, e
    num bípede isso é a diferença entre ficar de pé e cair para a frente. A
    primeira versão deste arquivo fez isso, e o pato tombava 73° em um segundo.

    O CoppeliaSim não deixa separar as duas coisas: um shape primitivo tem o
    frame no centro da geometria, e `sim.relocateShapeFrame` devolve `False`
    para um shape destes. Então o frame do shape FICA no centro de massa, e
    quem desconta a diferença é `body_to_link`, ao posicionar os filhos. A
    conta é a mesma; o que muda é onde ela vive.
    """
    size = box_from_inertia(body['mass'], body['inertia'])
    shape = sim.createPrimitiveShape(sim.primitiveshape_cuboid, list(size), 0)
    sim.setObjectAlias(shape, body['name'])
    sim.setObjectInt32Param(shape, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(shape, sim.shapeintparam_respondable_mask,
                            NO_SELF_COLLISION)
    sim.setObjectInt32Param(shape, sim.shapeintparam_static, 0)
    sim.setShapeMass(shape, body['mass'])
    sim.setShapeColor(shape, None, sim.colorcomponent_ambient_diffuse, colour)
    sim.setObjectParent(shape, parent, True)
    sim.setObjectPosition(shape, parent, list(body['com']))
    sim.setObjectQuaternion(shape, parent, [0.0, 0.0, 0.0, 1.0])
    return shape, size


def body_to_link(point, body):
    """Um ponto do frame do CORPO no frame do SHAPE daquele corpo.

    Os dois frames têm a mesma orientação e diferem por uma translação: o
    centro de massa. Tudo que o MJCF dá no frame do corpo — a pose de um
    filho, um site — passa por aqui antes de virar posição na cena.
    """
    return [point[axis] - body['com'][axis] for axis in range(3)]


def add_foot(sim, parent, site_pos, name):
    """A sola de um pé, na posição que o MJCF marca com um site.

    Criada como shape separado e agrupada ao tornozelo: o que toca o chão tem
    de ser uma superfície larga, e a caixa de inércia do tornozelo é um cubo
    de poucos milímetros pendurado no eixo.
    """
    pad = sim.createPrimitiveShape(sim.primitiveshape_cuboid, list(FOOT_PAD), 0)
    sim.setObjectAlias(pad, name)
    sim.setObjectInt32Param(pad, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(pad, sim.shapeintparam_respondable_mask,
                            NO_SELF_COLLISION)
    sim.setObjectInt32Param(pad, sim.shapeintparam_static, 0)
    sim.setShapeMass(pad, 0.004)
    sim.setShapeColor(pad, None, sim.colorcomponent_ambient_diffuse,
                      [0.9, 0.65, 0.1])
    sim.setObjectParent(pad, parent, True)
    sim.setObjectPosition(pad, parent, list(site_pos))
    sim.setObjectQuaternion(pad, parent, [0.0, 0.0, 0.0, 1.0])
    return pad


def build(sim, args):
    """Monta o pato inteiro e devolve os handles que o servidor precisa."""
    purge(sim, '/duck')
    add_floor(sim)

    bodies = parse_bodies(args.mjcf)
    by_name = {body['name']: body for body in bodies}
    shapes = {}
    joints = {}

    root = sim.createDummy(0.01)
    sim.setObjectAlias(root, 'duck')
    sim.setObjectPosition(root, sim.handle_world, [0.0, 0.0, 0.0])

    for body in bodies:
        colour = ([0.95, 0.85, 0.2] if body['parent'] is None
                  else [0.55, 0.58, 0.62])

        if body['parent'] is None:
            # O trunk tem freejoint no MJCF: aqui é só um corpo dinâmico solto.
            # A altura vem do próprio MJCF (`trunk_base pos`), que é a pose em
            # que o modelo foi feito para nascer: com ela as solas encostam no
            # chão. Chutar um valor faz o pato cair 20 mm antes de o
            # controlador ter qualquer chance.
            height = args.height if args.height is not None else body['pos'][2]
            shape, _ = make_link(sim, body, root, colour)
            sim.setObjectPosition(
                shape, sim.handle_world,
                [body['com'][0], body['com'][1], height + body['com'][2]])
            shapes[body['name']] = shape
            continue

        parent_shape = shapes[body['parent']]
        spec = body['joint']
        joint = sim.createJoint(sim.joint_revolute, sim.jointmode_dynamic, 0)
        sim.setObjectAlias(joint, f'{spec["name"]}_joint')
        sim.setObjectParent(joint, parent_shape, True)
        # A pose da junta é a do corpo filho em relação ao pai: no MJCF a junta
        # mora no frame do filho, com o eixo dado nesse frame. E o frame do
        # shape do pai É o frame do corpo pai — é para isso que serve a
        # relocação em `make_link`.
        sim.setObjectPosition(joint, parent_shape,
                              body_to_link(body['pos'], by_name[body['parent']]))
        sim.setObjectQuaternion(joint, parent_shape, list(body['quat']))
        if spec['range']:
            low, high = spec['range']
            sim.setJointInterval(joint, False, [low, high - low])
        sim.setObjectInt32Param(joint, sim.jointintparam_dynctrlmode,
                                sim.jointdynctrl_position)
        sim.setJointTargetForce(joint, args.torque)
        for engine in ENGINES:
            sim.setFloatArrayProperty(joint, f'{engine}.posPid',
                                      [args.servo_gain, 0.0, 0.0])

        shape, _ = make_link(sim, body, joint, colour)
        shapes[body['name']] = shape
        joints[spec['name']] = joint

    # As solas, nos sites que o MJCF marca.
    for side in ('left', 'right'):
        for body in bodies:
            site = body['sites'].get(f'{side}_foot')
            if site is not None:
                add_foot(sim, shapes[body['name']],
                         body_to_link(site, body), f'{side}_foot')

    # O IMU: um dummy no site `imu` do trunk, alinhado ao corpo. A atitude que
    # o protocolo pede é a do tronco, e lê-la de um referencial explícito é o
    # que um IMU de verdade é.
    trunk = next(body for body in bodies if body['parent'] is None)
    imu = sim.createDummy(0.012)
    sim.setObjectAlias(imu, 'imu')
    sim.setObjectParent(imu, shapes[trunk['name']], True)
    sim.setObjectPosition(
        imu, shapes[trunk['name']],
        body_to_link(trunk['sites'].get('imu', [0.0, 0.0, 0.0]), trunk))
    sim.setObjectQuaternion(imu, shapes[trunk['name']], [0.0, 0.0, 0.0, 1.0])

    # A pose de repouso, que é a pose contra a qual as políticas foram
    # treinadas.
    for index, name in enumerate(JOINT_ORDER):
        if name in joints:
            sim.setJointPosition(joints[name], DEFAULT_POSITION[index])
            sim.setJointTargetPosition(joints[name], DEFAULT_POSITION[index])

    return {'bodies': by_name, 'shapes': shapes, 'joints': joints}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=23000)
    parser.add_argument('--mjcf', default=DEFAULT_MJCF,
                        help='o MJCF do pato (padrão: o do repo microduck)')
    parser.add_argument('--height', type=float, default=None,
                        help='altura em que o tronco nasce [m]; o padrão é a '
                             'do próprio MJCF, que já põe as solas no chão')
    parser.add_argument('--torque', type=float, default=1.5,
                        help='torque máximo de cada servo [N.m]')
    parser.add_argument('--servo-gain', type=float, default=SERVO_GAIN,
                        help='ganho P do servo de posição; o padrão do '
                             'CoppeliaSim (0,1) é mole demais para 50 Hz')
    parser.add_argument('--dt', type=float, default=0.005,
                        help='passo da simulação [s]; o laço do robô é 50 Hz, '
                             'e um passo grosso não fecha um bípede')
    parser.add_argument('--no-save', action='store_true')
    args = parser.parse_args()

    if not os.path.exists(args.mjcf):
        raise SystemExit(
            f'não achei o MJCF em {args.mjcf}.\n'
            'Ele vem do repositório microduck, em '
            'kinematics/assets/alpha/robot_walk.xml')

    try:
        client = RemoteAPIClient(args.host, args.port)
        sim = client.require('sim')
    except Exception:
        raise SystemExit(f'CoppeliaSim não responde em {args.host}:{args.port}.')

    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            pass

    sim.setFloatParam(sim.floatparam_simulation_time_step, args.dt)
    info = build(sim, args)

    trunk = next(b for b in info['bodies'].values() if b['parent'] is None)
    height = args.height if args.height is not None else trunk['pos'][2]
    mass = sum(body['mass'] for body in info['bodies'].values())
    print(f'pato montado: {len(info["shapes"])} corpos, '
          f'{len(info["joints"])} juntas, {mass * 1000:.0f} g, '
          f'tronco a {height * 1000:.0f} mm')
    print(f'  passo da simulação: {args.dt * 1000:.0f} ms '
          f'(o laço de controle do robô é 50 Hz = 20 ms)')
    print(f'  ganho do servo: P = {args.servo_gain:.1f} '
          f'(o padrão 0,1 não segue o alvo dentro de um tique)')
    missing = [name for name in JOINT_ORDER if name not in info['joints']]
    print(f'  ausentes do MJCF, presentes no protocolo: '
          f'{", ".join(missing) if missing else "nenhuma"}')

    if args.no_save:
        print('\n--no-save: a cena ficou na memória.')
        return 0
    sim.saveScene(SCENE)
    print(f'salva em {SCENE}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
