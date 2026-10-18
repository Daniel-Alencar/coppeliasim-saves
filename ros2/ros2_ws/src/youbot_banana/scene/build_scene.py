#!/usr/bin/env python3
"""Monta a cena do trabalho e a salva em .ttt.

A cena não foi montada a mão: ela é gerada por este script, pela ZeroMQ Remote
API, a partir dos modelos do CoppeliaSim e das bananas e poops dos exercícios
anteriores. Duas razões:

1. Fica reproduzível. O arranjo sai de uma semente (`--seed`), então o mesmo
   comando dá a mesma cena, e trocar a semente dá um cenário novo para repetir
   os experimentos — é o que permite medir taxa de sucesso em vez de contar uma
   anedota.
2. Fica explicado. Por que a banana tem um corpo de colisão cuboide invisível,
   por que a malha é filha estática dele, por que os scripts do modelo do
   youBot são desabilitados: está escrito aqui, junto do código que faz.

    python3 scene/build_scene.py                      # monta e salva
    python3 scene/build_scene.py --bananas 8 --seed 7
    python3 scene/build_scene.py --no-save            # só monta, para olhar

O que a cena tem
----------------
    /youBot                      o robô, com os scripts de demonstração calados
    /walls/wall[0..3]            as paredes da arena
    /poops/poop[i]               os obstáculos
    /bananas/banana[i]           as bananas: corpo de colisão + malha visual
    /Basket                      o cesto onde as bananas são depositadas

A ESCALA DAS BANANAS
--------------------
A garra do youBot tem vão útil de 26 mm (fechada) a 76 mm (aberta), medido em
`measure_arm.py`. A banana dos exercícios anteriores tem 100 mm de largura: não
cabe. Por isso ela entra reduzida (`--banana-scale`, 0,5 por padrão), ficando
com 103 x 50 x 17 mm — 50 mm de largura, que é o que a garra consegue envolver.
É uma mudança de cena, não de algoritmo, e está registrada aqui porque muda o
que o leitor vê rodando.

O CORPO DE COLISÃO
------------------
A malha da banana tem 1015 triângulos e é NÃO CONVEXA. Marcada como respondable
assim, o motor de física produz contatos errados: a garra empurrava a banana em
vez de envolvê-la, e o objeto era expelido. A solução é a canônica do
CoppeliaSim: um cuboide convexo do tamanho da banana faz o corpo dinâmico, e a
malha vira filha estática, só para ser vista. Quem fecha a garra é física de
verdade — nada de grudar o objeto na mão por software.
"""

import argparse
import math
import os
import random
import sys

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from youbot_banana.coppelia_scene import (  # noqa: E402  (precisa do sys.path)
    silence_joint_scripts, tree_bbox)

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..', '..'))

#: Modelos do CoppeliaSim. O caminho da instalação sai da variável de ambiente
#: COPPELIASIM_ROOT quando ela existe, para a cena poder ser montada em outra
#: máquina sem editar o script.
COPPELIA = os.environ.get(
    'COPPELIASIM_ROOT',
    os.path.expanduser('~/Softwares/CoppeliaSim/CoppeliaSim_Edu'))
YOUBOT_MODEL = os.path.join(COPPELIA, 'models/robots/mobile/KUKA YouBot.ttm')

FLOOR_MODEL = os.path.join(COPPELIA,
                           'models/infrastructure/floors/5mX5m wooden floor.ttm')

#: Modelos dos exercícios anteriores, neste repositório.
BANANA_MODEL = os.path.join(REPO, 'projects/7 - obstacle_avoidance/banana.ttm')
POOP_MODEL = os.path.join(REPO, 'projects/7 - obstacle_avoidance/poop.ttm')

#: Onde a cena é salva: junto dos outros projetos e dentro do pacote.
SCENE_TARGETS = (
    os.path.join(REPO, 'projects/9 - youbot_banana/youbot_banana.ttt'),
    os.path.join(HERE, '..', 'coppeliasim', 'youbot_banana.ttt'),
)


# =============================================================================
#  Utilidades
# =============================================================================

def purge(sim, alias):
    """Apaga tudo que já existe com esse nome, modelo ou não.

    `removeModel` só aceita a base de um modelo; os objetos criados aqui (as
    paredes, os corpos das bananas) não são modelos, e para eles vale
    `removeObjects` sobre a árvore inteira.
    """
    handle = sim.getObject(alias, {'noError': True})
    while handle != -1:
        try:
            sim.removeModel(handle)
        except Exception:
            sim.removeObjects(sim.getObjectsInTree(handle, sim.handle_all))
        handle = sim.getObject(alias, {'noError': True})


def bbox(sim, shape):
    """Dimensões (dx, dy, dz) da caixa envolvente de um shape, em metros."""
    low = [sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_min_x + i)
           for i in range(3)]
    high = [sim.getObjectFloatParam(shape, sim.objfloatparam_objbbox_max_x + i)
            for i in range(3)]
    return tuple(high[i] - low[i] for i in range(3))


def group(sim, name):
    """Cria um dummy para agrupar objetos na hierarquia da cena.

    É o que permite ao nó de comunicação descobrir os objetos percorrendo a
    árvore (/bananas/banana[0], /poops/poop[3]) em vez de ter coordenadas
    escritas no código — a exigência que os enunciados anteriores já faziam.
    """
    dummy = sim.createDummy(0.01)
    sim.setObjectAlias(dummy, name)
    sim.setObjectPosition(dummy, sim.handle_world, [0.0, 0.0, 0.0])
    return dummy


# =============================================================================
#  Peças da cena
# =============================================================================

def add_floor(sim, size):
    """Garante um chão respondable sob a arena.

    Sem chão o robô cai no vazio — e cai mesmo: foi o primeiro erro deste
    projeto, com o youBot despencando a -27 m enquanto o braço obedecia
    direitinho aos comandos.
    """
    if sim.getObject('/Floor', {'noError': True}) != -1:
        return
    if os.path.exists(FLOOR_MODEL):
        sim.loadModel(FLOOR_MODEL)
        return
    floor = sim.createPrimitiveShape(sim.primitiveshape_cuboid,
                                     [size + 1.0, size + 1.0, 0.2], 0)
    sim.setObjectAlias(floor, 'Floor')
    sim.setObjectPosition(floor, sim.handle_world, [0.0, 0.0, -0.1])
    sim.setObjectInt32Param(floor, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(floor, sim.shapeintparam_static, 1)


def add_walls(sim, size, height=0.25, thickness=0.05):
    """Quatro paredes fechando a arena de `size` x `size` metros.

    As paredes dão sentido físico à borda que o mapa de ocupação já bloqueava
    por conta própria (ver grid.block_border do exercício anterior): sem elas,
    um caminho que saísse da área mapeada seria só um erro invisível.
    """
    parent = group(sim, 'walls')
    half = size / 2.0
    for x, y, dx, dy in (
            (0.0, +half, size + thickness, thickness),
            (0.0, -half, size + thickness, thickness),
            (+half, 0.0, thickness, size + thickness),
            (-half, 0.0, thickness, size + thickness)):
        wall = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid, [dx, dy, height], 0)
        sim.setObjectAlias(wall, 'wall')
        sim.setObjectPosition(wall, sim.handle_world, [x, y, height / 2.0])
        sim.setObjectInt32Param(wall, sim.shapeintparam_respondable, 1)
        sim.setObjectInt32Param(wall, sim.shapeintparam_static, 1)
        sim.setShapeColor(wall, None, sim.colorcomponent_ambient_diffuse,
                          [0.55, 0.55, 0.60])
        sim.setObjectParent(wall, parent, True)
    return parent


def add_youbot(sim, position, yaw):
    """Carrega o youBot, o põe no lugar e cala os scripts da cena.

    Dois cuidados que custaram depuração:

    - A ORIENTAÇÃO. O frame do /youBot nasce em (-90°, 0°, -90°): forçar
      `setObjectOrientation` para [0, 0, 0] vira o robô de cabeça para baixo, e
      a física o expele pelo chão. Para girá-lo no plano, usa-se
      `rotateAroundAxis` em torno do eixo z do mundo, que compõe com a
      orientação natural em vez de substituí-la.
    - OS SCRIPTS. O modelo traz um script de demonstração que manda o robô
      andar de lado e girar, e outro que comanda as juntas da garra a cada
      passo. Os dois escreveriam nas mesmas juntas que os nós ROS, e quem
      escreve por último vence: como eles rodam a cada passo de simulação,
      venceriam sempre. A ponte também os desabilita ao subir (ver
      coppelia_bridge.silence_rival_scripts); aqui eles já saem calados do
      arquivo salvo, para a cena funcionar mesmo que alguém dê play à mão.
    """
    robot = sim.loadModel(YOUBOT_MODEL)
    reference = sim.getObject('/youBot/youBot_ref')
    # A altura vem do próprio modelo: ele carrega com as rodas no chão.
    _, _, z = sim.getObjectPosition(robot, sim.handle_world)
    sim.setObjectPosition(robot, sim.handle_world, [position[0], position[1], z])
    if abs(yaw) > 1e-9:
        # rotateAroundAxis opera sobre uma POSE, não sobre um handle: ele
        # devolve a pose girada, que depois se escreve no objeto. E o pivô é o
        # youBot_ref, para o robô girar em torno do centro da base e não em
        # torno da origem do frame torto do modelo.
        pivot = sim.getObjectPosition(reference, sim.handle_world)
        pose = sim.getObjectPose(robot, sim.handle_world)
        sim.setObjectPose(robot, sim.handle_world,
                          sim.rotateAroundAxis(pose, [0.0, 0.0, 1.0], pivot, yaw))

    # O critério de quais scripts calar está em coppelia_scene, para ser o
    # mesmo que a ponte aplica em tempo de execução.
    return robot, len(silence_joint_scripts(sim, robot))


def add_banana(sim, parent, position, yaw, scale, mass=0.05):
    """Uma banana agarrável: corpo de colisão convexo + malha visual.

    O corpo é um cuboide do tamanho da banana, respondable e dinâmico — é ele
    que a garra aperta. A malha da banana entra como filha ESTÁTICA e não
    respondable: estática para acompanhar o pai (uma filha não estática cai
    sozinha, como um corpo livre; foi o que aconteceu na primeira tentativa, a
    malha despencando a -1589 m enquanto o cuboide ficava), e não respondable
    para a física nunca ver a malha côncava.

    O corpo fica VISÍVEL em modo de arame só se alguém quiser inspecioná-lo;
    por padrão sai da camada visível, e o que se vê é a banana.
    """
    mesh = sim.loadModel(BANANA_MODEL)
    if abs(scale - 1.0) > 1e-9:
        sim.scaleObject(mesh, scale, scale, scale, 0)
    size = bbox(sim, mesh)

    body = sim.createPrimitiveShape(sim.primitiveshape_cuboid, list(size), 0)
    # O CoppeliaSim numera sozinho os homônimos (banana, banana[1], ...),
    # que é a convenção da cena do exercício anterior; escrever o índice à
    # mão viraria 'banana_0_', porque ele troca os colchetes do alias.
    sim.setObjectAlias(body, 'banana')
    sim.setObjectInt32Param(body, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(body, sim.shapeintparam_static, 0)
    sim.setShapeMass(body, mass)
    # Fora da camada visível: o que se vê é a malha, não a caixa.
    sim.setObjectInt32Param(body, sim.objintparam_visibility_layer, 0)
    sim.setObjectPosition(body, sim.handle_world,
                          [position[0], position[1], size[2] / 2.0 + 0.001])
    sim.setObjectOrientation(body, sim.handle_world, [0.0, 0.0, yaw])

    sim.setObjectInt32Param(mesh, sim.shapeintparam_respondable, 0)
    sim.setObjectInt32Param(mesh, sim.shapeintparam_static, 1)
    sim.setObjectPose(mesh, sim.handle_world, sim.getObjectPose(body, sim.handle_world))
    sim.setObjectAlias(mesh, 'mesh')
    sim.setObjectParent(mesh, body, True)
    sim.setObjectParent(body, parent, True)
    return body, size


def add_poop(sim, parent, position, yaw):
    """Um obstáculo. Estático e respondable: bater nele é um erro de verdade.

    No exercício anterior os poops eram decorativos (não respondable) e a
    punição por passar em cima era o script da cena espalhar sujeira. Aqui eles
    bloqueiam mesmo, para que "o robô não colidiu" seja uma medida e não uma
    impressão.
    """
    poop = sim.loadModel(POOP_MODEL)
    sim.setObjectAlias(poop, 'poop')
    size = bbox(sim, poop)
    sim.setObjectPosition(poop, sim.handle_world,
                          [position[0], position[1], size[2] / 2.0])
    sim.setObjectOrientation(poop, sim.handle_world, [0.0, 0.0, yaw])
    sim.setObjectInt32Param(poop, sim.shapeintparam_respondable, 1)
    sim.setObjectInt32Param(poop, sim.shapeintparam_static, 1)
    sim.setObjectParent(poop, parent, True)
    return poop, size


def add_deposit(sim, position, inner=0.26, wall=0.05, thickness=0.012):
    """A bandeja onde as bananas são depositadas, construída aqui.

    Não é um modelo pronto do CoppeliaSim, e a razão é geométrica. O que limita
    o depósito é o alcance do braço:

        a base para a   (meio-comprimento do robô) + (meia-largura do depósito)
                      = 0,293 m + metade do depósito
        o braço precisa alcançar  essa distância - 0,166 m  (o eixo da junta 0
                                  fica 0,166 m à frente do centro da base)

    e o envelope do braço com a garra apontada para baixo vai, medido, de 0,15 a
    0,32 m do eixo da junta 0 — e ENCURTA com a altura: a 0,20 m de altura o
    máximo cai para 0,275 m.

    Com o cesto de 0,80 m do CoppeliaSim a conta nem fecha (o braço precisaria
    de 0,52 m). Com a tigela de 0,26 m fecha, mas a borda dela fica a 0,17 m de
    altura, e soltar a banana acima disso deixa só 11 mm de folga no envelope.

    Uma bandeja RASA resolve: 0,26 m de lado e paredes de 0,05 m. A banana é
    solta a 0,10 m de altura, onde o braço alcança até 0,32 m e só precisa de
    0,264 — 56 mm de margem. É o tipo de decisão que o alcance do manipulador
    impõe ao resto da cena, e é melhor registrá-la do que descobri-la no
    relatório.
    """
    parent = group(sim, 'deposit')
    floor = sim.createPrimitiveShape(
        sim.primitiveshape_cuboid, [inner + 2 * thickness, inner + 2 * thickness,
                                    thickness], 0)
    sim.setObjectAlias(floor, 'tray')
    sim.setObjectPosition(floor, sim.handle_world,
                          [position[0], position[1], thickness / 2.0])
    pieces = [floor]
    half = (inner + thickness) / 2.0
    for dx, dy, sx, sy in ((half, 0.0, thickness, inner + 2 * thickness),
                           (-half, 0.0, thickness, inner + 2 * thickness),
                           (0.0, half, inner + 2 * thickness, thickness),
                           (0.0, -half, inner + 2 * thickness, thickness)):
        side = sim.createPrimitiveShape(sim.primitiveshape_cuboid,
                                        [sx, sy, wall], 0)
        sim.setObjectAlias(side, 'rim')
        sim.setObjectPosition(side, sim.handle_world,
                              [position[0] + dx, position[1] + dy, wall / 2.0])
        pieces.append(side)
    for piece in pieces:
        sim.setObjectInt32Param(piece, sim.shapeintparam_respondable, 1)
        sim.setObjectInt32Param(piece, sim.shapeintparam_static, 1)
        sim.setShapeColor(piece, None, sim.colorcomponent_ambient_diffuse,
                          [0.25, 0.45, 0.75])
        sim.setObjectParent(piece, parent, True)
    return parent, tree_bbox(sim, parent)


# =============================================================================
#  Arranjo
# =============================================================================

def scatter(count, keep_out, bounds, spacing, rng, tries=4000):
    """Sorteia `count` pontos no retângulo, longe entre si e das zonas vedadas.

    keep_out   lista de (x, y, raio) onde não pode cair nada (o robô, o cesto)
    spacing    distância mínima entre dois pontos sorteados [m]

    Rejeição simples, que é o bastante para a dezena de objetos desta cena. Se
    não couber tudo, devolve o que couber e avisa quem chamou — melhor uma cena
    com menos bananas do que duas bananas uma dentro da outra.
    """
    x_min, x_max, y_min, y_max = bounds
    points = []
    for _ in range(tries):
        if len(points) == count:
            break
        point = (rng.uniform(x_min, x_max), rng.uniform(y_min, y_max))
        if any(math.dist(point, (cx, cy)) < radius
               for cx, cy, radius in keep_out):
            continue
        if any(math.dist(point, other) < spacing for other in points):
            continue
        points.append(point)
    return points


def build(sim, args):
    """Monta a cena inteira e devolve um resumo do que entrou nela."""
    rng = random.Random(args.seed)

    # Além dos grupos da própria cena, limpa restos que tenham ficado soltos
    # no topo: um /banana_body de um teste anterior seria salvo junto e
    # apareceria como uma banana fantasma que o robô nunca alcança.
    for alias in ('/youBot', '/bananas', '/poops', '/walls', '/Basket',
                  '/Banana', '/poop', '/banana_body', '/banana', '/mesh',
                  '/deposit', '/Bowl'):
        purge(sim, alias)

    add_floor(sim, args.arena)
    add_walls(sim, args.arena)

    robot_start = (-args.arena / 2.0 + 0.65, -args.arena / 2.0 + 0.65)
    basket_at = (args.arena / 2.0 - 0.70, args.arena / 2.0 - 0.70)

    robot, silenced = add_youbot(sim, robot_start, math.radians(args.robot_yaw))
    _, deposit_size = add_deposit(sim, basket_at, args.deposit_size)

    # Zonas vedadas: em volta do robô (para ele não nascer em cima de nada) e
    # do cesto (para a aproximação de depósito ficar livre).
    keep_out = [(robot_start[0], robot_start[1], 0.80),
                (basket_at[0], basket_at[1], max(deposit_size[:2]) / 2.0 + 0.70)]

    # A borda interna: nada encostado na parede, ou o robô não tem como
    # contornar o objeto para chegar nele.
    margin = 0.55
    bounds = (-args.arena / 2.0 + margin, args.arena / 2.0 - margin,
              -args.arena / 2.0 + margin, args.arena / 2.0 - margin)

    poop_spots = scatter(args.poops, keep_out, bounds, 0.42, rng)
    poops = group(sim, 'poops')
    for spot in poop_spots:
        add_poop(sim, poops, spot, rng.uniform(-math.pi, math.pi))

    # As bananas ficam longe dos poops o bastante para a garra descer em volta
    # delas sem bater no obstáculo: meia largura da garra mais a meia largura
    # do poop, com folga.
    banana_keep_out = keep_out + [(x, y, 0.38) for x, y in poop_spots]
    banana_spots = scatter(args.bananas, banana_keep_out, bounds, 0.45, rng)
    bananas = group(sim, 'bananas')
    banana_size = None
    for spot in banana_spots:
        _, banana_size = add_banana(sim, bananas, spot,
                                    rng.uniform(-math.pi, math.pi),
                                    args.banana_scale)

    return {
        'robot': robot_start,
        'robot_yaw': args.robot_yaw,
        'silenced': silenced,
        'deposit': basket_at,
        'deposit_size': deposit_size,
        'poops': poop_spots,
        'bananas': banana_spots,
        'banana_size': banana_size,
        'arena': args.arena,
    }


# =============================================================================
#  Programa
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=23000)
    # 5 m de lado, e não 4: o youBot tem 0,42 x 0,57 m, o que dá 0,36 m de raio
    # circunscrito. Com a inflação do mapa de ocupação, cada obstáculo bloqueia
    # um disco de uns 0,45 m, e numa arena de 4 m com 8 poops sobra pouco
    # corredor por onde passar.
    parser.add_argument('--arena', type=float, default=5.0,
                        help='lado da arena entre as paredes [m]')
    parser.add_argument('--bananas', type=int, default=5)
    parser.add_argument('--poops', type=int, default=6)
    parser.add_argument('--deposit-size', type=float, default=0.26,
                        help='lado interno da bandeja de depósito [m]; '
                             'aumentar exige que a base pare mais longe, e o '
                             'braço alcança até 0,32 m')
    parser.add_argument('--banana-scale', type=float, default=0.5,
                        help='a garra segura de 26 a 76 mm; 1,0 dá 100 mm de '
                             'largura e não cabe')
    parser.add_argument('--robot-yaw', type=float, default=45.0,
                        help='rumo inicial do robô [graus]')
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--no-save', action='store_true',
                        help='monta a cena mas não grava o .ttt')
    args = parser.parse_args()

    for path, label in ((YOUBOT_MODEL, 'o modelo do youBot'),
                        (BANANA_MODEL, 'a banana dos exercícios anteriores'),
                        (POOP_MODEL, 'o poop dos exercícios anteriores')):
        if not os.path.exists(path):
            raise SystemExit(
                f'não achei {label} em:\n  {path}\n'
                'Ajuste COPPELIASIM_ROOT ou os caminhos no topo deste script.')

    try:
        client = RemoteAPIClient(args.host, args.port)
        sim = client.require('sim')
    except Exception:
        raise SystemExit(f'CoppeliaSim não responde em {args.host}:{args.port}.')

    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            pass

    summary = build(sim, args)

    print(f'arena de {summary["arena"]:.1f} x {summary["arena"]:.1f} m')
    print(f'robô em ({summary["robot"][0]:+.2f}, {summary["robot"][1]:+.2f}), '
          f'rumo {summary["robot_yaw"]:.0f}°, '
          f'{summary["silenced"]} script(s) do modelo calados')
    print(f'depósito em ({summary["deposit"][0]:+.2f}, '
          f'{summary["deposit"][1]:+.2f}), {summary["deposit_size"][0]:.2f} m de '
          f'largura, borda a {summary["deposit_size"][5]:.3f} m')
    print(f'{len(summary["poops"])} poops e {len(summary["bananas"])} bananas')
    if summary['banana_size']:
        size = summary['banana_size']
        print(f'banana: {size[0] * 1000:.0f} x {size[1] * 1000:.0f} x '
              f'{size[2] * 1000:.0f} mm (a garra envolve de 26 a 76 mm)')
    if len(summary['bananas']) < args.bananas:
        print(f'ATENÇÃO: só couberam {len(summary["bananas"])} das '
              f'{args.bananas} bananas pedidas. Aumente --arena ou diminua '
              '--poops.')

    if args.no_save:
        print('\n--no-save: a cena ficou montada na memória, sem gravar.')
        return 0

    for target in SCENE_TARGETS:
        target = os.path.abspath(target)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        sim.saveScene(target)
        print(f'salva em {target}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
