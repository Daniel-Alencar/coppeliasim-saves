#!/usr/bin/env python3
# =============================================================================
#  Leitura das posições dos poops e das bananas da cena a_star_banana.ttt
#
#  Nada de coordenada fixa no código: os objetos são descobertos percorrendo
#  a hierarquia da cena (/poops/poop[i] e /bananas/Banana[i]) e cada posição
#  vem de sim.getObjectPosition no referencial do mundo.
#
#  Como usar (com o CoppeliaSim aberto na cena):
#      /usr/bin/python3 scene_objects.py
#      /usr/bin/python3 scene_objects.py --json posicoes.json
#
#  Dependência: pip install coppeliasim-zmqremoteapi-client
# =============================================================================

import argparse
import json

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Caminhos dos grupos na hierarquia da cena.
POOPS_PATH = '/poops'
BANANAS_PATH = '/bananas'
ROBOT_PATH = '/myRobot'


def get_children(sim, parent_handle):
    """Devolve os handles dos filhos diretos de um objeto, na ordem da cena."""
    children = []
    index = 0
    while True:
        child = sim.getObjectChild(parent_handle, index)
        if child == -1:
            break
        children.append(child)
        index += 1
    return children


def read_group(sim, group_path):
    """Lê nome e posição (mundo) de cada filho do grupo indicado.

    Devolve uma lista de dicionários {'name', 'path', 'position'}. Se o grupo
    não existir na cena, devolve lista vazia.
    """
    try:
        group = sim.getObject(group_path)
    except Exception:
        print(f'[aviso] grupo {group_path} não encontrado na cena')
        return []

    objects = []
    for handle in get_children(sim, group):
        alias = sim.getObjectAlias(handle, 1)      # só o nome do objeto
        path = sim.getObjectAlias(handle, 2)       # caminho completo na cena
        position = sim.getObjectPosition(handle, sim.handle_world)
        objects.append({'name': alias, 'path': path, 'position': position})
    return objects


def read_robot(sim):
    """Posição e orientação do robô, ou None se ele não estiver na cena."""
    try:
        handle = sim.getObject(ROBOT_PATH)
    except Exception:
        return None
    return {
        'name': sim.getObjectAlias(handle, 1),
        'path': ROBOT_PATH,
        'position': sim.getObjectPosition(handle, sim.handle_world),
        'orientation': sim.getObjectOrientation(handle, sim.handle_world),
    }


def print_group(title, objects):
    print(f'\n{title} ({len(objects)})')
    print('-' * 60)
    for obj in objects:
        x, y, z = obj['position']
        print(f'  {obj["path"]:<28} x={x:+.3f}  y={y:+.3f}  z={z:+.3f}')


def main():
    parser = argparse.ArgumentParser(
        description='Lê as posições dos poops e bananas da cena do CoppeliaSim')
    parser.add_argument('--json', metavar='ARQUIVO',
                        help='também salva o resultado neste arquivo JSON')
    args = parser.parse_args()

    client = RemoteAPIClient()
    sim = client.require('sim')

    poops = read_group(sim, POOPS_PATH)
    bananas = read_group(sim, BANANAS_PATH)
    robot = read_robot(sim)

    if robot is not None:
        x, y, z = robot['position']
        yaw = robot['orientation'][2]
        print('\nRobô')
        print('-' * 60)
        print(f'  {ROBOT_PATH:<28} x={x:+.3f}  y={y:+.3f}  z={z:+.3f}  '
              f'yaw={yaw:+.3f} rad')

    print_group('Poops (obstáculos)', poops)
    print_group('Bananas (alvos)', bananas)

    if args.json:
        data = {'robot': robot, 'poops': poops, 'bananas': bananas}
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f'\nSalvo em {args.json}')


if __name__ == '__main__':
    main()
