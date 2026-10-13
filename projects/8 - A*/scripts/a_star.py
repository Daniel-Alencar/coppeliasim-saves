#!/usr/bin/env python3
# =============================================================================
#  Planejamento de caminho com A* na cena a_star_banana.ttt
#
#  Os poops (mais a margem de inflação) são os obstáculos, e uma banana é o
#  objetivo. O mapa de ocupação vem de grid_map.py, que por sua vez lê as
#  posições da cena pela ZeroMQ Remote API — nenhuma coordenada é fixa no
#  código.
#
#  Busca em grade de 8 vizinhos:
#      g = custo acumulado (1 em linha reta, √2 na diagonal, em células)
#      h = distância octile até o objetivo — admissível para 8 vizinhos, então
#          o caminho devolvido é o de menor custo
#  O desempate prefere o nó de menor h, o que deixa a busca mais direta sem
#  mudar o custo final. Diagonal só é permitida quando as duas células
#  ortogonais vizinhas estão livres, para o robô não "cortar quina" de poop.
#
#  Como usar (com o CoppeliaSim aberto na cena):
#      /usr/bin/python3 a_star.py                          # banana mais próxima
#      /usr/bin/python3 a_star.py --goal '/bananas/Banana[4]'
#      /usr/bin/python3 a_star.py --resolution 0.1 --inflate 0.25
#      /usr/bin/python3 a_star.py --no-diagonal
#      /usr/bin/python3 a_star.py --json caminho.json      # waypoints p/ o ROS 2
# =============================================================================

import argparse
import heapq
import json
import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from grid_map import (INFLATED, POOP, arena_bounds, build_grid, cell_to_world,
                      print_grid, world_to_cell)
from scene_objects import BANANAS_PATH, POOPS_PATH, read_group, read_robot

SQRT2 = math.sqrt(2.0)

# Vizinhança de 8: (dlinha, dcoluna, custo)
NEIGHBORS_8 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]
NEIGHBORS_4 = NEIGHBORS_8[:4]


def is_blocked(grid, row, col):
    """Célula intransponível: poop ou margem de inflação."""
    return grid[row][col] in (POOP, INFLATED)


def heuristic(a, b, diagonal=True):
    """Distância octile (ou de Manhattan, sem diagonal), em células."""
    dr = abs(a[0] - b[0])
    dc = abs(a[1] - b[1])
    if not diagonal:
        return dr + dc
    return (dr + dc) + (SQRT2 - 2.0) * min(dr, dc)


def a_star(grid, start, goal, diagonal=True):
    """Caminho de menor custo entre duas células.

    Devolve (caminho, estatísticas). O caminho é a lista de células de start
    até goal, inclusive, ou None se o objetivo for inalcançável.
    """
    rows, cols = len(grid), len(grid[0])
    neighbors = NEIGHBORS_8 if diagonal else NEIGHBORS_4

    g_score = {start: 0.0}
    came_from = {}
    closed = set()

    h_start = heuristic(start, goal, diagonal)
    # (f, h, célula): o h no meio é o critério de desempate
    open_heap = [(h_start, h_start, start)]
    expanded = 0

    while open_heap:
        f, _, current = heapq.heappop(open_heap)
        if current in closed:
            continue                      # entrada obsoleta deixada no heap
        closed.add(current)
        expanded += 1

        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            stats = {'expanded': expanded, 'cost_cells': g_score[goal],
                     'length_cells': len(path) - 1}
            return path, stats

        row, col = current
        for dr, dc, step in neighbors:
            nr, nc = row + dr, col + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if is_blocked(grid, nr, nc) or (nr, nc) in closed:
                continue
            if dr != 0 and dc != 0:
                # sem cortar quina: as duas ortogonais precisam estar livres
                if is_blocked(grid, row, nc) or is_blocked(grid, nr, col):
                    continue

            tentative = g_score[current] + step
            if tentative < g_score.get((nr, nc), math.inf):
                came_from[(nr, nc)] = current
                g_score[(nr, nc)] = tentative
                h = heuristic((nr, nc), goal, diagonal)
                heapq.heappush(open_heap, (tentative + h, h, (nr, nc)))

    return None, {'expanded': expanded, 'cost_cells': None,
                  'length_cells': None}


def nearest_free_cell(grid, cell, max_radius=6):
    """Célula livre mais próxima de `cell` (ela mesma, se já estiver livre).

    Serve para quando o robô ou a banana caem dentro da margem de inflação —
    acontece com banana encostada num poop ou com inflação generosa.
    """
    row, col = cell
    rows, cols = len(grid), len(grid[0])
    if not is_blocked(grid, row, col):
        return cell

    for radius in range(1, max_radius + 1):
        best, best_distance = None, math.inf
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                rr, cc = row + dr, col + dc
                if not (0 <= rr < rows and 0 <= cc < cols):
                    continue
                if is_blocked(grid, rr, cc):
                    continue
                distance = math.hypot(dr, dc)
                if distance < best_distance:
                    best, best_distance = (rr, cc), distance
        if best is not None:
            return best
    return None


def simplify(path):
    """Tira as células intermediárias dos trechos retos.

    O que sobra são os pontos onde o caminho muda de direção — os waypoints
    que valem a pena mandar para o controlador de posição.
    """
    if not path or len(path) < 3:
        return list(path)

    simplified = [path[0]]
    for previous, current, following in zip(path, path[1:], path[2:]):
        before = (current[0] - previous[0], current[1] - previous[1])
        after = (following[0] - current[0], following[1] - current[1])
        if before != after:
            simplified.append(current)
    simplified.append(path[-1])
    return simplified


def choose_goal(bananas, info, grid, robot_cell, goal_path=None,
                diagonal=True):
    """Escolhe a banana alvo: a pedida por caminho da API, ou a mais próxima.

    Sem --goal, roda o A* para cada banana e fica com a de menor custo real —
    "mais próxima" aqui é pelo caminho que desvia dos poops, não em linha reta.
    """
    if goal_path:
        for banana in bananas:
            if banana['path'] == goal_path or banana['name'] == goal_path:
                return banana
        raise SystemExit(f'banana {goal_path!r} não encontrada na cena. '
                         f'Opções: {", ".join(b["path"] for b in bananas)}')

    best, best_cost = None, math.inf
    for banana in bananas:
        cell = world_to_cell(info, banana['position'])
        if cell is None:
            continue
        cell = nearest_free_cell(grid, cell)
        if cell is None:
            continue
        _, stats = a_star(grid, robot_cell, cell, diagonal)
        if stats['cost_cells'] is not None and stats['cost_cells'] < best_cost:
            best, best_cost = banana, stats['cost_cells']

    if best is None:
        raise SystemExit('nenhuma banana alcançável a partir da posição do robô')
    return best


def main():
    parser = argparse.ArgumentParser(
        description='Planeja um caminho A* do robô até uma banana, '
                    'desviando dos poops')
    parser.add_argument('--goal', metavar='CAMINHO_API',
                        help="banana alvo, ex.: '/bananas/Banana[4]' "
                             '(padrão: a mais próxima pelo caminho)')
    parser.add_argument('--resolution', type=float, default=0.25,
                        help='lado da célula em metros (padrão: 0.25)')
    parser.add_argument('--inflate', type=float, default=0.25,
                        help='margem de segurança ao redor dos poops, em '
                             'metros (padrão: 0.25 = raio do robô + folga)')
    parser.add_argument('--no-diagonal', action='store_true',
                        help='usa vizinhança de 4 em vez de 8')
    parser.add_argument('--json', metavar='ARQUIVO',
                        help='salva caminho e waypoints neste arquivo JSON')
    args = parser.parse_args()

    diagonal = not args.no_diagonal

    client = RemoteAPIClient()
    sim = client.require('sim')

    poops = read_group(sim, POOPS_PATH)
    bananas = read_group(sim, BANANAS_PATH)
    robot = read_robot(sim)
    if robot is None:
        raise SystemExit('robô não encontrado na cena')
    if not bananas:
        raise SystemExit('nenhuma banana na cena')

    points = [o['position'] for o in poops + bananas] + [robot['position']]
    bounds = arena_bounds(sim, points)
    grid, info = build_grid(poops, bananas, robot, bounds,
                            args.resolution, args.inflate)

    # --- células de partida e de chegada ------------------------------------
    start = world_to_cell(info, robot['position'])
    if start is None:
        raise SystemExit('robô fora dos limites do mapa')
    start_free = nearest_free_cell(grid, start)
    if start_free is None:
        raise SystemExit('o robô está cercado de obstáculos inflados; '
                         'diminua --inflate')
    if start_free != start:
        print(f'[aviso] robô dentro da margem de inflação em {start}; '
              f'partindo da célula livre mais próxima {start_free}')

    banana = choose_goal(bananas, info, grid, start_free, args.goal,
                         diagonal)
    goal = world_to_cell(info, banana['position'])
    if goal is None:
        raise SystemExit(f'{banana["path"]} está fora dos limites do mapa')
    goal_free = nearest_free_cell(grid, goal)
    if goal_free is None:
        raise SystemExit(f'{banana["path"]} está cercada de obstáculos; '
                         'escolha outra banana ou diminua --inflate')
    if goal_free != goal:
        print(f'[aviso] {banana["path"]} dentro da margem de inflação em '
              f'{goal}; mirando a célula livre mais próxima {goal_free}')

    # --- busca ---------------------------------------------------------------
    path, stats = a_star(grid, start_free, goal_free, diagonal)

    print(f'\nAlvo:  {banana["path"]}  '
          f'({banana["position"][0]:+.3f}, {banana["position"][1]:+.3f})')
    print(f'Robô:  ({robot["position"][0]:+.3f}, {robot["position"][1]:+.3f})  '
          f'célula {start_free} -> célula {goal_free}')
    print(f'Vizinhança: {"8 (diagonais)" if diagonal else "4 (sem diagonais)"}'
          f'  |  nós expandidos: {stats["expanded"]}')

    if path is None:
        print('\nSem caminho: o objetivo é inalcançável com este mapa.')
        print('Tente uma resolução mais fina, uma inflação menor, '
              'ou outra banana.')
        print_grid(grid, info)
        raise SystemExit(1)

    waypoints = simplify(path)
    world_path = [cell_to_world(info, r, c) for r, c in path]
    world_waypoints = [cell_to_world(info, r, c) for r, c in waypoints]
    # custo em células -> metros
    distance = stats['cost_cells'] * info['resolution']

    print(f'Caminho: {len(path)} células, {distance:.2f} m, '
          f'{len(waypoints)} waypoints\n')

    print_grid(grid, info, path)

    print('\nWaypoints (x, y) em metros, no referencial do mundo:')
    for i, (x, y) in enumerate(world_waypoints):
        print(f'  {i:>2}: ({x:+.3f}, {y:+.3f})')

    if args.json:
        data = {
            'info': info,
            'goal': banana,
            'robot': robot,
            'start_cell': list(start_free),
            'goal_cell': list(goal_free),
            'diagonal': diagonal,
            'stats': stats,
            'distance_m': distance,
            'path_cells': [list(c) for c in path],
            'path_world': [list(p) for p in world_path],
            'waypoints_cells': [list(c) for c in waypoints],
            'waypoints_world': [list(p) for p in world_waypoints],
        }
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print(f'\nSalvo em {args.json}')


if __name__ == '__main__':
    main()
