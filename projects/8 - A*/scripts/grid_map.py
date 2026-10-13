#!/usr/bin/env python3
# =============================================================================
#  Mapa de ocupação (matriz) da cena a_star_banana.ttt
#
#  Pega as posições com scene_objects.py (ZeroMQ Remote API), descobre os
#  limites da arena pelas paredes e discretiza tudo numa matriz 2D:
#
#      0 = livre        '.'
#      1 = poop         '#'   (obstáculo)
#      2 = inflação     'o'   (margem de segurança ao redor dos poops)
#      3 = banana       'B'   (alvo)
#      4 = robô         'R'
#
#  A célula (linha, coluna) cobre o quadrado de lado RESOLUTION cujo canto
#  inferior esquerdo é (x_min + col*res, y_min + lin*res): a linha cresce com
#  y e a coluna cresce com x. Na impressão as linhas saem invertidas, para a
#  matriz aparecer com a mesma orientação da vista de cima do simulador
#  (+y para cima, +x para a direita).
#
#  Como usar (com o CoppeliaSim aberto na cena):
#      /usr/bin/python3 grid_map.py
#      /usr/bin/python3 grid_map.py --resolution 0.1 --inflate 0.25
#      /usr/bin/python3 grid_map.py --json mapa.json
# =============================================================================

import argparse
import json
import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from scene_objects import read_group, read_robot, POOPS_PATH, BANANAS_PATH

WALLS_PATH = '/20cmHighWall100cm'

FREE, POOP, INFLATED, BANANA, ROBOT = 0, 1, 2, 3, 4
SYMBOLS = {FREE: '.', POOP: '#', INFLATED: 'o', BANANA: 'B', ROBOT: 'R'}


def arena_bounds(sim, points, margin=0.3):
    """Limites (x_min, x_max, y_min, y_max) da área a mapear.

    Parte dos centros das paredes da cena, quando elas existem, e estica a
    caixa para conter todos os objetos com uma margem — assim nada fica de
    fora da matriz, mesmo um objeto encostado na parede.
    """
    box = None
    try:
        walls = sim.getObject(WALLS_PATH)
        xs, ys = [], []
        for handle in sim.getObjectsInTree(walls, sim.handle_all, 0):
            x, y, _ = sim.getObjectPosition(handle, sim.handle_world)
            xs.append(x)
            ys.append(y)
        if xs:
            box = (min(xs), max(xs), min(ys), max(ys))
    except Exception:
        pass

    if points:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        objects_box = (min(xs) - margin, max(xs) + margin,
                       min(ys) - margin, max(ys) + margin)
        if box is None:
            box = objects_box
        else:
            box = (min(box[0], objects_box[0]), max(box[1], objects_box[1]),
                   min(box[2], objects_box[2]), max(box[3], objects_box[3]))

    return box


def world_to_cell(info, position):
    """Posição do mundo -> (linha, coluna), ou None se cair fora do mapa."""
    col = int(math.floor((position[0] - info['x_min']) / info['resolution']))
    row = int(math.floor((position[1] - info['y_min']) / info['resolution']))
    if 0 <= row < info['rows'] and 0 <= col < info['cols']:
        return row, col
    return None


def cell_to_world(info, row, col):
    """(linha, coluna) -> (x, y) do centro da célula."""
    x = info['x_min'] + (col + 0.5) * info['resolution']
    y = info['y_min'] + (row + 0.5) * info['resolution']
    return x, y


def build_grid(poops, bananas, robot, bounds, resolution, inflate=0.0):
    """Monta a matriz de ocupação. Devolve (grid, info)."""
    x_min, x_max, y_min, y_max = bounds
    cols = max(1, int(math.ceil((x_max - x_min) / resolution)))
    rows = max(1, int(math.ceil((y_max - y_min) / resolution)))

    grid = [[FREE] * cols for _ in range(rows)]

    info = {
        'rows': rows, 'cols': cols, 'resolution': resolution,
        'inflate': inflate,
        'x_min': x_min, 'x_max': x_max, 'y_min': y_min, 'y_max': y_max,
        'legend': {str(v): k for k, v in
                   zip(['free', 'poop', 'inflated', 'banana', 'robot'],
                       [FREE, POOP, INFLATED, BANANA, ROBOT])},
    }

    def to_cell(position):
        return world_to_cell(info, position)

    # Obstáculos primeiro, depois a inflação, depois alvos e robô: assim um
    # poop nunca é apagado por uma margem e o robô fica sempre visível.
    for poop in poops:
        cell = to_cell(poop['position'])
        if cell:
            grid[cell[0]][cell[1]] = POOP

    if inflate > 0:
        radius = int(math.ceil(inflate / resolution))
        occupied = [(r, c) for r in range(rows) for c in range(cols)
                    if grid[r][c] == POOP]
        for r, c in occupied:
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < rows and 0 <= cc < cols and grid[rr][cc] == FREE:
                        # distância euclidiana em células, para a margem ficar
                        # redonda em vez de quadrada
                        if math.hypot(dr, dc) * resolution <= inflate:
                            grid[rr][cc] = INFLATED

    for banana in bananas:
        cell = to_cell(banana['position'])
        if cell:
            grid[cell[0]][cell[1]] = BANANA

    if robot is not None:
        cell = to_cell(robot['position'])
        if cell:
            grid[cell[0]][cell[1]] = ROBOT

    return grid, info


def print_grid(grid, info, path=None):
    """Imprime a matriz. Se `path` (lista de células) for dado, o caminho
    aparece como '*', sem apagar o robô nem a banana alvo."""
    rows, cols = info['rows'], info['cols']
    path_cells = set(path or [])
    width = len(str(rows - 1))

    print(f"\nMapa {rows}x{cols}  |  resolução {info['resolution']} m/célula  "
          f"|  inflação {info['inflate']} m")
    print(f"x ∈ [{info['x_min']:+.2f}, {info['x_max']:+.2f}]  "
          f"y ∈ [{info['y_min']:+.2f}, {info['y_max']:+.2f}]")
    legend = 'legenda: . livre   # poop   o margem   B banana   R robô'
    print(legend + ('   * caminho\n' if path_cells else '\n'))

    for row in range(rows - 1, -1, -1):          # +y para cima
        line = ' '.join(
            '*' if (row, col) in path_cells and grid[row][col] in (FREE, INFLATED)
            else SYMBOLS[grid[row][col]]
            for col in range(cols))
        print(f'{row:>{width}} | {line}')

    # régua das colunas (só a dezena, para não poluir)
    print(' ' * (width + 2) + '-' * (cols * 2 - 1))
    print(' ' * (width + 3) + ' '.join(str(c % 10) for c in range(cols)))


def main():
    parser = argparse.ArgumentParser(
        description='Monta a matriz de ocupação da cena do CoppeliaSim')
    parser.add_argument('--resolution', type=float, default=0.25,
                        help='lado da célula em metros (padrão: 0.25)')
    parser.add_argument('--inflate', type=float, default=0.0,
                        help='margem de segurança ao redor dos poops, em metros')
    parser.add_argument('--json', metavar='ARQUIVO',
                        help='salva a matriz e os metadados neste arquivo JSON')
    args = parser.parse_args()

    client = RemoteAPIClient()
    sim = client.require('sim')

    poops = read_group(sim, POOPS_PATH)
    bananas = read_group(sim, BANANAS_PATH)
    robot = read_robot(sim)

    points = [o['position'] for o in poops + bananas]
    if robot is not None:
        points.append(robot['position'])

    bounds = arena_bounds(sim, points)
    grid, info = build_grid(poops, bananas, robot, bounds,
                            args.resolution, args.inflate)

    print_grid(grid, info)
    print(f'\n{len(poops)} poops, {len(bananas)} bananas, '
          f'robô {"encontrado" if robot else "não encontrado"}')

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump({'info': info, 'grid': grid}, f, indent=2)
        print(f'Salvo em {args.json}')


if __name__ == '__main__':
    main()
