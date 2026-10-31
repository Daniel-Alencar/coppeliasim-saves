"""O algoritmo A* sobre a matriz de ocupação, e nada além disso.

Busca de menor custo com vizinhança de 8 (ou de 4, se as diagonais forem
desligadas):

    g = custo acumulado, em células: 1 na reta, √2 na diagonal
    f = g + h, com h = distância octile até o objetivo

A heurística octile é admissível para a vizinhança de 8 (nunca superestima o
custo real), então o caminho devolvido é ótimo para esta grade. O desempate
prefere o nó de menor h, o que deixa a frente de busca mais direta sem mudar o
custo final.

Diagonal só é permitida quando as duas células ortogonais vizinhas estão
livres: sem isso o caminho "corta a quina" de uma parede, passando por um vão
de largura zero que o robô não tem como atravessar. Numa casa com portas isso
não é detalhe: a quina do batente é exatamente onde o corte aconteceria.

Este módulo é puro Python: não conhece ROS nem CoppeliaSim. Veio do pacote
a_star_banana sem mudança de algoritmo; o que muda aqui é só de onde vem o
mapa — paredes de uma planta, em vez de obstáculos espalhados numa arena.
"""

import heapq
import math

SQRT2 = math.sqrt(2.0)

# (dlinha, dcoluna, custo do passo)
NEIGHBORS_8 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]
NEIGHBORS_4 = NEIGHBORS_8[:4]


def heuristic(a, b, diagonal=True):
    """Distância octile (ou de Manhattan, sem diagonais), em células."""
    drow = abs(a[0] - b[0])
    dcol = abs(a[1] - b[1])
    if not diagonal:
        return drow + dcol
    return (drow + dcol) + (SQRT2 - 2.0) * min(drow, dcol)


def search(grid, start, goal, diagonal=True):
    """Caminho de menor custo entre duas células da `grid`.

    Devolve (caminho, estatísticas). O caminho é a lista de células de start
    até goal, inclusive, ou None quando o objetivo é inalcançável — o que
    acontece de verdade nesta casa quando a inflação é grande demais e fecha a
    porta, única passagem entre o corredor e o cômodo.
    """
    neighbors = NEIGHBORS_8 if diagonal else NEIGHBORS_4
    g_score = {start: 0.0}
    came_from = {}
    closed = set()
    expanded = 0

    h_start = heuristic(start, goal, diagonal)
    open_heap = [(h_start, h_start, start)]   # (f, h, célula)

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
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
            return path, {'expanded': expanded,
                          'cost_cells': g_score[goal],
                          'cells': len(path)}

        row, col = current
        for drow, dcol, step in neighbors:
            nrow, ncol = row + drow, col + dcol
            if not grid.inside(nrow, ncol) or (nrow, ncol) in closed:
                continue
            if grid.blocked(nrow, ncol):
                continue
            if drow != 0 and dcol != 0:
                if grid.blocked(row, ncol) or grid.blocked(nrow, col):
                    continue              # não corta quina de obstáculo
            tentative = g_score[current] + step
            if tentative < g_score.get((nrow, ncol), math.inf):
                came_from[(nrow, ncol)] = current
                g_score[(nrow, ncol)] = tentative
                h = heuristic((nrow, ncol), goal, diagonal)
                heapq.heappush(open_heap, (tentative + h, h, (nrow, ncol)))

    return None, {'expanded': expanded, 'cost_cells': None, 'cells': 0}


def simplify(path):
    """Tira as células do meio dos trechos retos.

    O que sobra são os pontos em que o caminho muda de direção: são esses os
    waypoints que valem a pena mandar para o controlador de posição. Mandar
    célula por célula faria o robô parar e realinhar a cada 20 cm.
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
