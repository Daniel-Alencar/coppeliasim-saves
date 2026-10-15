"""Mapa de ocupação da arena: discretização, inflação e conversões.

Este módulo não depende do ROS nem do CoppeliaSim: recebe listas de pontos
(x, y) em metros e devolve uma matriz. Quem lê a cena é a ponte, e quem monta a
nav_msgs/OccupancyGrid a partir daqui é o planner.

Convenções
----------
- A célula (linha, coluna) cobre o quadrado de lado `resolution` cujo canto
  inferior esquerdo é (x_min + coluna*res, y_min + linha*res): a linha cresce
  com y e a coluna cresce com x. É a mesma ordem da nav_msgs/OccupancyGrid,
  então o vetor `data` sai daqui sem transposição.
- Valores: FREE livre, OCCUPIED obstáculo (um poop), INFLATED margem de
  segurança ao redor de um obstáculo. Para o A*, OCCUPIED e INFLATED são a
  mesma coisa — intransponíveis; a distinção existe para o desenho do mapa e
  para explicar o que aconteceu.
"""

import math

FREE, OCCUPIED, INFLATED = 0, 1, 2
SYMBOLS = {FREE: '.', OCCUPIED: '#', INFLATED: 'o'}


def bounds_from_points(points, margin=0.4):
    """Retângulo que contém todos os pontos, com uma folga em volta.

    A arena não é fixada no código: ela sai de onde estão os objetos da cena
    (poops, bananas e robô) mais a margem. É o bastante para esta sala, em que
    os objetos chegam perto das paredes; `Grid` ainda bloqueia a borda da
    matriz, então o caminho nunca sai da área mapeada.
    """
    if not points:
        raise ValueError('sem pontos para delimitar a arena')
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    return (min(xs) - margin, max(xs) + margin,
            min(ys) - margin, max(ys) + margin)


class Grid:
    """Matriz de ocupação com os obstáculos já inflados.

    obstacles          pontos (x, y) dos obstáculos, em metros
    bounds             (x_min, x_max, y_min, y_max) da área a mapear
    resolution         lado da célula, em metros
    inflation_radius   raio do robô + margem de segurança, em metros
    border             bloqueia a borda da matriz (as paredes da sala)
    """

    def __init__(self, obstacles, bounds, resolution, inflation_radius=0.0,
                 border=True):
        self.x_min, self.x_max, self.y_min, self.y_max = bounds
        self.resolution = float(resolution)
        self.inflation_radius = float(inflation_radius)
        self.cols = max(1, int(math.ceil((self.x_max - self.x_min) / self.resolution)))
        self.rows = max(1, int(math.ceil((self.y_max - self.y_min) / self.resolution)))

        self.cells = [[FREE] * self.cols for _ in range(self.rows)]
        self.obstacle_cells = []
        for point in obstacles:
            cell = self.to_cell(point)
            if cell is not None:
                self.cells[cell[0]][cell[1]] = OCCUPIED
                self.obstacle_cells.append(cell)

        self.inflate(self.inflation_radius)
        if border:
            self.block_border()

    # ------------------------------------------------------------------ mapa

    def inflate(self, radius):
        """Marca como bloqueada toda célula a até `radius` de um obstáculo.

        A distância vira número de células pela resolução, e o disco é
        percorrido uma vez por obstáculo — não uma varredura do mapa inteiro.
        O teste é a distância euclidiana entre os centros das células, para a
        margem ficar redonda em vez de quadrada.
        """
        if radius <= 0.0:
            return
        reach = int(math.ceil(radius / self.resolution))
        for row, col in self.obstacle_cells:
            for drow in range(-reach, reach + 1):
                for dcol in range(-reach, reach + 1):
                    rr, cc = row + drow, col + dcol
                    if not self.inside(rr, cc):
                        continue
                    if self.cells[rr][cc] != FREE:
                        continue
                    if math.hypot(drow, dcol) * self.resolution <= radius:
                        self.cells[rr][cc] = INFLATED

    def block_border(self):
        """Fecha a borda da matriz: é a parede da sala.

        Sem isto o A* é livre para contornar um obstáculo por fora da área
        mapeada, já que fora dela não existe nada marcado.
        """
        border = ([(0, col) for col in range(self.cols)]
                  + [(self.rows - 1, col) for col in range(self.cols)]
                  + [(row, 0) for row in range(self.rows)]
                  + [(row, self.cols - 1) for row in range(self.rows)])
        for row, col in border:
            if self.cells[row][col] == FREE:
                self.cells[row][col] = INFLATED

    # -------------------------------------------------------------- consultas

    def inside(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols

    def blocked(self, row, col):
        """Célula intransponível: obstáculo ou margem de inflação."""
        return self.cells[row][col] != FREE

    def to_cell(self, point):
        """(x, y) em metros -> (linha, coluna), ou None se cair fora do mapa."""
        col = int(math.floor((point[0] - self.x_min) / self.resolution))
        row = int(math.floor((point[1] - self.y_min) / self.resolution))
        return (row, col) if self.inside(row, col) else None

    def to_world(self, row, col):
        """(linha, coluna) -> (x, y) do centro da célula, em metros."""
        return (self.x_min + (col + 0.5) * self.resolution,
                self.y_min + (row + 0.5) * self.resolution)

    def nearest_free(self, cell, max_reach=8):
        """Célula livre mais próxima (a própria, se já estiver livre).

        O robô e a banana podem cair dentro da margem de inflação: a banana
        encostada num poop, o robô passando raspando por um. Nesse caso o A*
        não teria nem de onde sair nem onde chegar; esta busca em anéis
        devolve a célula livre mais próxima para servir de partida ou chegada.
        Devolve None se não houver nenhuma livre por perto.
        """
        row, col = cell
        if not self.blocked(row, col):
            return cell
        for reach in range(1, max_reach + 1):
            best, best_distance = None, math.inf
            for drow in range(-reach, reach + 1):
                for dcol in range(-reach, reach + 1):
                    rr, cc = row + drow, col + dcol
                    if not self.inside(rr, cc) or self.blocked(rr, cc):
                        continue
                    distance = math.hypot(drow, dcol)
                    if distance < best_distance:
                        best, best_distance = (rr, cc), distance
            if best is not None:
                return best
        return None

    # ---------------------------------------------------------------- saídas

    def occupancy_data(self, free=0, occupied=100):
        """O mapa como o vetor `data` de uma nav_msgs/OccupancyGrid.

        Ordem de linhas, igual à da matriz: índice = linha * cols + coluna.
        A inflação sai como 99 (e não 100) só para aparecer num tom diferente
        no rviz; para o planejador os dois são bloqueio.
        """
        data = []
        for row in self.cells:
            for value in row:
                data.append(occupied if value == OCCUPIED
                            else (occupied - 1 if value == INFLATED else free))
        return data

    def render(self, path=None, robot=None, goal=None):
        """Desenho do mapa em texto, com +y para cima como na vista do simulador."""
        marks = {}
        for cell in (path or []):
            marks[tuple(cell)] = '*'
        if goal is not None:
            marks[tuple(goal)] = 'B'
        if robot is not None:
            marks[tuple(robot)] = 'R'

        lines = [
            f'mapa {self.rows}x{self.cols} | {self.resolution:.2f} m/célula | '
            f'inflação {self.inflation_radius:.2f} m',
            f'x ∈ [{self.x_min:+.2f}, {self.x_max:+.2f}]  '
            f'y ∈ [{self.y_min:+.2f}, {self.y_max:+.2f}]',
            'legenda: . livre  # poop  o margem  * caminho  R robô  B alvo',
        ]
        for row in range(self.rows - 1, -1, -1):
            lines.append(''.join(
                marks.get((row, col), SYMBOLS[self.cells[row][col]])
                for col in range(self.cols)))
        return '\n'.join(lines)
