"""Mapa de ocupação para o planejador: inflação das paredes e conversões.

O mapa que o rviz mostra tem as paredes com a espessura que elas têm. O
planejador não pode usar esse mapa cru: o A* move o **centro** do robô de
célula em célula, e um caminho que passa a 1 cm da parede encosta o corpo dele
nela. A inflação resolve isso bloqueando toda célula a menos de
`inflation_radius` de uma parede — assim qualquer caminho que o A* encontre tem
o corpo do robô cabendo.

    mapa cru (paredes)  --->  | Grid |  --->  mapa inflado (o que o A* vê)

Este módulo é Python puro: ele recebe a matriz de ocupação e devolve outra, sem
saber se ela veio da planta de house.py ou de um nav_msgs/OccupancyGrid vindo
de um SLAM. É por isso que o house_navigator planeja no mapa que chega pelo
tópico — o mesmo que você vê no rviz — e não numa cópia própria da planta.

Convenções
----------
- A célula (linha, coluna) cobre o quadrado de lado `resolution` cujo canto
  inferior esquerdo é (x_min + coluna*res, y_min + linha*res): a linha cresce
  com y e a coluna cresce com x. É a ordem da nav_msgs/OccupancyGrid, então o
  vetor `data` entra e sai daqui sem transposição.
- FREE livre, WALL parede do mapa, INFLATED a margem em volta dela. Para o A*,
  WALL e INFLATED são a mesma coisa; a distinção existe para o desenho e para
  explicar por que um caminho não existe.

É o grid.py do pacote a_star_banana, com uma mudança de origem: lá o mapa era
construído de uma lista de obstáculos pontuais (os poops), aqui ele vem de uma
matriz de ocupação já pronta.
"""

import math

FREE, WALL, INFLATED = 0, 1, 2
SYMBOLS = {FREE: '.', WALL: '#', INFLATED: 'o'}


class Grid:
    """Matriz de ocupação com as paredes já infladas.

    cells              matriz de 0/1 vinda do mapa, linhas crescendo com y
    origin             (x_min, y_min) do canto da célula (0, 0), em metros
    resolution         lado da célula, em metros
    inflation_radius   raio do robô + folga de segurança, em metros
    """

    def __init__(self, cells, origin, resolution, inflation_radius=0.0):
        self.rows = len(cells)
        self.cols = len(cells[0]) if self.rows else 0
        self.x_min, self.y_min = origin
        self.resolution = float(resolution)
        self.inflation_radius = float(inflation_radius)
        self.x_max = self.x_min + self.cols * self.resolution
        self.y_max = self.y_min + self.rows * self.resolution

        self.cells = [[WALL if value else FREE for value in row]
                      for row in cells]
        self.wall_cells = [(row, col)
                           for row in range(self.rows)
                           for col in range(self.cols)
                           if self.cells[row][col] == WALL]
        self.inflate(self.inflation_radius)

    # ------------------------------------------------------------------ mapa

    def inflate(self, radius):
        """Bloqueia toda célula a até `radius` de uma parede.

        O disco é percorrido uma vez por célula de parede, não o mapa inteiro
        por célula. A distância é euclidiana entre centros, para a margem ficar
        redonda em vez de quadrada — numa porta, a margem quadrada tiraria mais
        passagem do que o robô realmente ocupa.
        """
        if radius <= 0.0:
            return
        reach = int(math.ceil(radius / self.resolution))
        disc = [(drow, dcol)
                for drow in range(-reach, reach + 1)
                for dcol in range(-reach, reach + 1)
                if math.hypot(drow, dcol) * self.resolution <= radius]
        for row, col in self.wall_cells:
            for drow, dcol in disc:
                rr, cc = row + drow, col + dcol
                if self.inside(rr, cc) and self.cells[rr][cc] == FREE:
                    self.cells[rr][cc] = INFLATED

    # -------------------------------------------------------------- consultas

    def inside(self, row, col):
        return 0 <= row < self.rows and 0 <= col < self.cols

    def blocked(self, row, col):
        """Célula intransponível: parede ou margem de inflação."""
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

    def nearest_free(self, cell, max_reach=12):
        """Célula livre mais próxima (a própria, se já estiver livre).

        Serve para as duas pontas do caminho. O centro geométrico de um cômodo
        pequeno pode cair dentro da margem de inflação, e o robô, encostado
        numa parede, também: nos dois casos o A* não teria de onde sair nem
        onde chegar. Devolve None se não houver célula livre por perto.
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

    def render(self, path=None, robot=None, goal=None):
        """Desenho do mapa em texto, +y para cima como na vista do simulador."""
        marks = {}
        for cell in (path or []):
            marks[tuple(cell)] = '*'
        if goal is not None:
            marks[tuple(goal)] = 'G'
        if robot is not None:
            marks[tuple(robot)] = 'R'

        lines = [
            f'mapa {self.rows}x{self.cols} | {self.resolution:.2f} m/célula | '
            f'inflação {self.inflation_radius:.2f} m',
            'legenda: . livre  # parede  o margem  * caminho  R robô  G alvo',
        ]
        for row in range(self.rows - 1, -1, -1):
            lines.append(''.join(
                marks.get((row, col), SYMBOLS[self.cells[row][col]])
                for col in range(self.cols)))
        return '\n'.join(lines)
