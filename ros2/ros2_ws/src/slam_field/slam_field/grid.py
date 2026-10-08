"""O mapa do SLAM Toolbox visto pelo controlador: obstáculos, inflação e repulsão.

Este módulo só usa numpy: não depende do ROS. O navigator entrega a ele os
campos da nav_msgs/OccupancyGrid e recebe de volta forças e consultas.

Convenções da OccupancyGrid
---------------------------
- data é um vetor em ordem de linhas: a célula (linha, coluna) está no índice
  linha * width + coluna. A linha cresce com o y do mapa e a coluna com o x.
- origin é o canto inferior esquerdo da célula (0, 0), no referencial map. O
  SLAM Toolbox publica a origem sem rotação, então o centro da célula é

      x = origin.x + (coluna + 0,5) * resolution
      y = origin.y + (linha  + 0,5) * resolution

- Valores: -1 desconhecido, 0 livre, 100 ocupado. O SLAM Toolbox só publica
  esses três, mas a regra aqui aceita qualquer valor de 0 a 100: a partir de
  occupied_threshold a célula é obstáculo.
"""

import math

import numpy as np

UNKNOWN = -1


def disk_offsets(radius_cells):
    """Deslocamentos (dl, dc) de todas as células a até radius_cells do centro."""
    r = int(math.ceil(radius_cells))
    offsets = []
    for dl in range(-r, r + 1):
        for dc in range(-r, r + 1):
            if dl * dl + dc * dc <= radius_cells * radius_cells:
                offsets.append((dl, dc))
    return offsets


def dilate(mask, radius_cells):
    """Expande as células True de mask por um disco de raio radius_cells.

    É a inflação: cada célula ocupada marca como bloqueadas todas as vizinhas
    a até radius_cells dela. Feita por deslocamentos do vetor inteiro (um OU
    por célula do disco), sem laço em Python sobre as células do mapa: um disco
    de 5 células são 81 deslocamentos, alguns milissegundos num mapa 400x400.
    """
    if radius_cells <= 0 or not mask.any():
        return mask.copy()
    h, w = mask.shape
    r = int(math.ceil(radius_cells))
    padded = np.zeros((h + 2 * r, w + 2 * r), dtype=bool)
    padded[r:r + h, r:r + w] = mask
    out = np.zeros_like(mask)
    for dl, dc in disk_offsets(radius_cells):
        out |= padded[r + dl:r + dl + h, r + dc:r + dc + w]
    return out


class InflatedMap:
    """Uma OccupancyGrid já classificada e inflada.

    occupied  células com ocupação >= occupied_threshold (os obstáculos)
    unknown   células nunca observadas (-1)
    blocked   occupied inflado pelo raio do robô + margem; com
              unknown_is_obstacle, as desconhecidas entram aqui também
    """

    def __init__(self, data, width, height, resolution, origin_x, origin_y,
                 inflation_radius, occupied_threshold=65,
                 unknown_is_obstacle=False):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.inflation_radius = float(inflation_radius)

        grid = np.asarray(data, dtype=np.int16).reshape(self.height, self.width)
        self.occupied = grid >= occupied_threshold
        self.unknown = grid == UNKNOWN
        self.free = ~self.occupied & ~self.unknown

        # Raio em células: a distância é medida entre centros de célula, então
        # arredonda para cima. Melhor bloquear meia célula a mais do que deixar
        # o centro do robô chegar perto demais da parede.
        self.inflation_cells = math.ceil(self.inflation_radius / self.resolution)
        obstacles = self.occupied | self.unknown if unknown_is_obstacle else self.occupied
        self.blocked = dilate(obstacles, self.inflation_cells)

    # ------------------------------------------------------------------
    #  Índices <-> posições
    # ------------------------------------------------------------------

    def to_cell(self, x, y):
        """(linha, coluna) da célula que contém o ponto (x, y) do mapa."""
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return row, col

    def inside(self, row, col):
        return 0 <= row < self.height and 0 <= col < self.width

    def cell_center(self, row, col):
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def occupied_points(self):
        """Centros (x, y) de todas as células ocupadas: os obstáculos do mapa."""
        rows, cols = np.nonzero(self.occupied)
        xs = self.origin_x + (cols + 0.5) * self.resolution
        ys = self.origin_y + (rows + 0.5) * self.resolution
        return np.column_stack((xs, ys))

    def is_blocked(self, x, y):
        """True se (x, y) está dentro da região inflada. Fora do mapa é livre."""
        row, col = self.to_cell(x, y)
        return self.inside(row, col) and bool(self.blocked[row, col])

    def state_at(self, x, y):
        """'occupied', 'blocked', 'free', 'unknown' ou 'outside' no ponto."""
        row, col = self.to_cell(x, y)
        if not self.inside(row, col):
            return 'outside'
        if self.occupied[row, col]:
            return 'occupied'
        if self.blocked[row, col]:
            return 'blocked'
        return 'unknown' if self.unknown[row, col] else 'free'

    # ------------------------------------------------------------------
    #  Vizinhança
    # ------------------------------------------------------------------

    def _window(self, mask, x, y, radius):
        """Pontos de mask a até radius de (x, y): (dx, dy, d) do ponto ao robô.

        Só a janela quadrada em volta do robô é examinada; o resto do mapa nem
        é lido. dx, dy apontam da célula para o robô.
        """
        row, col = self.to_cell(x, y)
        r = int(math.ceil(radius / self.resolution)) + 1
        l0, l1 = max(0, row - r), min(self.height, row + r + 1)
        c0, c1 = max(0, col - r), min(self.width, col + r + 1)
        if l0 >= l1 or c0 >= c1:
            empty = np.zeros(0)
            return empty, empty, empty
        rows, cols = np.nonzero(mask[l0:l1, c0:c1])
        px = self.origin_x + (cols + c0 + 0.5) * self.resolution
        py = self.origin_y + (rows + l0 + 0.5) * self.resolution
        dx, dy = x - px, y - py
        d = np.hypot(dx, dy)
        near = d <= radius
        return dx[near], dy[near], d[near]

    def boundary_points(self, x, y, radius, sectors):
        """Ponto bloqueado mais próximo em cada setor angular em volta do robô.

        Devolve uma lista de (dx, dy, d): vetor do ponto ao robô e a distância.
        No máximo um ponto por setor, e só dentro de radius.

        É isto que torna a repulsão independente da resolução e do tamanho do
        obstáculo: uma parede vista por 40 células ou por 400 continua dando um
        único ponto por setor — o mais próximo, que é a borda da região
        inflada voltada para o robô.
        """
        dx, dy, d = self._window(self.blocked, x, y, radius)
        if d.size == 0:
            return []
        # Ângulo do robô visto do ponto bloqueado; o setor é o do obstáculo
        # visto do robô (ângulo oposto), mas basta ser consistente.
        sector = ((np.arctan2(-dy, -dx) + math.pi) / (2.0 * math.pi) * sectors)
        sector = np.clip(sector.astype(int), 0, sectors - 1)
        points = []
        for s in np.unique(sector):
            idx = np.nonzero(sector == s)[0]
            k = idx[np.argmin(d[idx])]
            points.append((float(dx[k]), float(dy[k]), float(d[k])))
        return points

    def nearest_occupied(self, x, y, radius):
        """(dx, dy, d) da célula ocupada mais próxima, ou None se não houver."""
        dx, dy, d = self._window(self.occupied, x, y, radius)
        if d.size == 0:
            return None
        k = int(np.argmin(d))
        return float(dx[k]), float(dy[k]), float(d[k])

    def clearance(self, x, y, radius=2.0):
        """Distância do ponto à célula ocupada mais próxima (até radius)."""
        nearest = self.nearest_occupied(x, y, radius)
        return radius if nearest is None else nearest[2]

    def segment_blocked_fraction(self, a, b, step=None):
        """Fração do segmento a-b que passa por células bloqueadas."""
        step = step or self.resolution
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(length / step) + 1)
        hits = 0
        for i in range(n):
            t = i / (n - 1)
            if self.is_blocked(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])):
                hits += 1
        return hits / n


def repulsive_force(grid, x, y, influence, sectors, k_rep, d_min, escape_gain):
    """Repulsão das bordas infladas na vizinhança do robô.

    Para cada setor com borda a uma distância d < influence, a forma clássica
    (Khatib) do campo potencial:

        F = k_rep * (1/d - 1/influence) / d²   apontando da borda para o robô

    Como a inflação já descontou o raio do robô e a margem, d é a folga do
    centro do robô até a região proibida; d = 0 é encostar na margem.

    Se o centro do robô já está dentro da região inflada (a odometria pulou, o
    mapa mudou, um obstáculo apareceu), as bordas o cercam e a soma perde o
    sentido. Nesse caso a força é uma fuga: escape_gain apontando para longe
    da célula ocupada mais próxima.

    Devolve (fx, fy, inside, n_pontos).
    """
    if grid.is_blocked(x, y):
        nearest = grid.nearest_occupied(x, y, grid.inflation_radius + influence)
        if nearest is None:
            return 0.0, 0.0, True, 0
        dx, dy, d = nearest
        d = max(d, 1e-6)
        return escape_gain * dx / d, escape_gain * dy / d, True, 1

    fx = fy = 0.0
    points = grid.boundary_points(x, y, influence, sectors)
    for dx, dy, d in points:
        dist = max(d, d_min)
        magnitude = k_rep * (1.0 / dist - 1.0 / influence) / (dist * dist)
        fx += magnitude * dx / max(d, 1e-6)
        fy += magnitude * dy / max(d, 1e-6)
    return fx, fy, False, len(points)
