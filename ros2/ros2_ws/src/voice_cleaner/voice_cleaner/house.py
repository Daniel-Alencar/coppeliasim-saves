r"""A planta da casa: cômodos, paredes e portas, num só lugar.

Este módulo é a fonte única de verdade da casa. Dele saem, sem nenhuma
duplicação de medidas, as quatro coisas que precisam concordar:

    1. o mapa de ocupação do rviz        -> occupancy()
    2. em que cômodo o robô está         -> room_at(x, y)
    3. para onde ir quando alguém diz    -> goal_of('cozinha')
       "vá para a cozinha"
    4. as paredes criadas no CoppeliaSim -> wall_boxes()

Se as paredes do simulador e o mapa do rviz viessem de lugares diferentes, o
robô planejaria num mapa e bateria noutro. É por isso que tudo nasce das mesmas
constantes daqui, e é por isso que este módulo não importa ROS nem CoppeliaSim:
ele é geometria, e tem teste sem nenhum dos dois.

A planta
--------
Cinco metros por cinco, o tamanho da arena da cena herdada (AREA_SIZE = 5.0 em
buildScene.lua). Um corredor atravessa a casa no meio, e os seis cômodos se
abrem para ele — nenhum cômodo dá direto no outro, então todo caminho passa
pelo corredor, como numa casa de verdade:

      y
    +2.5 ┌──────────┬────────┬───────────┐
         │          │        │           │
         │  quarto  │ banh.  │  cozinha  │     y ∈ [+0.55, +2.50]
         │          │        │           │
    +0.55├───┐  ┌───┴─┐  ┌───┴──┐  ┌─────┤     <- portas (vãos de 0,90 m)
         │   corredor                    │     y ∈ [−0.55, +0.55]
    −0.55├─────┐  ┌────┐  ┌───────┐  ┌───┤
         │     │           │          │   │
         │ escritório│  sala  │ garagem   │     y ∈ [−2.50, −0.55]
    −2.5 └───────────┴────────┴──────────┘
        −2.5                            +2.5  x

Os sete cômodos cobrem a casa inteira sem sobra nem vão: todo ponto dentro das
paredes externas pertence a exatamente um deles. É o que faz `room_at` sempre
ter uma resposta, inclusive com o robô parado numa porta.

Os nomes são os de commands.py (PLACES), senão "vá para a cozinha" viraria um
comando para um lugar que a casa não tem. O teste test_house.py trava isso.

As medidas e o robô
-------------------
O robô tem 0,32 m de diâmetro. Com a inflação do planejador (raio + folga,
0,22 m por padrão) uma porta de 0,90 m deixa 0,46 m de passagem livre e o
corredor deixa 0,58 m — folgado para a célula de 0,05 m do mapa. Apertar mais a
casa fecha as portas para o A*, e o sintoma é "sem caminho até a cozinha" com o
mapa parecendo aberto a olho nu.
"""

import math

#: Lado da casa, em metros. Igual ao AREA_SIZE do buildScene.lua da cena.
SIZE = 5.0

#: Meia-largura do corredor central, em metros (corredor de 1,10 m).
CORRIDOR_HALF = 0.55

#: Espessura das paredes, em metros. Vale no mapa e no simulador.
WALL = 0.08

#: Altura das paredes no CoppeliaSim, em metros. Não entra no mapa 2D.
WALL_HEIGHT = 0.5

#: Largura do vão de cada porta, em metros.
DOOR = 0.90

#: Folga entre a parede externa e a borda do mapa, para a parede caber inteira.
MAP_MARGIN = 0.10

_HALF = SIZE / 2.0

#: Divisórias verticais da fileira ao norte do corredor, em x.
_NORTH_SPLITS = (-0.80, 0.60)

#: Divisórias verticais da fileira ao sul do corredor, em x.
_SOUTH_SPLITS = (-0.90, 1.00)


class Room:
    """Um cômodo: o nome que se diz, e o retângulo que ele ocupa."""

    def __init__(self, name, x0, y0, x1, y1):
        self.name = name
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1

    @property
    def center(self):
        return ((self.x0 + self.x1) / 2.0, (self.y0 + self.y1) / 2.0)

    def contains(self, x, y):
        """Diz se o ponto está neste cômodo.

        Fechado embaixo e aberto em cima, de propósito: os cômodos se tocam nas
        divisórias, e este critério faz o ponto da fronteira cair em exatamente
        um deles em vez de em dois.
        """
        return self.x0 <= x < self.x1 and self.y0 <= y < self.y1

    def __repr__(self):
        return (f'Room({self.name!r}, [{self.x0:+.2f}, {self.x1:+.2f}] x '
                f'[{self.y0:+.2f}, {self.y1:+.2f}])')


def _build_rooms():
    """Os sete cômodos, ladrilhando a casa de parede a parede."""
    rooms = []
    # Fileira ao norte do corredor.
    north = (-_HALF,) + _NORTH_SPLITS + (_HALF,)
    for name, x0, x1 in zip(('quarto', 'banheiro', 'cozinha'), north, north[1:]):
        rooms.append(Room(name, x0, CORRIDOR_HALF, x1, _HALF))
    # O corredor, de ponta a ponta.
    rooms.append(Room('corredor', -_HALF, -CORRIDOR_HALF, _HALF, CORRIDOR_HALF))
    # Fileira ao sul.
    south = (-_HALF,) + _SOUTH_SPLITS + (_HALF,)
    for name, x0, x1 in zip(('escritório', 'sala', 'garagem'), south, south[1:]):
        rooms.append(Room(name, x0, -_HALF, x1, -CORRIDOR_HALF))
    return tuple(rooms)


#: Todos os cômodos da casa.
ROOMS = _build_rooms()

#: Por nome, para a busca do comando de voz.
BY_NAME = {room.name: room for room in ROOMS}

#: Onde o robô começa e para onde ele volta em "voltar para a base".
#: No meio da sala, que é o cômodo de onde a demonstração costuma partir.
BASE = BY_NAME['sala'].center


# =============================================================================
#  Paredes
# =============================================================================

class Segment:
    """Um trecho de parede, como um retângulo de espessura WALL.

    Guardado como segmento (dois pontos) e não como caixa porque é assim que a
    planta é natural de escrever; a caixa sai em `box`, que é o que o
    CoppeliaSim e o rasterizador querem.
    """

    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1

    @property
    def box(self):
        """(cx, cy, largura_x, largura_y) do retângulo que a parede ocupa.

        A espessura é somada só no eixo perpendicular ao trecho: engordar
        também no sentido do comprimento faria cada parede avançar WALL/2 sobre
        as portas das pontas, estreitando de 0,90 m para 0,82 m um vão que a
        planta diz ter 0,90. Os cantos fecham sem essa folga porque as paredes
        externas vão de ponta a ponta e se cruzam.
        """
        xa, xb = min(self.x0, self.x1), max(self.x0, self.x1)
        ya, yb = min(self.y0, self.y1), max(self.y0, self.y1)
        return ((xa + xb) / 2.0, (ya + yb) / 2.0,
                max(xb - xa, WALL), max(yb - ya, WALL))

    def __repr__(self):
        return (f'Segment(({self.x0:+.2f}, {self.y0:+.2f}) -> '
                f'({self.x1:+.2f}, {self.y1:+.2f}))')


def _carve(a, b, gaps):
    """Divide o intervalo [a, b] nos trechos que sobram fora de `gaps`.

    É o que transforma "parede de ponta a ponta" em "parede com portas": cada
    gap é um par (início, fim) a remover. Sem isto a casa seria uma caixa de
    cômodos lacrados, e o A* não acharia caminho para nenhum deles.
    """
    pieces = [(a, b)]
    for gap_start, gap_end in sorted(gaps):
        resto = []
        for start, end in pieces:
            if gap_end <= start or gap_start >= end:
                resto.append((start, end))      # o gap não toca este trecho
                continue
            if start < gap_start:
                resto.append((start, gap_start))
            if gap_end < end:
                resto.append((gap_end, end))
        pieces = resto
    # Trecho menor que a espessura da parede não é parede: é ruído de medida.
    return [(start, end) for start, end in pieces if end - start > WALL]


def _door_gap(room):
    """O vão da porta do cômodo, em x, centrado na largura dele."""
    cx = room.center[0]
    return (cx - DOOR / 2.0, cx + DOOR / 2.0)


def walls():
    """Todos os trechos de parede da casa, portas já abertas."""
    segments = []

    # Paredes externas: a caixa da casa.
    segments += [
        Segment(-_HALF, -_HALF, _HALF, -_HALF),   # sul
        Segment(-_HALF, _HALF, _HALF, _HALF),     # norte
        Segment(-_HALF, -_HALF, -_HALF, _HALF),   # oeste
        Segment(_HALF, -_HALF, _HALF, _HALF),     # leste
    ]

    # As duas paredes do corredor, com uma porta por cômodo da fileira.
    for y, nomes in ((CORRIDOR_HALF, ('quarto', 'banheiro', 'cozinha')),
                     (-CORRIDOR_HALF, ('escritório', 'sala', 'garagem'))):
        gaps = [_door_gap(BY_NAME[nome]) for nome in nomes]
        for x_start, x_end in _carve(-_HALF, _HALF, gaps):
            segments.append(Segment(x_start, y, x_end, y))

    # Divisórias entre cômodos vizinhos da mesma fileira: sem porta, porque
    # todo caminho de um cômodo a outro passa pelo corredor.
    for x in _NORTH_SPLITS:
        segments.append(Segment(x, CORRIDOR_HALF, x, _HALF))
    for x in _SOUTH_SPLITS:
        segments.append(Segment(x, -_HALF, x, -CORRIDOR_HALF))

    return segments


def wall_boxes():
    """As paredes como caixas (cx, cy, sx, sy, sz), para criar no CoppeliaSim."""
    return [(cx, cy, sx, sy, WALL_HEIGHT)
            for cx, cy, sx, sy in (s.box for s in walls())]


# =============================================================================
#  Onde o robô está
# =============================================================================

def room_at(x, y):
    """O nome do cômodo em (x, y), ou None se o ponto está fora da casa."""
    for room in ROOMS:
        if room.contains(x, y):
            return room.name
    return None


def goal_of(name):
    """O ponto (x, y) para onde ir quando pedirem este cômodo.

    O centro do cômodo. Quem garante que ele está livre depois da inflação é o
    planejador, que procura a célula livre mais próxima — aqui é geometria, não
    navegação.
    """
    room = BY_NAME.get(name)
    return None if room is None else room.center


def room_names():
    """Os nomes dos cômodos, na ordem em que a casa foi escrita."""
    return [room.name for room in ROOMS]


# =============================================================================
#  O mapa de ocupação
# =============================================================================

def bounds():
    """(x_min, x_max, y_min, y_max) do mapa, com a parede externa cabendo."""
    limit = _HALF + MAP_MARGIN
    return (-limit, limit, -limit, limit)


def occupancy(resolution=0.05):
    """O mapa da casa: paredes rasterizadas numa matriz de ocupação.

    Devolve (cells, info), com `cells` em linhas de baixo para cima (a ordem da
    nav_msgs/OccupancyGrid, então o vetor `data` sai sem transposição) e `info`
    trazendo resolução e origem, que é o que a mensagem precisa no cabeçalho.

    Uma célula é parede quando se **sobrepõe** à caixa de algum trecho, não
    quando o centro dela cai dentro. A diferença importa: a parede tem 0,08 m e
    a célula pode ter 0,10, e aí o teste do centro deixa a parede cair entre
    dois centros e desaparecer do mapa — um mapa com parede faltando faz o A*
    planejar atravessando ela. A sobreposição erra para o lado seguro, deixando
    a parede no máximo uma célula mais grossa do que é.

    O encosto não conta como sobreposição (a comparação é estrita), senão cada
    porta perderia uma célula de cada lado sem precisar.
    """
    x_min, x_max, y_min, y_max = bounds()
    cols = int(math.ceil((x_max - x_min) / resolution))
    rows = int(math.ceil((y_max - y_min) / resolution))
    cells = [[0] * cols for _ in range(rows)]

    boxes = [s.box for s in walls()]
    for row in range(rows):
        cell_y0 = y_min + row * resolution
        cell_y1 = cell_y0 + resolution
        for col in range(cols):
            cell_x0 = x_min + col * resolution
            cell_x1 = cell_x0 + resolution
            for cx, cy, sx, sy in boxes:
                if (cell_x0 < cx + sx / 2.0 and cell_x1 > cx - sx / 2.0
                        and cell_y0 < cy + sy / 2.0 and cell_y1 > cy - sy / 2.0):
                    cells[row][col] = 1
                    break
    info = {'resolution': resolution, 'origin': (x_min, y_min),
            'rows': rows, 'cols': cols}
    return cells, info


def render(resolution=0.1, mark=None):
    """Desenha a casa em texto, +y para cima.

    `mark` é um dict (x, y) -> caractere, para marcar pontos no desenho.
    """
    cells, info = occupancy(resolution)
    x_min, y_min = info['origin']
    marks = {}
    for (x, y), char in (mark or {}).items():
        col = int((x - x_min) / resolution)
        row = int((y - y_min) / resolution)
        marks[(row, col)] = char
    lines = []
    for row in range(info['rows'] - 1, -1, -1):
        lines.append(''.join(
            marks.get((row, col), '#' if cells[row][col] else ' ')
            for col in range(info['cols'])))
    return '\n'.join(lines)


def save_pgm(path, resolution=0.05):
    """Grava o mapa como .pgm + .yaml, o par que o nav2_map_server serve.

    O nó publica o mapa direto num tópico, então estes arquivos não são
    necessários para o rviz; eles existem para o mapa poder ser aberto por
    qualquer outra ferramenta de mapa do ROS, e para ser olhado como imagem.
    """
    import os

    cells, info = occupancy(resolution)
    base = os.path.splitext(path)[0]
    rows, cols = info['rows'], info['cols']

    # O .pgm é escrito de cima para baixo, o inverso da ordem das linhas.
    with open(base + '.pgm', 'wb') as arquivo:
        arquivo.write(f'P5\n{cols} {rows}\n255\n'.encode('ascii'))
        for row in range(rows - 1, -1, -1):
            arquivo.write(bytes(0 if cell else 254 for cell in cells[row]))

    x_min, y_min = info['origin']
    with open(base + '.yaml', 'w') as arquivo:
        arquivo.write(
            f'image: {os.path.basename(base)}.pgm\n'
            f'mode: trinary\n'
            f'resolution: {resolution}\n'
            f'origin: [{x_min}, {y_min}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.196\n')
    return base + '.pgm', base + '.yaml'
