"""A casa tem teste, e ele roda sem ROS, sem simulador e sem microfone.

O que vale a pena travar aqui não é o desenho da planta — é cada coisa que, se
quebrar, só aparece com o robô batendo numa parede ou dizendo que não sabe
chegar à cozinha:

    - todo cômodo é alcançável a partir da base, com o A* e a inflação de
      verdade (é o teste que pega uma porta estreitada demais)
    - os nomes dos cômodos são os que o comando de voz sabe dizer
    - as paredes não têm furo e a casa não tem vazamento para fora
    - a planta e as caixas mandadas ao CoppeliaSim são a mesma coisa
"""

import math

import pytest

from voice_cleaner import a_star, commands, house
from voice_cleaner.house_grid import Grid

#: A inflação que o house_navigator usa por padrão: raio do robô + folga.
INFLATION = 0.16 + 0.06


@pytest.fixture(scope='module')
def grid():
    """Monta a grade de planejamento como o house_navigator a monta."""
    cells, info = house.occupancy(0.05)
    return Grid(cells, info['origin'], info['resolution'],
                inflation_radius=INFLATION)


# =============================================================================
#  O que o robô precisa conseguir fazer
# =============================================================================

@pytest.mark.parametrize('nome', house.room_names())
def test_todo_comodo_e_alcancavel(grid, nome):
    """Da base até cada cômodo existe caminho, com as paredes infladas.

    É o teste mais importante do módulo. Estreitar uma porta ou aumentar a
    folga de segurança fecha a passagem para o A* sem mudar nada que se note
    olhando o mapa — e o sintoma, em uso, é o robô recusando o comando.
    """
    start = grid.nearest_free(grid.to_cell(house.BASE))
    alvo = grid.nearest_free(grid.to_cell(house.goal_of(nome)))
    assert start is not None, 'a base não tem célula livre'
    assert alvo is not None, f'o centro do(a) {nome} não tem célula livre'
    caminho, _ = a_star.search(grid, start, alvo)
    assert caminho is not None, f'sem caminho da base até o(a) {nome}'


@pytest.mark.parametrize('nome', house.room_names())
def test_o_alvo_cai_dentro_do_comodo(grid, nome):
    """Chegar ao alvo do cômodo é estar nele.

    O alvo é o centro do cômodo, mas o planejador pode deslocá-lo para a
    célula livre mais próxima. Se esse deslocamento atravessasse uma divisória,
    o robô anunciaria "cheguei na cozinha" parado no corredor.
    """
    alvo = grid.nearest_free(grid.to_cell(house.goal_of(nome)))
    x, y = grid.to_world(*alvo)
    assert house.room_at(x, y) == nome


def test_caminho_entre_comodos_passa_pelo_corredor(grid):
    """Ir do quarto à garagem atravessa o corredor, porque não há outra via."""
    origem = grid.nearest_free(grid.to_cell(house.goal_of('quarto')))
    destino = grid.nearest_free(grid.to_cell(house.goal_of('garagem')))
    caminho, _ = a_star.search(grid, origem, destino)
    assert caminho is not None
    visitados = {house.room_at(*grid.to_world(r, c)) for r, c in caminho}
    assert 'corredor' in visitados


# =============================================================================
#  A casa e o vocabulário têm de falar a mesma língua
# =============================================================================

def test_os_comodos_sao_lugares_que_o_robo_entende():
    """Todo cômodo da casa é um lugar que o comando de voz sabe dizer.

    Sem isto a casa teria um cômodo para o qual "vá para lá" é impossível de
    pronunciar, porque o command_parser não extrairia o argumento.
    """
    faladas = set(commands.PLACES.values())
    for nome in house.room_names():
        assert nome in faladas, (
            f'{nome!r} não está em commands.PLACES: ninguém consegue pedir')


def test_lugares_do_vocabulario_existem_na_casa():
    """E o contrário: todo lugar que se pode pedir existe na casa.

    "casa toda" é a exceção e não é um cômodo: o navegador a trata como o
    percurso por todos eles.
    """
    for lugar in commands.PLACES.values():
        if lugar == 'casa toda':
            continue
        assert lugar in house.BY_NAME, (
            f'dá para pedir {lugar!r}, mas a casa não tem esse cômodo')


# =============================================================================
#  Geometria: nada de furo na parede nem vazamento para fora
# =============================================================================

def test_os_comodos_ladrilham_a_casa_sem_sobreposicao():
    """Cada ponto da casa pertence a exatamente um cômodo.

    É o que faz room_at ter sempre uma resposta, inclusive numa porta.
    """
    passo = 0.07
    limite = house.SIZE / 2.0
    y = -limite + passo / 2.0
    while y < limite:
        x = -limite + passo / 2.0
        while x < limite:
            dentro = [r.name for r in house.ROOMS if r.contains(x, y)]
            assert len(dentro) == 1, (
                f'({x:+.2f}, {y:+.2f}) está em {dentro}')
            x += passo
        y += passo


def test_a_casa_nao_vaza_para_fora(grid):
    """Não existe caminho de dentro da casa para fora dela.

    Um furo na parede externa faria o robô "escapar" num replanejamento, e o
    mapa continuaria parecendo fechado na tela.
    """
    dentro = grid.nearest_free(grid.to_cell(house.BASE))
    # Um ponto na faixa de margem, fora das paredes externas.
    fora = grid.to_cell((house.SIZE / 2.0 + house.MAP_MARGIN / 2.0, 0.0))
    assert fora is not None, 'o ponto de fora caiu fora do mapa'
    if grid.blocked(*fora):
        return      # a margem toda está bloqueada: não há nem por onde tentar
    caminho, _ = a_star.search(grid, dentro, fora)
    assert caminho is None, 'há um furo na parede externa'


def test_toda_porta_cabe_o_robo():
    """Cada cômodo tem, na parede do corredor, um vão maior que o robô.

    A conta é a do mapa inflado: o vão de 0,90 m menos a inflação dos dois
    batentes. Se esta margem for a zero, o A* para de achar caminho — e é mais
    fácil entender a falha aqui do que no log do navegador.
    """
    livre = house.DOOR - 2.0 * INFLATION
    assert livre > 0.2, (
        f'porta de {house.DOOR:.2f} m com inflação {INFLATION:.2f} m deixa só '
        f'{livre:.2f} m de passagem')


def test_as_paredes_do_simulador_sao_as_da_planta():
    """As caixas mandadas ao CoppeliaSim são os trechos da planta.

    É a invariante que impede o pior defeito possível deste pacote: o robô
    planejando num mapa e batendo noutro.
    """
    caixas = house.wall_boxes()
    trechos = house.walls()
    assert len(caixas) == len(trechos)
    for (cx, cy, sx, sy, sz), trecho in zip(caixas, trechos):
        bx, by, bsx, bsy = trecho.box
        assert (cx, cy, sx, sy) == (bx, by, bsx, bsy)
        assert sz == house.WALL_HEIGHT


def test_a_base_esta_na_sala():
    """A base é onde o house_builder põe o robô; ela tem de ser um cômodo."""
    assert house.room_at(*house.BASE) == 'sala'


# =============================================================================
#  O mapa publicado
# =============================================================================

def test_o_mapa_tem_o_tamanho_que_a_mensagem_declara():
    """Linhas e colunas conferem: a OccupancyGrid sai sem transposição."""
    cells, info = house.occupancy(0.05)
    assert len(cells) == info['rows']
    assert all(len(row) == info['cols'] for row in cells)
    esperado = math.ceil((house.SIZE + 2 * house.MAP_MARGIN) / 0.05)
    assert info['rows'] == esperado and info['cols'] == esperado


def test_o_mapa_tem_parede_e_tem_vazio():
    """Um mapa todo livre, ou todo bloqueado, passaria calado em tudo acima."""
    cells, _ = house.occupancy(0.05)
    valores = {value for row in cells for value in row}
    assert valores == {0, 1}
