"""Testes da geometria e da marcha, sem simulador.

`geometry.py` é Python puro de propósito: a matemática da marcha se confere
aqui, em segundos. Que o rover SOBE é outra verificação, e está em
`scene/characterize.py`, que mede no simulador.

    pytest test/
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from stair_whegs import geometry  # noqa: E402  (precisa do sys.path acima)
from stair_whegs.geometry import (  # noqa: E402
    GAITS, SPOKES, WHEGS, gait_targets, longest_gap, max_riser, phases,
    rephase, stair_height, step_angle)


# =============================================================================
#  Fases
# =============================================================================

def test_toda_marcha_define_os_quatro_whegs():
    for name, pattern in GAITS.items():
        assert set(pattern) == set(WHEGS), f'marcha {name} incompleta'


def test_fases_sao_fracoes_de_um_passo_nao_de_uma_volta():
    """O passo é o ângulo entre dois raios, e é ele que se repete no contato."""
    assert step_angle(3) == pytest.approx(2.0 * math.pi / 3.0)
    assert step_angle(4) == pytest.approx(math.pi / 2.0)
    # Meia-fase na marcha alternada é meio passo, não meia volta.
    half = phases('alternada', spokes=3)['fr']
    assert half == pytest.approx(step_angle(3) / 2.0)
    assert half == pytest.approx(math.radians(60.0))


def test_fases_acompanham_o_numero_de_raios():
    """Com quatro raios o passo encurta, e a defasagem encurta junto."""
    assert phases('alternada', spokes=4)['fr'] == pytest.approx(math.radians(45.0))


def test_diagonais_andam_juntas_na_marcha_alternada():
    """É a definição da marcha: fl com rr, fr com rl."""
    p = phases('alternada')
    assert p['fl'] == pytest.approx(p['rr'])
    assert p['fr'] == pytest.approx(p['rl'])
    assert abs(p['fl'] - p['fr']) == pytest.approx(step_angle() / 2.0)


def test_marcha_desconhecida_diz_quais_existem():
    with pytest.raises(KeyError) as error:
        phases('galope')
    assert 'alternada' in str(error.value)


# =============================================================================
#  Geração de alvos
# =============================================================================

def test_parado_os_alvos_nao_andam():
    a = gait_targets(0.0, speed=0.0)
    b = gait_targets(5.0, speed=0.0)
    assert a == b


def test_os_alvos_andam_na_velocidade_pedida():
    """θ(t) = φ + ω t: em um segundo, cada wheg avança ω radianos."""
    speed = 2.5
    a = gait_targets(1.0, speed)
    b = gait_targets(2.0, speed)
    for name in WHEGS:
        assert b[name] - a[name] == pytest.approx(speed)


def test_sem_giro_os_dois_lados_andam_igual():
    targets = gait_targets(3.0, speed=2.0, turn=0.0)
    # Descontadas as fases, o avanço é o mesmo dos dois lados.
    base = phases()
    avancos = {name: targets[name] - base[name] for name in WHEGS}
    assert len(set(round(v, 9) for v in avancos.values())) == 1


def test_girar_separa_os_lados():
    """O giro é por derrapagem: um lado anda mais que o outro."""
    turn = 1.0
    targets = gait_targets(2.0, speed=3.0, turn=turn)
    base = phases()
    left = targets['fl'] - base['fl']
    right = targets['fr'] - base['fr']
    # A diferença entre os lados é `turn` por segundo de marcha.
    assert left - right == pytest.approx(turn * 2.0)


def test_girar_parado_gira_no_lugar():
    """Velocidade zero e giro: os lados andam em sentidos opostos."""
    targets = gait_targets(1.0, speed=0.0, turn=2.0)
    base = phases()
    left = targets['fl'] - base['fl']
    right = targets['fr'] - base['fr']
    assert left == pytest.approx(-right)
    assert left != 0.0


def test_as_duas_rodas_de_um_lado_andam_juntas():
    targets = gait_targets(2.0, speed=3.0, turn=0.7)
    base = phases()
    assert (targets['fl'] - base['fl']) == pytest.approx(targets['rl'] - base['rl'])
    assert (targets['fr'] - base['fr']) == pytest.approx(targets['rr'] - base['rr'])


# =============================================================================
#  Troca de marcha sem salto
# =============================================================================

def test_trocar_de_marcha_nao_salta():
    """O ponto de `rephase`: a marcha nova começa onde os whegs já estão.

    Sem isso, as juntas correriam para a fase nova na velocidade máxima do
    motor e o rover levaria um tranco — o tipo de movimento que faz perder o
    apoio no meio de um degrau.
    """
    time, speed = 4.3, 2.0
    atual = gait_targets(time, speed, gait='alternada')
    offset = rephase(atual, time, speed, gait='junta')
    nova = gait_targets(time, speed, gait='junta', offset=offset)
    for name in WHEGS:
        assert nova[name] == pytest.approx(atual[name], abs=1e-12)


def test_depois_da_troca_a_marcha_nova_vale():
    """O salto some, mas a RELAÇÃO entre os whegs passa a ser a da marcha nova."""
    time, speed = 4.3, 2.0
    atual = gait_targets(time, speed, gait='alternada')
    offset = rephase(atual, time, speed, gait='junta')
    depois = gait_targets(time + 1.0, speed, gait='junta', offset=offset)
    # Em `junta` os quatro ficam com a mesma defasagem entre si que tinham no
    # instante da troca, e passam a avançar juntos.
    avancos = {name: depois[name] - atual[name] for name in WHEGS}
    assert len(set(round(v, 9) for v in avancos.values())) == 1


# =============================================================================
#  Medidas de projeto
# =============================================================================

def test_vao_sem_apoio_separa_as_marchas():
    """A medida que distingue uma marcha espalhada de uma marcha junta."""
    assert longest_gap('alternada') == pytest.approx(0.5)
    assert longest_gap('dianteira') == pytest.approx(0.5)
    assert longest_gap('junta') == pytest.approx(1.0)


def test_balanco_do_corpo_cresce_quando_ha_menos_raios():
    """Menos raios, mais balanço: é o preço de não ter aro."""
    bounce = [geometry.body_bounce(0.05, n) for n in (2, 3, 4, 6)]
    assert bounce == sorted(bounce, reverse=True)
    # Com três raios, metade de um raio... não: 1 - cos(60°) = 0,5.
    assert geometry.body_bounce(0.05, 3) == pytest.approx(0.025)
    # Com muitos raios a roda vira aro e o balanço some.
    assert geometry.body_bounce(0.05, 36) < 0.001


def test_degrau_vencivel_vem_da_medida_e_nao_do_raio():
    """CLIMB_RATIO é medido: o rover sobe MAIS que um raio, não menos."""
    assert geometry.CLIMB_RATIO > 1.0
    assert max_riser(0.05) == pytest.approx(0.05 * geometry.CLIMB_RATIO)
    # O degrau padrão da cena tem de caber com folga.
    assert geometry.RISER < max_riser()


# =============================================================================
#  A escada
# =============================================================================

def test_altura_da_escada_e_zero_antes_dela():
    assert stair_height(0.0) == 0.0
    assert stair_height(geometry.STAIR_X0 - 0.01) == 0.0


def test_altura_da_escada_sobe_um_degrau_por_piso():
    x0, tread, riser = geometry.STAIR_X0, geometry.TREAD, geometry.RISER
    assert stair_height(x0 + 0.5 * tread) == pytest.approx(riser)
    assert stair_height(x0 + 1.5 * tread) == pytest.approx(2 * riser)
    assert stair_height(x0 + 2.5 * tread) == pytest.approx(3 * riser)


def test_a_escada_para_de_subir_no_topo():
    x0, tread = geometry.STAIR_X0, geometry.TREAD
    topo = geometry.STEPS * geometry.RISER
    assert stair_height(x0 + geometry.STEPS * tread + 0.3) == pytest.approx(topo)
    assert stair_height(10.0) == pytest.approx(topo)


def test_a_escada_da_cena_cabe_no_rover():
    """O degrau padrão tem de estar dentro do que o wheg vence."""
    assert geometry.RISER <= max_riser(geometry.WHEG_RADIUS)
    # E o piso tem de ser mais fundo que o balanço, ou o rover nunca assenta.
    assert geometry.TREAD > 2 * geometry.body_bounce()


# =============================================================================
#  Consistência do pacote
# =============================================================================

def test_a_ordem_dos_whegs_e_a_mesma_em_todo_lugar():
    """A ponte zipa esta ordem com as juntas da cena: ela não pode mudar."""
    assert WHEGS == ('fl', 'fr', 'rl', 'rr')
    assert len(set(WHEGS)) == 4


def test_a_marcha_padrao_existe():
    assert geometry.DEFAULT_GAIT in GAITS


def test_spokes_e_coerente_com_o_passo():
    assert step_angle(SPOKES) == pytest.approx(2.0 * math.pi / SPOKES)
