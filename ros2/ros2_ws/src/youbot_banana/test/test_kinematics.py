"""Testes da cinemática, sem simulador.

`kinematics.py` é Python puro de propósito: dá para conferir a matemática aqui,
em segundos, sem abrir o CoppeliaSim. O que ESTES testes garantem é a coerência
interna (a inversa desfaz a direta, os limites são respeitados, as unidades
fecham); que o modelo corresponde ao robô DA CENA é outra verificação, e está em
`scene/measure_arm.py`, que compara contra a posição medida da garra.

    pytest test/
    python3 -m pytest test/test_kinematics.py -v
"""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from youbot_banana import kinematics  # noqa: E402  (precisa do sys.path acima)
from youbot_banana.kinematics import (  # noqa: E402
    ARM, HALF_BASE, HALF_TRACK, Unreachable, WHEEL_RADIUS, arm_fk, arm_ik,
    arm_ik_best, arm_plane, base_pose_for_grasp, grasp_roll, gripper_gap,
    gripper_opening, gripper_opening_for, gripper_targets, ref_to_ros,
    ros_to_ref, scale_to_limit, wheel_velocities, within_limits, wrap)


# =============================================================================
#  Base omnidirecional
# =============================================================================

def test_parado_nao_gira_roda():
    assert wheel_velocities(0.0, 0.0, 0.0) == (0.0, 0.0, 0.0, 0.0)


def test_andar_para_a_frente_gira_as_quatro_igual():
    """Só a translação para a frente: as quatro rodas na mesma velocidade."""
    speeds = wheel_velocities(0.1, 0.0, 0.0)
    assert len(set(round(speed, 9) for speed in speeds)) == 1
    # Velocidade negativa é para a frente nesta montagem (ver kinematics).
    assert speeds[0] < 0.0
    assert math.isclose(abs(speeds[0]), 0.1 / WHEEL_RADIUS)


def test_andar_de_lado_opoe_as_diagonais():
    """Translação lateral pura: as rodas em diagonal giram em sentidos opostos.

    É a assinatura da roda sueca — e é o que o robô diferencial dos exercícios
    anteriores não tem como fazer.
    """
    fl, fr, rl, rr = wheel_velocities(0.0, 0.1, 0.0)
    assert fl * fr < 0.0          # frente-esquerda contra frente-direita
    assert fl * rl < 0.0          # frente-esquerda contra trás-esquerda
    assert math.isclose(fl, rr)   # diagonais iguais
    assert math.isclose(fr, rl)


def test_girar_no_lugar_opoe_os_lados():
    """Giro puro: o lado esquerdo contra o direito, e nada de translação."""
    fl, fr, rl, rr = wheel_velocities(0.0, 0.0, 0.5)
    assert math.isclose(fl, rl)
    assert math.isclose(fr, rr)
    assert fl * fr < 0.0
    # O braço de alavanca é a soma dos dois semi-lados.
    assert math.isclose(abs(fl), 0.5 * (HALF_TRACK + HALF_BASE) / WHEEL_RADIUS)


def test_as_tres_liberdades_sao_independentes():
    """A soma dos três comandos é o comando da soma: o modelo é linear.

    É o que "holonômico" quer dizer em termos práticos, e é por isso que o
    controlador da base pode tratar avanço, deslocamento lateral e giro como
    três malhas separadas.
    """
    a = wheel_velocities(0.1, 0.0, 0.0)
    b = wheel_velocities(0.0, 0.07, 0.0)
    c = wheel_velocities(0.0, 0.0, 0.3)
    together = wheel_velocities(0.1, 0.07, 0.3)
    for index in range(4):
        assert math.isclose(together[index], a[index] + b[index] + c[index])


def test_saturacao_preserva_a_direcao():
    """Saturar reduz as quatro no mesmo fator, mantendo as proporções."""
    speeds = wheel_velocities(0.4, 0.2, 1.0)
    limited = scale_to_limit(speeds, 5.0)
    assert max(abs(speed) for speed in limited) == pytest.approx(5.0)
    for original, scaled in zip(speeds, limited):
        if abs(original) > 1e-9:
            assert math.isclose(scaled / original,
                                limited[0] / speeds[0], rel_tol=1e-9)


def test_saturacao_nao_mexe_no_que_ja_cabe():
    speeds = wheel_velocities(0.05, 0.0, 0.0)
    assert scale_to_limit(speeds, 5.0) == speeds


# =============================================================================
#  Conversão entre os dois referenciais
# =============================================================================

def test_frente_do_ros_e_frente_da_cena():
    """O +x do ROS é o +y da cena: é a frente do robô nas duas convenções."""
    assert ros_to_ref((1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))
    # E a esquerda do ROS (+y) é o -x da cena.
    assert ros_to_ref((0.0, 1.0, 0.0)) == pytest.approx((-1.0, 0.0, 0.0))


def test_conversao_de_frame_e_reversivel():
    point = (0.31, -0.12, 0.08)
    assert ref_to_ros(ros_to_ref(point)) == pytest.approx(point)
    assert ros_to_ref(ref_to_ros(point)) == pytest.approx(point)


# =============================================================================
#  Braço: direta e inversa
# =============================================================================

def test_postura_zero_estica_o_braco_para_cima():
    """Com as juntas em zero o braço aponta para cima, sobre o eixo da junta 0."""
    x, y, z = arm_fk((0.0, 0.0, 0.0, 0.0, 0.0))
    assert x == pytest.approx(ARM.tool_offset, abs=1e-6)
    assert y == pytest.approx(ARM.axis_y + ARM.shoulder_r, abs=1e-6)
    # A altura é a soma de todos os elos a partir do ombro.
    assert z == pytest.approx(
        ARM.shoulder_z + ARM.upper_arm + ARM.forearm + ARM.wrist_to_tip, abs=1e-6)


def test_junta_zero_gira_o_braco_em_torno_do_eixo_vertical():
    """A junta 0 muda o azimute e preserva o raio e a altura."""
    reference = arm_fk((0.0, -0.5, -1.0, -0.6, 0.0))
    turned = arm_fk((0.7, -0.5, -1.0, -0.6, 0.0))
    for point in (reference, turned):
        assert point[2] == pytest.approx(reference[2])
    radius_of = lambda p: math.hypot(p[0], p[1] - ARM.axis_y)   # noqa: E731
    assert radius_of(turned) == pytest.approx(radius_of(reference), abs=1e-9)
    _, _, _ = turned


def test_junta_quatro_nao_move_a_ponta():
    """A junta 4 gira em torno do próprio elo: orienta a garra, não a desloca."""
    base = arm_fk((0.2, -0.5, -1.0, -0.6, 0.0))
    rolled = arm_fk((0.2, -0.5, -1.0, -0.6, 1.2))
    assert rolled == pytest.approx(base)


@pytest.mark.parametrize('height', [0.02, 0.05, 0.10, 0.15])
@pytest.mark.parametrize('azimuth', [0.0, 0.6, -1.1])
def test_inversa_desfaz_a_direta(height, azimuth):
    """O teste central: resolver a inversa e aplicar a direta volta ao alvo."""
    radius = 0.22
    target = (-math.sin(azimuth) * radius,
              ARM.axis_y + math.cos(azimuth) * radius,
              height)
    configuration = arm_ik_best(target, approach=math.pi)
    assert arm_fk(configuration) == pytest.approx(target, abs=1e-9)


def test_inversa_respeita_os_limites_das_juntas():
    """Toda solução devolvida cabe nos intervalos lidos da cena."""
    for radius in (0.16, 0.20, 0.25, 0.30):
        configuration = arm_ik_best((0.0, ARM.axis_y + radius, 0.03),
                                    approach=math.pi)
        assert within_limits(configuration)


def test_o_limite_das_juntas_escolhe_o_cotovelo():
    """Neste braço a escolha do cotovelo não é livre: os limites decidem.

    A inversa de dois elos tem sempre duas soluções — cotovelo para cima e para
    baixo. Mas a junta 1 do youBot tem intervalo ASSIMÉTRICO ([-1,571, +1,309]),
    e o resultado é que, em todo o envelope de preensão, uma das duas soluções
    cai fora de limite. Por isso `arm_ik_best` tenta as duas e devolve a que
    couber, em vez de preferir uma por princípio.
    """
    viable = 0
    for radius in (0.16, 0.20, 0.25, 0.30):
        for height in (0.03, 0.10, 0.18):
            target = (0.0, ARM.axis_y + radius, height)
            solutions = []
            for elbow in ('up', 'down'):
                try:
                    solutions.append(arm_ik(target, approach=math.pi, elbow=elbow))
                except Unreachable:
                    pass
            if not solutions:
                continue
            viable += 1
            # Só uma sobrevive aos limites...
            assert len(solutions) == 1
            # ...e ela chega no alvo pedido.
            assert arm_fk(solutions[0]) == pytest.approx(target, abs=1e-9)
            assert arm_ik_best(target, approach=math.pi) == solutions[0]
    assert viable >= 8, 'o envelope encolheu: confira a geometria'


def test_os_dois_cotovelos_chegam_no_mesmo_ponto_quando_existem():
    """A matemática dos dois cotovelos, sem a interferência dos limites.

    Aqui os limites são afrouxados de propósito, para testar a GEOMETRIA: as
    duas soluções são posturas distintas que põem a ponta no mesmo lugar.
    """
    class Loose(type(ARM)):
        limits = ((-7.0, 7.0),) * 5

    loose = Loose()
    target = (0.0, ARM.axis_y + 0.22, 0.10)
    up = arm_ik(target, approach=math.pi, geom=loose, elbow='up')
    down = arm_ik(target, approach=math.pi, geom=loose, elbow='down')
    assert up != down
    assert arm_fk(up, loose) == pytest.approx(target, abs=1e-9)
    assert arm_fk(down, loose) == pytest.approx(target, abs=1e-9)


def test_longe_demais_explica_que_falta_alcance():
    with pytest.raises(Unreachable) as error:
        arm_ik_best((0.0, ARM.axis_y + 0.60, 0.03), approach=math.pi)
    assert 'estica' in str(error.value) or 'aproxime' in str(error.value)


def test_perto_demais_tambem_falha():
    """Alvo colado no eixo da junta 0: dentro do raio morto, ou fora de limite."""
    with pytest.raises(Unreachable):
        arm_ik_best((0.0, ARM.axis_y + 0.02, 0.03), approach=math.pi)


def test_envelope_de_preensao_no_chao_e_o_medido():
    """A faixa de raio em que o braço pega do chão: de ~0,15 a ~0,32 m.

    É o número que a missão usa para decidir onde parar a base, e por isso
    merece um teste: se uma mudança na geometria mexer nele, a missão passa a
    parar no lugar errado.
    """
    solved = []
    radius = 0.05
    while radius <= 0.45:
        try:
            arm_ik_best((0.0, ARM.axis_y + radius, 0.028), approach=math.pi)
            solved.append(radius)
        except Unreachable:
            pass
        radius += 0.005
    assert min(solved) == pytest.approx(0.15, abs=0.01)
    assert max(solved) == pytest.approx(0.315, abs=0.01)
    # E é contínuo: sem buracos no meio do envelope.
    assert len(solved) == pytest.approx(
        round((max(solved) - min(solved)) / 0.005) + 1, abs=1)


def test_o_angulo_de_aproximacao_muda_a_postura():
    """O mesmo ponto, com a garra mais inclinada, dá outra configuração.

    A inclinação não é livre: a 0,25 m do eixo e 0,12 m de altura, só de 150° a
    180° existe solução — mais deitada que isso, o pulso não alcança. Pegar
    DE LADO (90°) está fora do envelope deste braço nessa posição, e é por isso
    que a tarefa usa preensão por cima.
    """
    target = (0.0, ARM.axis_y + 0.25, 0.12)
    from_above = arm_ik_best(target, approach=math.pi)
    tilted = arm_ik_best(target, approach=math.radians(150.0))
    assert from_above != tilted
    assert arm_fk(from_above) == pytest.approx(target, abs=1e-9)
    assert arm_fk(tilted) == pytest.approx(target, abs=1e-9)
    with pytest.raises(Unreachable):
        arm_ik_best(target, approach=math.pi / 2.0)


def test_arm_plane_concorda_com_a_direta():
    """A decomposição em (azimute, raio, altura) é a que a direta usa."""
    configuration = (0.4, -0.7, -1.2, -0.5, 0.0)
    point = arm_fk(configuration)
    azimuth, radius, height = arm_plane(point)
    assert azimuth == pytest.approx(configuration[0], abs=1e-9)
    assert height == pytest.approx(point[2])
    assert radius > 0.0


# =============================================================================
#  Garra
# =============================================================================

def test_garra_fechada_e_aberta_sao_os_extremos():
    assert gripper_targets(0.0) == pytest.approx(ARM.gripper_closed)
    assert gripper_targets(1.0) == pytest.approx(ARM.gripper_open)


def test_abertura_e_sua_propria_inversa():
    for wanted in (0.0, 0.25, 0.5, 0.75, 1.0):
        _, j2 = gripper_targets(wanted)
        assert gripper_opening(j2) == pytest.approx(wanted)


def test_vao_da_garra_nos_extremos():
    """O vão útil vai de 26 a 76 mm — medido na cena, conferido aqui."""
    assert gripper_gap(*ARM.gripper_closed) == pytest.approx(
        ARM.gripper_gap_closed, abs=1e-4)
    assert gripper_gap(*ARM.gripper_open) == pytest.approx(
        ARM.gripper_gap_open, abs=1e-4)


def test_vao_cresce_com_a_abertura():
    gaps = [gripper_gap(*gripper_targets(value / 10.0)) for value in range(11)]
    assert gaps == sorted(gaps)


def test_banana_em_tamanho_natural_nao_cabe():
    """100 mm de largura contra 76 mm de vão: é por isso que a cena reduz a banana."""
    assert gripper_opening_for(0.100) is None


def test_banana_reduzida_cabe_com_folga():
    """A banana a 0,5x tem 50 mm, e a garra fecha em volta dela."""
    opening = gripper_opening_for(0.050, clearance=0.008)
    assert opening is not None
    assert 0.0 < opening < 1.0
    # A abertura pedida tem de deixar a banana passar entre os dedos.
    assert gripper_gap(*gripper_targets(opening)) > 0.050


# =============================================================================
#  Pose de preensão
# =============================================================================

def test_giro_da_garra_fecha_atravessado_no_objeto():
    """Os dedos abrem perpendiculares ao eixo maior do objeto.

    O azimute da abertura é q0 - q4 (medido no simulador), então conferir a
    perpendicularidade é conferir essa conta de ponta a ponta.
    """
    for object_yaw in (0.0, 0.4, -0.9, 1.5):
        for q0 in (0.0, 0.3, -0.6):
            roll = grasp_roll(object_yaw, q0)
            opening_azimuth = q0 - roll
            # Perpendicular ao eixo do objeto, a menos de meia volta (a garra é
            # simétrica: abrir em ψ ou em ψ+180° é a mesma preensão).
            difference = abs(wrap(opening_azimuth - object_yaw))
            assert (difference == pytest.approx(math.pi / 2.0, abs=1e-6)
                    or difference == pytest.approx(math.pi / 2.0, abs=1e-6))


def test_giro_da_garra_cabe_no_limite_da_junta():
    low, high = ARM.limits[4]
    for object_yaw in [value * math.pi / 8.0 for value in range(-8, 9)]:
        for q0 in (-1.0, 0.0, 1.0):
            assert low <= grasp_roll(object_yaw, q0) <= high


def test_pose_da_base_desconta_o_deslocamento_do_braco():
    """A base para mais longe do objeto do que o alcance pedido.

    O braço fica 0,166 m à frente do centro da base: esquecer isso deixa o
    objeto 17 cm fora do alcance com o robô aparentemente no lugar certo.
    """
    banana = (1.0, 2.0)
    reach = 0.233
    x, y, yaw = base_pose_for_grasp(banana, 0.0, reach)
    assert yaw == 0.0
    assert math.dist((x, y), banana) == pytest.approx(reach + ARM.axis_y)
    # E o robô fica virado PARA o objeto.
    assert math.atan2(banana[1] - y, banana[0] - x) == pytest.approx(yaw)


@pytest.mark.parametrize('approach_yaw', [0.0, 1.2, -2.5, math.pi])
def test_pose_da_base_olha_para_o_objeto_em_qualquer_direcao(approach_yaw):
    banana = (-0.4, 1.7)
    x, y, yaw = base_pose_for_grasp(banana, approach_yaw, 0.233)
    assert wrap(math.atan2(banana[1] - y, banana[0] - x) - yaw) == pytest.approx(
        0.0, abs=1e-9)


def test_a_banana_cai_no_envelope_quando_a_base_para_onde_devia():
    """O fecho do raciocínio: base na pose planejada -> inversa tem solução.

    Junta as três peças (pose da base, conversão de frame, inversa) e confirma
    que elas concordam entre si. É este encadeamento que faz o robô pegar a
    banana em vez de parar perto dela.
    """
    banana_world = (1.0, 2.0)
    reach = 0.233
    for approach_yaw in (0.0, 0.8, -2.0, 3.0):
        base_x, base_y, base_yaw = base_pose_for_grasp(
            banana_world, approach_yaw, reach)
        # A banana vista da base, na convenção do ROS.
        dx = banana_world[0] - base_x
        dy = banana_world[1] - base_y
        cos_yaw, sin_yaw = math.cos(base_yaw), math.sin(base_yaw)
        local = (cos_yaw * dx + sin_yaw * dy,
                 -sin_yaw * dx + cos_yaw * dy,
                 0.028)
        configuration = arm_ik_best(ros_to_ref(local), approach=math.pi)
        assert within_limits(configuration)
        # E a ponta cai sobre a banana.
        back = ref_to_ros(arm_fk(configuration))
        assert back == pytest.approx(local, abs=1e-9)


# =============================================================================
#  Utilidades
# =============================================================================

def test_wrap_normaliza_para_meia_volta():
    assert wrap(0.0) == pytest.approx(0.0)
    # Meia volta é a mesma direção em +pi e em -pi; qual dos dois sai depende
    # do sinal do zero que o atan2 recebe, e não importa para o uso.
    assert abs(wrap(3.0 * math.pi)) == pytest.approx(math.pi)
    assert abs(wrap(-3.0 * math.pi)) == pytest.approx(math.pi)
    assert wrap(math.pi / 2.0) == pytest.approx(math.pi / 2.0)


def test_alcance_do_braco_e_a_soma_dos_elos():
    assert ARM.reach == pytest.approx(
        ARM.shoulder_r + ARM.upper_arm + ARM.forearm + ARM.wrist_to_tip)


def test_ganho_de_escorregamento_e_o_medido():
    """O ganho medido na base. Se mudar, base_control precisa saber."""
    assert 0.5 < kinematics.SLIP_GAIN < 0.8
