#!/usr/bin/env python3
"""Mede os limites do rover: até que degrau ele sobe, e o que cada peça faz.

Roda em lote, direto pela Remote API, sem ROS. É deliberado: o que se mede aqui
é o MECANISMO — a geometria do wheg contra a geometria do degrau — e misturar a
máquina de estados no meio só acrescentaria variáveis. O que os nós ROS
acrescentam (achar a escada, alinhar, parar no topo) é outra coisa, e se
demonstra rodando o launch.

    python3 scene/characterize.py roda        # fechada x aberta, a mesma escada
    python3 scene/characterize.py bitola      # estabilidade lateral da garra
    python3 scene/characterize.py abertura    # quanto a perna deve abrir
    python3 scene/characterize.py espelho     # até que altura de degrau sobe
    python3 scene/characterize.py raio        # efeito do raio do wheg
    python3 scene/characterize.py marcha      # alternada vs. junta vs. dianteira
    python3 scene/characterize.py cauda       # com e sem cauda
    python3 scene/characterize.py raios       # 2, 3, 4, 5 raios por wheg
    python3 scene/characterize.py tudo

Cada ensaio monta a cena do zero, solta o rover com a marcha pedida em malha
aberta e observa. Sucesso é chegar ao patamar NIVELADO — não é ganhar altura,
porque um rover empinado na beirada também ganha altura, e foi assim que a
primeira versão deste script se enganou.
"""

import argparse
import math
import os
import statistics
import sys
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))

from stair_whegs import geometry  # noqa: E402  (precisa do sys.path acima)

import build_scene  # noqa: E402  (mesma pasta)


class Args:
    """Os parâmetros que `build_scene` espera, como objeto simples."""

    def __init__(self, **kwargs):
        self.wheg_radius = geometry.WHEG_RADIUS
        self.spokes = geometry.SPOKES
        self.leg_open = geometry.LEG_OPEN
        self.leg_width = 0.012
        self.wheelbase = geometry.WHEELBASE
        self.track = geometry.TRACK
        self.body_height = 0.030
        self.tail = geometry.TAIL_LENGTH
        self.start_x = 0.0
        self.riser = geometry.RISER
        self.tread = geometry.TREAD
        self.steps = geometry.STEPS
        self.width = geometry.STAIR_WIDTH
        self.stair_x = geometry.STAIR_X0
        self.landing = geometry.LANDING
        self.__dict__.update(kwargs)


def trial(client, sim, gait=geometry.DEFAULT_GAIT, speed=geometry.CRUISE,
          seconds=26.0, wheel=geometry.GARRA, **scene):
    """Um ensaio: monta a cena, anda em malha aberta, devolve o que aconteceu.

    `wheel` é o estado da roda transformável durante o ensaio inteiro:
    `geometry.GARRA` (o padrão, pernas abertas — é a configuração de escalada)
    ou `geometry.RODA` (aro fechado).

    Devolve um dicionário com a altura ganha, se chegou ao patamar nivelado, e
    quanto tempo levou.
    """
    args = Args(**scene)
    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.03)

    info = build_scene.build(sim, args)
    chassis = sim.getObject('/rover/chassis')
    imu = sim.getObject('/rover/chassis/imu')
    joints = {name: sim.getObject(f'/rover/chassis/{name}_joint')
              for name in geometry.WHEGS}
    # As juntas de perna: é por elas que a roda fecha ou abre. Aqui o estado
    # é FIXO durante o ensaio inteiro, e de propósito — o que se mede neste
    # arquivo é o mecanismo, e trocar de estado no meio misturaria a decisão
    # do `climber` com a geometria da roda. Quem troca em movimento é o launch.
    legs = [sim.getObject(f'/rover/chassis/{name}_joint/{name}_hub/'
                          f'{name}_leg{index}_joint')
            for name in geometry.WHEGS for index in range(args.spokes)]
    # O ângulo vem de `args.leg_open`, e não da constante do módulo: o ensaio
    # de abertura monta rodas com aberturas diferentes, e ler a constante faria
    # todas elas receberem o alvo da roda padrão.
    leg_angle = (0.0 if wheel == geometry.RODA
                 else -geometry.LEG_RAKE * args.leg_open)
    phases = geometry.phases(gait, args.spokes)

    client.setStepping(True)
    sim.startSimulation()
    # A cena nasce com a roda FECHADA. Abrir levanta o rover de um raio para
    # quase dois, e o que vem logo depois é uma queda: estes passos parados
    # são para ele assentar ANTES de a medida de altura começar, ou a subida
    # do ensaio incluiria a própria transformação.
    for _ in range(40):
        for joint in legs:
            sim.setJointTargetPosition(joint, leg_angle)
        sim.step()
    start = sim.getObjectPosition(chassis, sim.handle_world)

    elapsed = 0.0
    best_z = start[2]
    arrived_at = None
    # A ROLAGEM passou a ser um modo de falha de verdade. Com a garra aberta o
    # centro de massa sobe para quase o dobro e a bitola não muda: o rover
    # deixa de empacar no degrau e passa a CAPOTAR DE LADO, que um registro só
    # de altura e pitch não distingue de "parou".
    rolls = []
    # O patamar começa onde a escada acaba; chegar a ele com o corpo nivelado
    # é o único critério de sucesso.
    top_z = info['top_z']
    pitches = []
    while elapsed < seconds:
        elapsed += 0.05
        for name, joint in joints.items():
            sim.setJointTargetPosition(joint, phases[name] - speed * elapsed)
        for joint in legs:
            sim.setJointTargetPosition(joint, leg_angle)
        sim.step()
        position = sim.getObjectPosition(chassis, sim.handle_world)
        matrix = sim.getObjectMatrix(imu, sim.handle_world)
        pitch = math.asin(max(-1.0, min(1.0, matrix[8])))
        pitches.append(abs(pitch))
        rolls.append(abs(math.atan2(-matrix[9], matrix[10])))
        best_z = max(best_z, position[2])
        on_top = (position[0] > info['top_x']
                  and position[2] > start[2] + top_z - args.riser / 2.0
                  and abs(pitch) < math.radians(12.0))
        if on_top and arrived_at is None:
            arrived_at = elapsed
            break

    position = sim.getObjectPosition(chassis, sim.handle_world)
    sim.stopSimulation()
    client.setStepping(False)
    while sim.getSimulationState() != sim.simulation_stopped:
        time.sleep(0.03)

    return {
        'subiu': position[2] - start[2],
        'pico': best_z - start[2],
        'x': position[0],
        'topo': arrived_at is not None,
        't': arrived_at if arrived_at else seconds,
        'pitch_max': math.degrees(max(pitches)) if pitches else 0.0,
        'pitch_medio': math.degrees(statistics.mean(pitches)) if pitches else 0.0,
        'roll_max': math.degrees(max(rolls)) if rolls else 0.0,
        'capotou': bool(rolls) and max(rolls) > math.radians(60.0),
        'degraus_possiveis': top_z / args.riser,
    }


# =============================================================================
#  Os experimentos
# =============================================================================

def header(title, columns):
    print()
    print(f'=== {title} ===')
    print(columns)


def line(label, result, riser):
    print(f'{label:26} {result["subiu"] * 100:8.1f} {result["subiu"] / riser:8.1f} '
          f'{result["t"]:7.1f} {result["pitch_max"]:9.0f} '
          f'{result["roll_max"]:8.0f} '
          f'{"CAPOTOU" if result["capotou"] else ("SIM" if result["topo"] else "nao"):>7}')


COLUMNS = (f'{"ensaio":26} {"subiu cm":>8} {"degraus":>8} {"t s":>7} '
           f'{"pitch max":>9} {"roll max":>8} {"topo":>7}')


def experiment_riser(client, sim):
    """Até que altura de degrau o rover sobe, com o wheg padrão."""
    header(f'altura do degrau (wheg de {geometry.WHEG_RADIUS * 1000:.0f} mm)',
           COLUMNS)
    for riser in (0.020, 0.030, 0.040, 0.050, 0.060, 0.070):
        result = trial(client, sim, riser=riser)
        line(f'espelho {riser * 1000:.0f} mm '
             f'({riser / geometry.WHEG_RADIUS:.1f} raio)', result, riser)


def experiment_radius(client, sim):
    """Qual degrau cada raio de wheg vence, mantendo o degrau fixo."""
    header('raio do wheg (degrau de 50 mm)', COLUMNS)
    for radius in (0.035, 0.045, 0.050, 0.060, 0.070):
        result = trial(client, sim, wheg_radius=radius, riser=0.050)
        line(f'raio {radius * 1000:.0f} mm '
             f'({0.050 / radius:.1f} degrau/raio)', result, 0.050)


def experiment_gait(client, sim):
    """O que a coordenação de fases muda."""
    header('marcha', COLUMNS)
    for gait in ('alternada', 'dianteira', 'junta'):
        result = trial(client, sim, gait=gait)
        line(f'{gait} (vão {geometry.longest_gap(gait):.0%})',
             result, geometry.RISER)


def experiment_tail(client, sim):
    """O que a cauda muda."""
    header('cauda', COLUMNS)
    for tail, label in ((0.0, 'sem cauda'), (0.06, 'cauda 60 mm'),
                        (0.09, 'cauda 90 mm'), (0.12, 'cauda 120 mm')):
        result = trial(client, sim, tail=tail)
        line(label, result, geometry.RISER)


def experiment_spokes(client, sim):
    """Quantos raios por wheg: o compromisso entre subir e andar liso."""
    header('raios por wheg', COLUMNS)
    for spokes in (2, 3, 4, 5):
        result = trial(client, sim, spokes=spokes)
        bounce = geometry.body_bounce(geometry.WHEG_RADIUS, spokes) * 1000
        line(f'{spokes} raios (balanço {bounce:.0f} mm)', result, geometry.RISER)


def experiment_robustness(client, sim, riser=0.060, tries=5):
    """Taxa de sucesso de cada marcha num degrau difícil, e não um ensaio só.

    O ensaio isolado é determinístico — mesma cena, mesmas condições iniciais,
    mesmo resultado — e por isso repeti-lo não diz nada sobre robustez. O que
    muda o resultado é a FASE em que o rover encontra o primeiro degrau, e essa
    fase depende de onde ele partiu. Variando a posição de partida em frações
    de um passo, sai uma taxa em vez de uma anedota.

    É a diferença entre "esta marcha subiu" e "esta marcha sobe".
    """
    header(f'robustez: degrau de {riser * 1000:.0f} mm, '
           f'{tries} posições de partida',
           f'{"marcha":26} {"sucessos":>10} {"degraus médios":>15} '
           f'{"pitch máx":>10}')
    # Um passo de marcha avança o rover cerca de 2*R*sen(60°); espalhar as
    # partidas por essa distância cobre todas as fases de ataque ao degrau.
    stride = 2.0 * geometry.WHEG_RADIUS * math.sin(math.pi / geometry.SPOKES)
    for gait in ('alternada', 'dianteira', 'junta'):
        wins, climbed, pitch = 0, [], []
        for index in range(tries):
            offset = stride * index / tries
            result = trial(client, sim, gait=gait, riser=riser,
                           start_x=-offset)
            wins += 1 if result['topo'] else 0
            climbed.append(result['subiu'] / riser)
            pitch.append(result['pitch_max'])
        print(f'{gait:26} {wins:>7}/{tries} {statistics.mean(climbed):15.1f} '
              f'{max(pitch):10.0f}')


def experiment_wheel(client, sim):
    """O ensaio que justifica o mecanismo: os dois estados, degrau por degrau.

    A primeira versão deste ensaio usava um degrau só, o de 40 mm da cena, e
    dava a resposta errada: OS DOIS ESTADOS SOBEM. Foi uma surpresa útil. A
    intuição de que "roda lisa não sobe degrau" vale para uma roda só, puxando
    contra a face vertical do espelho; este rover tem tração nas quatro, uma
    cauda, e o aro fechado se apoia na QUINA como qualquer roda se apoia num
    meio-fio. Com 40 mm contra um aro de 50 mm — 0,8 raio — ele passa.

    O mecanismo não serve para subir o degrau fácil. Serve para o degrau em
    que o aro para. Por isso o ensaio varre a ALTURA e procura onde as duas
    colunas se separam: é lá que a transformação deixa de ser enfeite.
    """
    header(f'estado da roda por altura de degrau '
           f'(aro de {geometry.WHEG_RADIUS * 1000:.0f} mm, garra de '
           f'{geometry.claw_reach() * 1000:.0f} mm)', COLUMNS)
    for riser in (0.040, 0.055, 0.070, 0.085):
        for wheel, label in ((geometry.RODA, 'fechada'),
                             (geometry.GARRA, 'aberta ')):
            # A marcha de ESCALADA nos dois, que é a que o `climber` usa
            # quando ataca o degrau. Com `alternada` a comparação não mede a
            # roda: mede o balanço de rolagem dela, e a garra aberta capota
            # antes de mostrar o que alcança (ver `geometry.CLIMB_GAIT`).
            result = trial(client, sim, wheel=wheel, riser=riser,
                           gait=geometry.CLIMB_GAIT)
            line(f'{riser * 1000:.0f} mm, {label} '
                 f'({riser / geometry.WHEG_RADIUS:.1f} aro)', result, riser)


def experiment_track(client, sim):
    """Bitola: o que a garra aberta cobra em estabilidade lateral.

    Abrir a garra levanta o centro de massa e não mexe na bitola, e o ângulo
    de tombamento lateral — `atan(meia-bitola / altura)` — cai junto. A marcha
    `alternada` ainda balança o corpo em rolagem, porque põe os dois lados em
    fases opostas: medido em piso plano, ±14°. Os dois números se encontram, e
    o rover deita.

    É um modo de falha que a versão de raios fixos não tinha, e que nenhuma
    medida de altura revela: o rover "para de subir" porque está de lado. Daí
    a coluna de rolagem e o veredito CAPOTOU existirem.
    """
    header(f'bitola (garra de {geometry.claw_reach() * 1000:.0f} mm, '
           f'degrau de {geometry.RISER * 1000:.0f} mm)', COLUMNS)
    for track in (0.15, 0.18, 0.22, 0.26):
        for gait in ('alternada', 'junta'):
            result = trial(client, sim, gait=gait, track=track)
            tip = math.degrees(math.atan2(track / 2.0, geometry.claw_reach()))
            line(f'{track * 1000:.0f} mm, {gait} ({tip:.0f}°)',
                 result, geometry.RISER)


def experiment_opening(client, sim):
    """Quanto a perna deve abrir — e por que o limite NÃO é o alcance.

    `geometry.claw_reach` é monótona: cada grau a mais alcança mais longe, e
    por esse critério a perna deveria abrir o máximo possível. O ensaio diz
    outra coisa, porque o alcance da garra é também a ALTURA EM QUE O ROVER
    ANDA, e com ela crescem o braço que o empina na quina, a altura do centro
    de massa e o atraso com que a cauda rígida encontra o chão.
    """
    header(f'abertura da perna (degrau de {geometry.RISER * 1000:.0f} mm, '
           f'entre-eixos de {geometry.WHEELBASE * 1000:.0f} mm)',
           COLUMNS + f' {"garra mm":>9} {"g/base":>8} {"cauda":>7}')
    for degrees in (30, 45, 60, 80):
        opening = math.radians(degrees)
        reach = geometry.claw_reach(open_angle=opening)
        catch = geometry.tail_catch_angle(reach)
        result = trial(client, sim, leg_open=opening)
        line(f'{degrees}° ({reach / geometry.WHEG_RADIUS:.2f} aro)',
             result, geometry.RISER)
        print(f'{"":26} {"":8} {"":8} {"":7} {"":9} {"":8} {"":7} '
              f'{reach * 1000:9.0f} {reach / geometry.WHEELBASE:8.2f} '
              f'{math.degrees(catch) if catch else float("nan"):6.0f}°')


EXPERIMENTS = {
    'abertura': experiment_opening,
    'bitola': experiment_track,
    'roda': experiment_wheel,
    'robustez': experiment_robustness,
    'espelho': experiment_riser,
    'raio': experiment_radius,
    'marcha': experiment_gait,
    'cauda': experiment_tail,
    'raios': experiment_spokes,
}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('experimento', nargs='?', default='tudo',
                        choices=sorted(EXPERIMENTS) + ['tudo'])
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=23000)
    args = parser.parse_args()

    try:
        client = RemoteAPIClient(args.host, args.port)
        sim = client.require('sim')
    except Exception:
        raise SystemExit(f'CoppeliaSim não responde em {args.host}:{args.port}.')

    chosen = (sorted(EXPERIMENTS) if args.experimento == 'tudo'
              else [args.experimento])
    for name in chosen:
        EXPERIMENTS[name](client, sim)
    print()
    print('"topo" é chegar ao patamar com o corpo nivelado (pitch < 12°). '
          'Ganhar altura sem chegar lá é o rover empinado na escada.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
