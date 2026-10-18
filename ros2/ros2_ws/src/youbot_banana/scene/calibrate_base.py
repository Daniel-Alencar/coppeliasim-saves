#!/usr/bin/env python3
"""Confere a cinemática inversa da base omnidirecional contra o simulador.

O padrão de sinais das quatro rodas suecas sai do script do próprio modelo, mas
o fator que converte velocidade de roda em velocidade do corpo não: para a roda
sueca ele depende do ângulo dos rolos no aro. `kinematics.wheel_velocities` usa

    braço de alavanca do giro = meia-bitola + meia-base

e este script mede se é verdade. Ele comanda, uma de cada vez, as três
liberdades da base e mede o que o corpo realmente fez:

    python3 scene/calibrate_base.py
    python3 scene/calibrate_base.py --speed 0.3 --seconds 3

A medida é feita no frame do youBot_ref no instante inicial, porque é nele que
"para a frente" e "para a esquerda" fazem sentido. Com a cena aberta e o robô
num espaço livre — ele anda alguns decímetros durante o ensaio.
"""

import argparse
import math
import os
import sys
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from youbot_banana.coppelia_scene import (  # noqa: E402  (precisa do sys.path)
    WHEEL_JOINTS, find_joints, script_names, silence_joint_scripts)
from youbot_banana.kinematics import (  # noqa: E402
    HALF_BASE, HALF_TRACK, MAX_WHEEL_FORCE, MAX_WHEEL_SPEED, WHEEL_RADIUS,
    wheel_velocities)


def reset_to(sim, robot, pose):
    """Devolve o robô à pose inicial e zera a velocidade dele.

    Sem isto, cada ensaio parte de onde o anterior terminou, o robô atravessa a
    arena e, por volta do quinto ensaio, encosta numa parede ou num poop. O
    ganho medido então despenca, e a culpa parece ser da cinemática — foi
    exatamente o que aconteceu na primeira versão deste script, com o mesmo
    comando de 0,10 m/s medindo ganho 0,66 numa rodada e 0,17 na seguinte.

    `resetDynamicObject` reconstrói o corpo dinâmico na pose nova, o que zera
    as velocidades acumuladas; teleportar sem isso guardaria o impulso do
    ensaio anterior.
    """
    sim.setObjectPose(robot, sim.handle_world, pose)
    sim.resetDynamicObject(robot)
    for _ in range(20):
        sim.step()


def measure(sim, client, wheels, reference, command, seconds):
    """Comanda um twist por `seconds` e devolve o twist realmente observado."""
    start_pose = sim.getObjectMatrix(reference, sim.handle_world)
    start_time = sim.getSimulationTime()

    targets = wheel_velocities(*command)
    for handle, target in zip(wheels, targets):
        sim.setJointTargetVelocity(handle, target)

    steps = int(seconds / sim.getFloatParam(sim.floatparam_simulation_time_step))
    for _ in range(steps):
        sim.step()

    for handle in wheels:
        sim.setJointTargetVelocity(handle, 0.0)
    for _ in range(20):            # deixa parar antes de medir
        sim.step()

    end_pose = sim.getObjectMatrix(reference, sim.handle_world)
    elapsed = sim.getSimulationTime() - start_time

    # Deslocamento no mundo, levado ao frame do robô no instante inicial. As
    # colunas 1 e 2 da matriz 3x4 (em ordem de linhas) são os eixos x e y do
    # robô vistos do mundo; a frente do youBot é o +y.
    dx = end_pose[3] - start_pose[3]
    dy = end_pose[7] - start_pose[7]
    left_axis = (-start_pose[0], -start_pose[4])     # -x do robô = esquerda
    front_axis = (start_pose[1], start_pose[5])      # +y do robô = frente
    forward = dx * front_axis[0] + dy * front_axis[1]
    sideways = dx * left_axis[0] + dy * left_axis[1]

    start_yaw = math.atan2(start_pose[5], start_pose[1])
    end_yaw = math.atan2(end_pose[5], end_pose[1])
    rotation = math.atan2(math.sin(end_yaw - start_yaw),
                          math.cos(end_yaw - start_yaw))

    return (forward / elapsed, sideways / elapsed, rotation / elapsed), elapsed


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=23000)
    parser.add_argument('--robot', default='/youBot')
    parser.add_argument('--speed', type=float, default=0.20, help='[m/s]')
    parser.add_argument('--turn', type=float, default=0.60, help='[rad/s]')
    parser.add_argument('--seconds', type=float, default=2.0)
    args = parser.parse_args()

    try:
        client = RemoteAPIClient(args.host, args.port)
        sim = client.require('sim')
    except Exception:
        raise SystemExit(f'CoppeliaSim não responde em {args.host}:{args.port}.')

    robot = sim.getObject(args.robot, {'noError': True})
    if robot == -1:
        raise SystemExit(f'não há {args.robot} na cena. Rode scene/build_scene.py.')

    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            time.sleep(0.05)

    # Só os scripts que COMANDAM JUNTAS são calados. Os scripts das rodas
    # precisam continuar rodando: são eles que produzem o efeito de roda sueca.
    # Calar tudo foi o primeiro erro deste ensaio, e o sintoma foi justamente o
    # movimento lateral quase desaparecer.
    silenced = silence_joint_scripts(sim, robot)
    if silenced:
        print('scripts calados:', ', '.join(script_names(sim, silenced)))

    wheels = find_joints(sim, robot, WHEEL_JOINTS, 'juntas de rolamento')
    reference = sim.getObject(f'{args.robot}/youBot_ref')

    print(f'raio da roda {WHEEL_RADIUS:.3f} m | meia-bitola {HALF_TRACK:.3f} m | '
          f'meia-base {HALF_BASE:.3f} m')
    print(f'braço de alavanca do modelo: {HALF_TRACK + HALF_BASE:.3f} m')
    print()

    client.setStepping(True)
    sim.startSimulation()
    for _ in range(20):                 # deixa assentar nas rodas
        sim.step()
    home_pose = sim.getObjectPose(robot, sim.handle_world)

    trials = (
        ('frente', (args.speed, 0.0, 0.0)),
        ('ré', (-args.speed, 0.0, 0.0)),
        ('esquerda', (0.0, args.speed, 0.0)),
        ('direita', (0.0, -args.speed, 0.0)),
        ('giro +', (0.0, 0.0, args.turn)),
        ('giro -', (0.0, 0.0, -args.turn)),
        ('diagonal', (args.speed, args.speed, 0.0)),
    )

    print(f'{"ensaio":10} {"pedido (vf, vl, w)":>24} {"medido (vf, vl, w)":>24} '
          f'{"ganho":>8} {"cruzado":>9}')
    gains, crosstalk = [], []
    for label, command in trials:
        reset_to(sim, robot, home_pose)
        observed, _ = measure(sim, client, wheels, reference, command,
                              args.seconds)
        # Duas medidas separadas, porque dizem coisas diferentes:
        #
        #   GANHO    quanto do pedido o corpo realmente fez, pela projeção do
        #            twist medido sobre o pedido. Em roda sueca ele é menor que
        #            1 por construção (os rolos escorregam): é informação para
        #            o controlador, não erro de modelo.
        #   CRUZADO  o que sobra do twist medido FORA da direção pedida. É este
        #            que acusa modelo errado — pedir "frente" e o robô girar
        #            significa sinal ou alavanca trocados.
        #
        # As três liberdades entram na mesma unidade antes de comparar: o giro
        # vira velocidade de borda multiplicando pelo braço de alavanca. Sem
        # isso, somar m/s com rad/s não quer dizer nada.
        scale = (1.0, 1.0, HALF_TRACK + HALF_BASE)
        want = [command[i] * scale[i] for i in range(3)]
        have = [observed[i] * scale[i] for i in range(3)]
        norm = sum(value * value for value in want)
        gain = sum(have[i] * want[i] for i in range(3)) / norm
        # O resto é a parte do movimento ortogonal ao pedido.
        residual = [have[i] - gain * want[i] for i in range(3)]
        cross = math.sqrt(sum(value * value for value in residual)) / math.sqrt(norm)
        gains.append(gain)
        crosstalk.append(cross)
        print(f'{label:10} ({command[0]:+.3f}, {command[1]:+.3f}, {command[2]:+.3f}) '
              f'    ({observed[0]:+.3f}, {observed[1]:+.3f}, {observed[2]:+.3f}) '
              f' {gain:8.3f} {cross * 100:8.1f} %')

    sim.stopSimulation()
    client.setStepping(False)

    print()
    print(f'ganho médio: {sum(gains) / len(gains):.3f}  '
          f'(mínimo {min(gains):.3f}, máximo {max(gains):.3f})')
    print(f'pior acoplamento cruzado: {max(crosstalk) * 100:.1f} %')
    print()

    if min(gains) <= 0.0:
        print('ERRO: alguma liberdade saiu com o SINAL TROCADO. O padrão de '
              'sinais de kinematics.wheel_velocities não corresponde a esta '
              'base.')
        return 1
    if max(crosstalk) > 0.25:
        print('ERRO: pedir uma liberdade mexeu noutra em mais de 25 %. '
              'Confira o braço de alavanca e a ordem das rodas.')
        return 1

    # A média é dos ensaios de eixo puro: a diagonal escorrega muito mais
    # (duas rodas paradas enquanto duas giram) e puxaria a média para baixo.
    pure = gains[:6]
    average = sum(pure) / len(pure)
    print('O modelo está certo no que importa: cada liberdade sai no sentido '
          'pedido e quase não contamina as outras.')
    print(f'O ganho de {average:.2f} nos eixos puros é o escorregamento dos '
          f'rolos — pôr base_control com cmd_gain={1.0 / average:.2f} faz a '
          'velocidade pedida virar a velocidade andada.')
    print(f'Na diagonal o ganho cai para {gains[6]:.2f}: ali duas rodas ficam '
          'paradas enquanto duas giram, e escorrega mais. É por isso que o '
          'controle é em malha fechada, e não por tempo de marcha.')
    print(f'Acima de ~{MAX_WHEEL_SPEED * WHEEL_RADIUS:.2f} m/s o motor da roda '
          f'satura ({MAX_WHEEL_FORCE:.1f} N.m) e o ganho cai: é o teto que '
          'base_control usa.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
