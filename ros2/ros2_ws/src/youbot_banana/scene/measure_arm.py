#!/usr/bin/env python3
"""Mede a geometria do braço na cena e confere a cinemática contra ela.

De onde saíram as constantes de `kinematics.py`: este script zera as juntas do
braço, lê as posições pela Remote API e imprime os valores; depois comanda
configurações aleatórias e compara a posição que o MODELO prevê para a ponta da
garra com a que o SIMULADOR mede. É a prova de que a cinemática está certa, e é
o que se refaz se o modelo do youBot mudar de versão.

    python3 scene/measure_arm.py              # mede e confere
    python3 scene/measure_arm.py --trials 50  # mais configurações

Não precisa da simulação rodando: as juntas são postas por `setJointPosition`,
que funciona com a cena parada (é cinemática pura, sem física). Precisa apenas
de uma cena aberta com um /youBot dentro.
"""

import argparse
import math
import os
import random
import sys

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from youbot_banana.kinematics import (  # noqa: E402  (precisa do sys.path acima)
    ARM, Unreachable, arm_fk, arm_ik_best)


def connect(host, port):
    try:
        return RemoteAPIClient(host, port).require('sim')
    except Exception:
        raise SystemExit(f'CoppeliaSim não responde em {host}:{port}. '
                         'Ele está aberto?')


def measure(sim, robot):
    """Imprime a geometria do braço lida da cena, com as juntas em zero."""
    ref = sim.getObject(f'{robot}/youBot_ref')
    joints = [sim.getObject(f'{robot}/youBotArmJoint{i}') for i in range(5)]
    grippers = [sim.getObject(f'{robot}/youBotGripperJoint1'),
                sim.getObject(f'{robot}/youBotGripperJoint2')]
    pads = [sim.getObject(f'{robot}/Rectangle2'), sim.getObject(f'{robot}/Rectangle3')]

    for joint in joints:
        sim.setJointPosition(joint, 0.0)
    # A garra tem de ficar SIMÉTRICA, ou o ponto médio das pastilhas (que é o
    # ponto de preensão) sai deslocado e a conferência acusa um erro constante
    # que não existe. Foi exatamente o que aconteceu na primeira medição.
    for joint, value in zip(grippers, ARM.gripper_open):
        sim.setJointPosition(joint, value)

    print('=== juntas do braço no frame youBot_ref, todas em zero ===')
    frames = []
    for index, joint in enumerate(joints):
        position = sim.getObjectPosition(joint, ref)
        matrix = sim.getObjectMatrix(joint, ref)
        axis = (matrix[2], matrix[6], matrix[10])   # eixo do giro = z do frame
        frames.append(position)
        print(f'  junta {index}: pos=({position[0]:+.4f}, {position[1]:+.4f}, '
              f'{position[2]:+.4f})  eixo=({axis[0]:+.2f}, {axis[1]:+.2f}, {axis[2]:+.2f})')

    tip = tip_position(sim, pads, ref)
    print(f'  ponta da garra: ({tip[0]:+.4f}, {tip[1]:+.4f}, {tip[2]:+.4f})')

    print()
    print('=== o que isso dá para kinematics.py ===')
    print(f'  axis_y        = {frames[0][1]:+.4f}   (eixo da junta 0 à frente do centro)')
    print(f'  shoulder_r    = {frames[1][1] - frames[0][1]:+.4f}')
    print(f'  shoulder_z    = {frames[1][2]:+.4f}')
    upper = math.dist(frames[1][1:], frames[2][1:])
    fore = math.dist(frames[2][1:], frames[3][1:])
    print(f'  upper_arm     = {upper:+.4f}   (junta 1 -> 2, no plano y-z)')
    print(f'  forearm       = {fore:+.4f}   (junta 2 -> 3)')
    wrist = (math.dist(frames[3][1:], frames[4][1:])
             + math.dist(frames[4][1:], tip[1:]))
    print(f'  wrist_to_tip  = {wrist:+.4f}   (junta 3 -> 4 -> ponta, colineares)')
    print(f'  tool_offset   = {tip[0]:+.4f}   (desvio da ponta do plano do braço)')

    print()
    print('=== intervalos das juntas lidos da cena ===')
    for index, joint in enumerate(joints + grippers):
        cyclic, interval = sim.getJointInterval(joint)
        name = f'junta {index}' if index < 5 else f'garra {index - 4}'
        print(f'  {name}: [{interval[0]:+.3f}, {interval[0] + interval[1]:+.3f}] '
              f'{"cíclica" if cyclic else ""}')

    return ref, joints, pads


def tip_position(sim, pads, ref):
    """Ponto de preensão: o meio das duas pastilhas de borracha dos dedos.

    O ponto médio não depende de quanto a garra está aberta (as duas pastilhas
    se afastam simetricamente do eixo da ferramenta), então serve de referência
    da ponta em qualquer abertura.
    """
    left, right = (sim.getObjectPosition(pad, ref) for pad in pads)
    return tuple((left[i] + right[i]) / 2.0 for i in range(3))


def check_fk(sim, ref, joints, pads, trials, seed):
    """Compara a cinemática direta do módulo com a posição medida na cena."""
    print()
    print('=== cinemática direta: modelo vs. simulador ===')
    random.seed(seed)
    worst = 0.0
    for trial in range(trials):
        q = ([0.0] * 5 if trial == 0
             else [random.uniform(low, high) for low, high in ARM.limits])
        for joint, value in zip(joints, q):
            sim.setJointPosition(joint, value)
        model = arm_fk(q)
        measured = tip_position(sim, pads, ref)
        error = math.dist(model, measured)
        worst = max(worst, error)
        if trial < 6 or error > 2e-3:
            print('  q = ' + ' '.join(f'{v:+.2f}' for v in q)
                  + f' | modelo ({model[0]:+.4f}, {model[1]:+.4f}, {model[2]:+.4f})'
                  + f' | medido ({measured[0]:+.4f}, {measured[1]:+.4f}, {measured[2]:+.4f})'
                  + f' | {error * 1000:.2f} mm')
    print(f'  pior erro em {trials} configurações: {worst * 1000:.2f} mm')
    return worst


def check_ik(sim, ref, joints, pads, trials, seed):
    """Pede à inversa pontos conhecidos e mede onde a garra realmente parou.

    É a conferência que importa para a tarefa: o alvo é um ponto no espaço (a
    banana), e o que se quer saber é a que distância dele a garra fica.
    """
    print()
    print('=== cinemática inversa: alvo pedido vs. garra medida ===')
    random.seed(seed + 1)
    worst, solved = 0.0, 0
    for _ in range(trials):
        azimuth = random.uniform(-1.5, 1.5)
        radius = random.uniform(0.16, 0.29)     # envelope útil, medido abaixo
        height = random.uniform(0.01, 0.45)
        target = (-math.sin(azimuth) * radius,
                  ARM.axis_y + math.cos(azimuth) * radius,
                  height)
        try:
            q = arm_ik_best(target, approach=math.pi)
        except Unreachable:
            continue
        solved += 1
        for joint, value in zip(joints, q):
            sim.setJointPosition(joint, value)
        measured = tip_position(sim, pads, ref)
        error = math.dist(target, measured)
        worst = max(worst, error)
        if solved <= 6 or error > 2e-3:
            print(f'  alvo ({target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f})'
                  f' | medido ({measured[0]:+.3f}, {measured[1]:+.3f}, {measured[2]:+.3f})'
                  f' | {error * 1000:.2f} mm')
    print(f'  {solved}/{trials} com solução | pior erro: {worst * 1000:.2f} mm')
    return worst


def envelope(sim, ref, joints, pads, height):
    """Em que faixa de raio o braço consegue pegar algo nessa altura.

    É o número que o nó de missão usa para decidir onde parar a base: fora
    desta faixa a inversa não tem solução, por limite de junta (perto) ou por
    alcance (longe).
    """
    print()
    print(f'=== envelope de preensão por cima, a z = {height:.3f} m ===')
    ok = []
    radius = 0.08
    while radius <= 0.42:
        target = (0.0, ARM.axis_y + radius, height)
        try:
            arm_ik_best(target, approach=math.pi)
            ok.append(radius)
        except Unreachable:
            pass
        radius += 0.005
    if ok:
        print(f'  raio útil: {min(ok):.3f} m a {max(ok):.3f} m '
              f'(do eixo da junta 0); meio em {(min(ok) + max(ok)) / 2:.3f} m')
        print(f'  a base para a {(min(ok) + max(ok)) / 2 + ARM.axis_y:.3f} m do objeto, '
              'medido do centro do robô')
    else:
        print('  nenhum raio tem solução nessa altura')
    for joint in joints:
        sim.setJointPosition(joint, 0.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=23000)
    parser.add_argument('--robot', default='/youBot')
    parser.add_argument('--trials', type=int, default=20)
    parser.add_argument('--seed', type=int, default=1)
    parser.add_argument('--grasp-height', type=float, default=0.02,
                        help='altura do ponto de preensão de uma banana no chão [m]')
    args = parser.parse_args()

    sim = connect(args.host, args.port)
    if sim.getObject(args.robot, {'noError': True}) == -1:
        raise SystemExit(f'não há {args.robot} na cena aberta. '
                         'Rode scene/build_scene.py primeiro.')

    ref, joints, pads = measure(sim, args.robot)
    fk_error = check_fk(sim, ref, joints, pads, args.trials, args.seed)
    ik_error = check_ik(sim, ref, joints, pads, args.trials, args.seed)
    envelope(sim, ref, joints, pads, args.grasp_height)

    print()
    tolerance = 3e-3
    if max(fk_error, ik_error) > tolerance:
        print(f'ATENÇÃO: erro acima de {tolerance * 1000:.0f} mm. A geometria de '
              'kinematics.py não corresponde a este modelo de youBot: '
              'copie os valores impressos acima.')
        return 1
    print(f'cinemática confere com a cena (erro < {tolerance * 1000:.0f} mm).')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
