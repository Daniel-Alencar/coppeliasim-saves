#!/usr/bin/env python3
"""Faz o Microduck andar: a política de marcha do robô, contra o corpo no CoppeliaSim.

O cérebro do pato é o `robotd`, em Rust. Construí-lo traz o workspace inteiro
junto — ONNX Runtime, GStreamer, quinze caixas. Este arquivo refaz em Python a
**parte do tique que faz o robô andar**, que é pequena e está toda escrita no
repositório do Microduck:

    robotd/src/control.rs      alvos ← pose de repouso + escala × ação
    duck-control/src/obs.rs    o vetor de observação, 61 floats
    duck-control/src/policy.rs quando a rede de postura entra no lugar da marcha

e roda a política de verdade — `velstand.onnx`, do conjunto que o robô baixa do
Hugging Face — contra o corpo servido por `duck_body.py`.

    python3 build_duck.py
    python3 duck_body.py --no-display
    python3 walk.py                      # anda para a frente
    python3 walk.py --vx 0 --vyaw 1.0    # gira no lugar
    python3 walk.py --seconds 20 --vx 0.15

O QUE ISTO NÃO É
----------------
Não é o `robotd`. Falta tudo o que ele faz em volta: a camada de segurança, a
detecção de queda, o IPC, as habilidades (chute, levantar, sentar), a
adaptação por tensão, o escalonamento entre redes. O que está aqui é a espinha
— observação, rede, alvos — e ela existe para responder uma pergunta só: **a
política anda neste corpo?**

O VETOR DE OBSERVAÇÃO É A PARTE PERIGOSA
-----------------------------------------
O `obs.rs` do Microduck abre dizendo que é o código de maior risco do pacote:
61 floats em que cada índice tem de bater com o que a política viu no treino, e
um deslocamento errado não falha — produz um robô plausível que cai, e o
sintoma parece problema de ajuste ou de tempo, não de índice.

Por isso o layout está escrito aqui ao lado do código que o preenche, copiado
da tabela do `obs.rs`:

    0..3     3   giro, referencial do tronco, rad/s
    3..6     3   gravidade projetada, referencial do tronco, unitário
    6..20   14   posição da junta menos a pose de repouso, sem a boca
    20..34  14   velocidade da junta, sem a boca
    34..48  14   ação anterior, sem a boca
    48..61  13   comando:
                 48..51  vx, vy, vyaw
                 51..55  neck_pitch, head_pitch, head_yaw, head_roll
                 55..57  corpo x, y   — sempre zero, livres no treino
                 57      corpo z
                 58      corpo roll
                 59      corpo pitch
                 60      corpo yaw    — sempre zero, livre no treino

Duas armadilhas que o `obs.rs` nomeia: **x, y e yaw do corpo são zero de
propósito** (é a codificação nominal, não um lugar vago), e **o alvo da cabeça
viaja no comando**, não é somado à saída da política — fazer os dois dobraria a
cabeça duas vezes.
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np
import onnxruntime as ort

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from build_duck import DEFAULT_POSITION  # noqa: E402
from poke_body import Link  # noqa: E402

PROTOCOL = 1
NUM_JOINTS = 15
OBS_LEN = 61
ACTION_LEN = 14
PERIOD = 0.020

#: O índice da boca, de `duck_control::model::MOUTH_INDEX`. A política não a
#: aciona: o vetor de ação tem catorze valores e salta este lugar.
MOUTH_INDEX = 9

#: Escala da saída da política, de `robotd::control::Tuning::action_scale`.
#:
#: 0,9 e não 1,0, e não é arbitrário: é o padrão alfa do protótipo. A política
#: de postura roda a 1,0 (`standing_action_scale`), mas a `velstand` é uma rede
#: só para andar e parar — `Policy::will_stand` exige uma rede de postura
#: SEPARADA, que não existe nesse conjunto — então aqui a escala é sempre a de
#: marcha.
ACTION_SCALE = 0.9

#: Filtros de primeira ordem sobre os alvos, de `robotd::control::Tuning`.
#: `alpha` é o peso do alvo novo: quanto menor, mais suave e mais atrasado.
HEAD_LOWPASS = 0.5
LEGS_LOWPASS = 0.7

#: As juntas da cabeça, de `robotd::control::HEAD_JOINTS`.
HEAD_JOINTS = range(5, 9)


def joint_of(slot):
    """A junta a que o slot `slot` da política se refere.

    Abaixo da boca os índices passam direto; a partir dela, tudo sobe um. É o
    `joint_of` do `obs.rs`, e a mesma função serve nas duas direções — ler as
    catorze de quinze e espalhar as catorze de volta — de modo que as duas não
    possam discordar sobre onde mora o n-ésimo valor da política.
    """
    return slot if slot < MOUTH_INDEX else slot + 1


#: Os catorze slots da política, como índices de junta. Calculado uma vez.
SLOTS = [joint_of(slot) for slot in range(ACTION_LEN)]


def policy_joints(values):
    """Os catorze valores que a política vê, de um vetor de quinze."""
    return np.array([values[joint] for joint in SLOTS], dtype=np.float32)


def scatter_action(action):
    """Os catorze valores da política de volta em quinze juntas, boca em zero."""
    out = np.zeros(NUM_JOINTS, dtype=np.float64)
    for slot, joint in enumerate(SLOTS):
        out[joint] = action[slot]
    return out


class Pilot:
    """Observação, rede e alvos: o miolo do tique de controle do robô."""

    def __init__(self, policy_path):
        self.session = ort.InferenceSession(
            policy_path, providers=['CPUExecutionProvider'])
        inputs = self.session.get_inputs()
        outputs = self.session.get_outputs()
        if len(inputs) != 1 or list(inputs[0].shape) != [1, OBS_LEN]:
            raise SystemExit(
                f'esta política quer {inputs[0].shape}; o contrato alfa é '
                f'[1, {OBS_LEN}] -> [1, {ACTION_LEN}]')
        if list(outputs[0].shape) != [1, ACTION_LEN]:
            raise SystemExit(
                f'esta política devolve {outputs[0].shape}; o contrato alfa é '
                f'[1, {ACTION_LEN}]')
        self.input_name = inputs[0].name
        self.home = np.array(DEFAULT_POSITION, dtype=np.float64)
        self.home_slots = policy_joints(self.home)
        self.last_action = np.zeros(ACTION_LEN, dtype=np.float32)
        self.previous_targets = None

    def observe(self, sensors, command):
        """Os 61 floats, na ordem da tabela do `obs.rs`."""
        imu = sensors['imu']
        obs = np.empty(OBS_LEN, dtype=np.float32)
        obs[0:3] = imu['gyro']
        obs[3:6] = imu['gravity']
        obs[6:20] = policy_joints(sensors['positions']) - self.home_slots
        obs[20:34] = policy_joints(sensors['velocities'])
        obs[34:48] = self.last_action
        obs[48:61] = command
        return obs

    def step(self, sensors, command):
        """Um tique: sensores e comando entram, quinze alvos saem."""
        obs = self.observe(sensors, command)
        action = self.session.run(
            None, {self.input_name: obs.reshape(1, OBS_LEN)})[0][0]
        self.last_action = action.astype(np.float32)

        targets = self.home + ACTION_SCALE * scatter_action(action)

        # Os filtros do `control.rs`. Eles existem porque a saída da rede é
        # ruidosa a 50 Hz e um servo persegue esse ruído: a cabeça é filtrada
        # mais forte que as pernas porque ela não precisa de banda e balançar a
        # cabeça move o IMU, que é entrada da própria política.
        if self.previous_targets is not None:
            for joint in range(NUM_JOINTS):
                if joint == MOUTH_INDEX:
                    continue
                alpha = (HEAD_LOWPASS if joint in HEAD_JOINTS
                         else LEGS_LOWPASS)
                targets[joint] = (alpha * targets[joint]
                                  + (1.0 - alpha) * self.previous_targets[joint])
        self.previous_targets = targets.copy()
        return targets


def command_block(vx, vy, vyaw, head=(0.0, 0.0, 0.0, 0.0),
                  body=(0.0, 0.0, 0.0)):
    """Os treze floats do comando. `body` é (z, roll, pitch), nessa ordem.

    A ordem dentro do bloco do corpo é `z, roll, pitch`, e não
    `z, pitch, roll` — o `obs.rs` chama atenção para isso, e é o tipo de troca
    que não falha, só faz o robô andar torto.
    """
    return [vx, vy, vyaw, head[0], head[1], head[2], head[3],
            0.0, 0.0, body[0], body[1], body[2], 0.0]


def tilt_degrees(gravity):
    """Quanto o tronco está fora da vertical [graus]. Em pé é zero."""
    return math.degrees(math.acos(max(-1.0, min(1.0, -gravity[2]))))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=7801)
    parser.add_argument('--policy',
                        default=os.path.join(HERE, 'policies', 'velstand.onnx'),
                        help='a política ONNX; o padrão é a de marcha')
    parser.add_argument('--vx', type=float, default=0.15,
                        help='velocidade para a frente [m/s]')
    parser.add_argument('--vy', type=float, default=0.0)
    parser.add_argument('--vyaw', type=float, default=0.0,
                        help='velocidade de guinada [rad/s]')
    parser.add_argument('--settle', type=float, default=2.0,
                        help='segundos de comando ZERO antes de andar, para a '
                             'política assentar o pato de pé')
    parser.add_argument('--seconds', type=float, default=15.0)
    args = parser.parse_args()

    if not os.path.exists(args.policy):
        raise SystemExit(
            f'não achei a política em {args.policy}.\n'
            'Baixe o conjunto oficial:\n'
            '  mkdir -p policies && curl -L -o policies/velstand.onnx \\\n'
            '    https://huggingface.co/pollen-robotics/microduck-policies'
            '/resolve/main/velstand.onnx')

    manifest_path = os.path.join(os.path.dirname(args.policy), 'manifest.json')
    if os.path.exists(manifest_path):
        with open(manifest_path, encoding='utf-8') as handle:
            manifest = json.load(handle)
        print(f'conjunto de políticas: obs {manifest.get("obs_len")}, '
              f'ações {manifest.get("action_len")}, '
              f'{manifest.get("robot", {}).get("control_hz")} Hz')

    pilot = Pilot(args.policy)
    link = Link(args.host, args.port)
    hello = link.call(op='hello', protocol=PROTOCOL, joints=NUM_JOINTS)
    if hello.get('protocol') != PROTOCOL:
        raise SystemExit(f'o corpo fala protocolo {hello.get("protocol")}')
    link.call(op='torque', on=True)
    print(f'política carregada; {args.settle:.0f} s parado e depois '
          f'vx={args.vx:+.2f} m/s, vyaw={args.vyaw:+.2f} rad/s')

    started = time.monotonic()
    reported = -1.0
    ticks = 0
    walking = False
    while True:
        elapsed = time.monotonic() - started
        if elapsed >= args.seconds:
            break
        # O pato assenta com comando ZERO: a `velstand` fica parada em pé nessa
        # condição, e sair andando de uma pose solta é pedir uma queda.
        if elapsed < args.settle:
            command = command_block(0.0, 0.0, 0.0)
        else:
            if not walking:
                walking = True
                print('--- andando ---')
            command = command_block(args.vx, args.vy, args.vyaw)

        sensors = link.call(op='read')
        targets = pilot.step(sensors, command)
        link.call(op='write', targets=[float(v) for v in targets])
        ticks += 1

        if elapsed - reported >= 1.0:
            reported = elapsed
            gravity = sensors['imu']['gravity']
            print(f't={elapsed:5.1f}s  inclinação {tilt_degrees(gravity):5.1f}°'
                  f'  ação |max| {np.abs(pilot.last_action).max():5.2f}'
                  f'  {"andando" if walking else "assentando"}')
        time.sleep(max(0.0, PERIOD - (time.monotonic() - started - elapsed)))

    sensors = link.call(op='read')
    print(f'\n{ticks} tiques em {args.seconds:.0f} s '
          f'({ticks / args.seconds:.0f} Hz)')
    print(f'final: inclinação {tilt_degrees(sensors["imu"]["gravity"]):.1f}°')
    link.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
