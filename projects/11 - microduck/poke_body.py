#!/usr/bin/env python3
"""Exercita o corpo pelo protocolo, no lugar do `robotd`.

O `robotd` é um daemon em Rust, e construí-lo traz o workspace inteiro junto —
ONNX Runtime, GStreamer, quinze caixas. Este arquivo faz o que ele faz no fio:
`hello`, e depois um laço de `read` → decidir → `write` a 50 Hz. Serve para
duas coisas que valem por si:

1. **Verificar a costura.** Se o pato se mexe daqui, o protocolo está certo, as
   quinze juntas estão na ordem certa e o IMU está no referencial certo. O que
   faltar depois é o cérebro, não o corpo.
2. **Ver o corpo sozinho.** Uma varredura de junta mostra limites, sentido de
   giro e acoplamento — as coisas que um modelo montado a partir de um MJCF sem
   geometria pode ter errado e que nenhum teste estático pega.

    python3 poke_body.py                  # pose de repouso por 5 s
    python3 poke_body.py --sweep left_knee
    python3 poke_body.py --sweep head_yaw --seconds 8
    python3 poke_body.py --limp           # torque desligado: o pato desaba

O que ele NÃO faz é equilibrar. Isso é uma política de RL treinada no
`microduck_rl`, e é a peça que o `robotd` carrega.
"""

import argparse
import json
import math
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from build_duck import DEFAULT_POSITION, JOINT_ORDER  # noqa: E402

PROTOCOL = 1
NUM_JOINTS = 15
PERIOD = 0.020


class Link:
    """A conexão com o corpo: uma linha de JSON por mensagem."""

    def __init__(self, host, port):
        self.socket = socket.create_connection((host, port), timeout=5.0)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.file = self.socket.makefile('rwb')

    def call(self, **message):
        self.file.write((json.dumps(message) + '\n').encode())
        self.file.flush()
        line = self.file.readline()
        if not line:
            raise SystemExit('o corpo fechou a conexão')
        answer = json.loads(line)
        if isinstance(answer, dict) and answer.get('error'):
            raise SystemExit(f'o corpo recusou {message.get("op")}: '
                             f'{answer["error"]}')
        return answer

    def close(self):
        try:
            self.file.close()
            self.socket.close()
        except OSError:
            pass


def describe(sensors):
    """Uma linha com o que o quadro de sensores diz."""
    imu = sensors['imu']
    gravity = imu['gravity']
    # Em pé, a gravidade projetada no tronco é [0, 0, -1]; o ângulo entre ela e
    # esse vetor é o quanto o pato está tombado, e é exatamente o sinal em que
    # a detecção de queda do robô se apoia.
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, -gravity[2]))))
    return (f'inclinação {tilt:5.1f}°  '
            f'gravidade ({gravity[0]:+.2f},{gravity[1]:+.2f},{gravity[2]:+.2f})'
            f'  giro ({imu["gyro"][0]:+.2f},{imu["gyro"][1]:+.2f},'
            f'{imu["gyro"][2]:+.2f}) rad/s')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=7801)
    parser.add_argument('--seconds', type=float, default=5.0)
    parser.add_argument('--sweep', metavar='JUNTA',
                        help='varre uma junta entre os seus limites; '
                             f'nomes: {", ".join(JOINT_ORDER)}')
    parser.add_argument('--amplitude', type=float, default=0.6,
                        help='amplitude da varredura [rad]')
    parser.add_argument('--limp', action='store_true',
                        help='desliga o torque: o pato desaba')
    args = parser.parse_args()

    if args.sweep and args.sweep not in JOINT_ORDER:
        raise SystemExit(f'junta "{args.sweep}" não existe. '
                         f'Conhecidas: {", ".join(JOINT_ORDER)}')

    link = Link(args.host, args.port)
    hello = link.call(op='hello', protocol=PROTOCOL, joints=NUM_JOINTS)
    if hello.get('protocol') != PROTOCOL:
        raise SystemExit(
            f'o corpo fala protocolo {hello.get("protocol")} e este cliente '
            f'fala {PROTOCOL} — um dos dois está desatualizado')
    print(f'conectado: protocolo {PROTOCOL}')

    slow = link.call(op='slow')
    print(f'bateria {slow["volts"]:.1f} V, '
          f'servo mais quente {max(slow["temps_c"]):.0f} °C')

    link.call(op='gain', kp=900)
    link.call(op='torque', on=not args.limp)
    if args.limp:
        print('torque DESLIGADO: o pato vai desabar, que é o esperado')

    index = JOINT_ORDER.index(args.sweep) if args.sweep else None
    targets = list(DEFAULT_POSITION)
    started = time.monotonic()
    reported = 0.0
    ticks = 0

    while True:
        elapsed = time.monotonic() - started
        if elapsed >= args.seconds:
            break
        sensors = link.call(op='read')
        if index is not None:
            # Uma senoide em torno da pose de repouso: percorre a faixa toda
            # sem saltos, que é o que um salto de alvo faria a junta fazer na
            # velocidade máxima do servo.
            targets[index] = (DEFAULT_POSITION[index]
                              + args.amplitude * math.sin(2 * math.pi * 0.5
                                                          * elapsed))
        link.call(op='write', targets=targets)
        ticks += 1

        if elapsed - reported >= 0.5:
            reported = elapsed
            line = f't={elapsed:4.1f}s  {describe(sensors)}'
            if index is not None:
                line += (f'  {args.sweep}: alvo {targets[index]:+.3f} '
                         f'lido {sensors["positions"][index]:+.3f} rad')
            print(line)
        time.sleep(max(0.0, PERIOD - (time.monotonic() - started - elapsed)))

    sensors = link.call(op='read')
    print(f'\n{ticks} tiques em {args.seconds:.0f} s '
          f'({ticks / args.seconds:.0f} Hz)')
    print(f'final: {describe(sensors)}')
    link.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
