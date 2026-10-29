#!/usr/bin/env python3
"""O corpo do Microduck, em CoppeliaSim, falando o protocolo que o `robotd` espera.

No repositório do Microduck, `robotd --sim host:porta` troca o barramento dos
servos por `duck_control::sim::RemoteIo` e passa a conversar por TCP com um
processo que tem o corpo. Esse processo é o `duck-body`, em MuJoCo, e mora no
repositório `microduck_rl`. **Este arquivo é o outro lado desse fio**, com o
corpo em CoppeliaSim.

    python3 build_duck.py          # monta o pato na cena
    python3 duck_body.py           # serve o corpo na porta 7801
    robotd --sim 127.0.0.1:7801    # o cérebro do robô, do outro repositório

Tudo acima da costura — o laço de 50 Hz, a política, a segurança, a detecção de
queda, a cinemática, a odometria, o IPC inteiro, o `robotctl` — é o código que
um robô de verdade roda, e não tem como saber a diferença. É a propriedade que
o `sim.rs` do Microduck descreve, e ela vale igual com este corpo.

O PROTOCOLO
-----------
Uma linha de JSON por mensagem, pedido e resposta, em TCP sem Nagle:

    {"op":"hello","protocol":1,"joints":15}  ->  {"protocol":1}
    {"op":"read"}                            ->  {"positions":[15],
                                                  "velocities":[15],
                                                  "currents_ma":[15],
                                                  "imu":{"gyro":[3],
                                                         "gravity":[3],
                                                         "quat":[4]}}
    {"op":"write","targets":[15]}            ->  {}
    {"op":"gain","kp":N}                     ->  {}
    {"op":"torque","on":true|false}          ->  {}
    {"op":"slow"}                            ->  {"volts":f,"temps_c":[15]}

Uma resposta com `{"error": "..."}` é uma recusa que o daemon propaga; qualquer
linha que não parseie derruba a conexão, porque não há como ressincronizar um
protocolo de linha.

QUEM DÁ O PASSO DA SIMULAÇÃO
----------------------------
O `write` é o fim de um tique do robô: ele leu os sensores, rodou a política e
mandou os alvos. Então é no `write` que este servidor adianta o mundo em um
PERÍODO DE CONTROLE (20 ms do robô = quatro passos de 5 ms da cena), e não no
`read`.

A alternativa — deixar a cena correr livre e só amostrar — foi descartada pela
mesma razão que as pontes dos outros trabalhos deste repositório rodam em
stepping: em curso livre o laço de controle fica à mercê da carga da máquina, e
o que se mede deixa de ser o controlador. Aqui o efeito seria pior, porque uma
política de RL foi treinada contra um passo fixo.

A BOCA NÃO EXISTE
-----------------
O protocolo indexa quinze juntas; o MJCF do pato tem catorze. Falta `mouth`,
que nenhuma política alfa aciona. Ela ocupa o índice 9 do vetor e aqui é
reportada parada em zero — o lugar dela tem de continuar existindo, ou os
catorze índices restantes andam um para a esquerda e a observação inteira sai
errada sem nada acusar.

O QUE ESTE CORPO NÃO É
----------------------
Os servos do pato são Dynamixel com um modelo de atuador que as políticas viram
no treino (BAM, no `microduck_rl`). Aqui cada junta é uma junta em controle de
posição do CoppeliaSim, com um limite de torque. É um atuador diferente, e a
diferença aparece onde o atuador importa: uma política de marcha treinada
contra o modelo BAM não tem por que andar igual contra este. O que este corpo
serve para exercitar é tudo o que está ACIMA do atuador.
"""

import argparse
import json
import math
import os
import socket
import socketserver
import sys
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from build_duck import DEFAULT_POSITION, JOINT_ORDER  # noqa: E402

#: Versão do formato do fio, de `duck_control::sim::PROTOCOL`. Se o daemon
#: anunciar outra, os dois lados estão em versões diferentes e o erro tem de
#: citar os dois números, porque "o simulador está velho" e "o daemon está
#: velho" são o mesmo sintoma visto de lados opostos.
PROTOCOL = 1

#: Quantas juntas o protocolo carrega, de `duck_control::model::NUM_JOINTS`.
NUM_JOINTS = 15

#: Período do laço de controle do robô [s]. O `robotd` roda a 50 Hz.
CONTROL_PERIOD = 0.020


class Body:
    """O pato na cena, e o estado que o protocolo precisa manter.

    Uma instância só, usada de uma thread só — ver `Server` para por que o
    servidor não é multithread.
    """

    def __init__(self, host, port, duck='/duck', currents=False):
        self.client = RemoteAPIClient(host, port)
        self.sim = self.client.require('sim')
        sim = self.sim

        self.trunk = self.find(f'{duck}/trunk_base', 'o tronco do pato')
        self.imu = self.find(f'{duck}/trunk_base/imu', 'o IMU do tronco')

        # Uma junta por nome do protocolo; `None` onde o MJCF não tem a junta.
        self.joints = []
        for name in JOINT_ORDER:
            handle = sim.getObject(f':/{name}_joint', {'noError': True})
            self.joints.append(None if handle == -1 else handle)
        missing = [JOINT_ORDER[i] for i, h in enumerate(self.joints)
                   if h is None]

        self.report_currents = currents
        self.targets = list(DEFAULT_POSITION)
        self.torque_on = True
        self.kp = 0
        self.step_time = sim.getFloatParam(
            sim.floatparam_simulation_time_step)
        self.steps_per_tick = max(1, round(CONTROL_PERIOD / self.step_time))
        self.ticks = 0
        self.wall = 0.0

        if sim.getSimulationState() == sim.simulation_stopped:
            self.client.setStepping(True)
            sim.startSimulation()
            # O PRIMEIRO passo custa ~1 s: é onde o CoppeliaSim monta o mundo
            # dinâmico. Pagá-lo aqui, e não no primeiro tique do daemon, evita
            # que o robô comece com um tique mil vezes mais longo que os
            # outros — o tipo de coisa que um detector de queda lê como um
            # salto de estado.
            for _ in range(self.steps_per_tick):
                sim.step()
        self.gravity = sim.getArrayParam(sim.arrayparam_gravity)

        print(f'corpo pronto: {NUM_JOINTS - len(missing)} juntas na cena, '
              f'ausentes: {", ".join(missing) if missing else "nenhuma"}')
        print(f'  passo da cena {self.step_time * 1000:.0f} ms, '
              f'{self.steps_per_tick} por tique de controle '
              f'({CONTROL_PERIOD * 1000:.0f} ms)')

    def find(self, path, what):
        handle = self.sim.getObject(path, {'noError': True})
        if handle == -1:
            raise SystemExit(
                f'não há "{path}" na cena ({what}).\n'
                'Monte o pato com: python3 build_duck.py')
        return handle

    # =========================================================================
    #  Sensores
    # =========================================================================

    def sensors(self):
        """O quadro que o `read` devolve, nas unidades do robô.

        A conversão é feita AQUI, e não do lado do daemon, porque quem conhece
        a ordem das juntas, a escala e o referencial deste modelo é este
        processo. É a mesma divisão que o `sim.rs` do Microduck descreve: o que
        cruza o fio já está nas unidades do robô.
        """
        sim = self.sim
        positions = []
        velocities = []
        for handle in self.joints:
            if handle is None:
                # A boca: presente no vetor, parada em zero.
                positions.append(0.0)
                velocities.append(0.0)
                continue
            positions.append(float(sim.getJointPosition(handle)))
            velocities.append(float(sim.getJointVelocity(handle)))

        matrix = sim.getObjectMatrix(self.imu, sim.handle_world)
        quat = sim.getObjectQuaternion(self.imu, sim.handle_world)
        _, angular = sim.getObjectVelocity(self.trunk)

        return {
            'positions': positions,
            'velocities': velocities,
            **({'currents_ma': self.currents()}
               if self.report_currents else {}),
            'imu': {
                # O protocolo quer a velocidade angular NO REFERENCIAL DO
                # TRONCO; o CoppeliaSim a devolve no do mundo.
                'gyro': rotate_into_body(matrix, angular),
                'gravity': rotate_into_body(matrix, unit(self.gravity)),
                # Escalar primeiro: o CoppeliaSim devolve (x, y, z, w).
                'quat': [quat[3], quat[0], quat[1], quat[2]],
            },
        }

    def currents(self):
        """Corrente de cada servo [mA], do torque que a junta está fazendo.

        Não é a corrente de um Dynamixel — é o torque da junta por uma
        constante plausível. O bloco é OPCIONAL no protocolo, e por padrão não
        vai: medi-lo custa catorze idas e voltas da Remote API por tique, e o
        daemon trata a ausência corretamente (`RobotIo::measures_load` carrega
        a diferença). `--currents` liga para quem quiser.
        """
        sim = self.sim
        amps = []
        for handle in self.joints:
            if handle is None:
                amps.append(0.0)
                continue
            try:
                torque = float(sim.getJointForce(handle))
            except Exception:
                torque = 0.0
            amps.append(torque * 1000.0)
        return amps

    # =========================================================================
    #  As operações do protocolo
    # =========================================================================

    def op_hello(self, message):
        joints = message.get('joints')
        if joints is not None and joints != NUM_JOINTS:
            return {'protocol': PROTOCOL,
                    'error': f'o daemon quer {joints} juntas e este corpo '
                             f'tem {NUM_JOINTS}'}
        return {'protocol': PROTOCOL}

    def op_read(self, message):
        return self.sensors()

    def op_write(self, message):
        targets = message.get('targets')
        if not isinstance(targets, list) or len(targets) != NUM_JOINTS:
            return {'error': f'targets tem de ser uma lista de {NUM_JOINTS} '
                             f'valores'}
        self.targets = [float(value) for value in targets]
        if self.torque_on:
            for handle, target in zip(self.joints, self.targets):
                if handle is not None:
                    self.sim.setJointTargetPosition(handle, target)
        # O write fecha o tique: é aqui que o mundo anda.
        started = time.monotonic()
        for _ in range(self.steps_per_tick):
            self.sim.step()
        self.wall += time.monotonic() - started
        self.ticks += 1
        # O fator de tempo real, como o `duck-sim realtime` do Microduck.
        # Abaixo de 1 o mundo roda devagar, e isso NÃO é um erro: cada tique
        # continua adiantando exatamente um período de controle, e o daemon do
        # robô é quem dá o ritmo. O que seria erro é o passo VARIAR.
        if self.ticks % 50 == 0:
            factor = (self.ticks * CONTROL_PERIOD) / max(self.wall, 1e-9)
            # A posição do tronco é a visão do SIMULADOR, não do robô: o
            # protocolo não a carrega, e não carrega de propósito — um robô de
            # verdade não sabe onde está. Ela vai no log do corpo, que é o
            # equivalente do `duck-sim simlog`, para responder a única
            # pergunta que o robô não pode responder sozinho: ele saiu do
            # lugar?
            where = self.sim.getObjectPosition(self.trunk, self.sim.handle_world)
            print(f'{self.ticks:5d} tiques  {factor:.2f}x tempo real  '
                  f'tronco x={where[0]*100:+6.1f} y={where[1]*100:+6.1f} '
                  f'z={where[2]*100:5.1f} cm')
        return {}

    def op_gain(self, message):
        """O ganho de posição dos servos.

        O Dynamixel tem um Kp em unidades dele; a junta do CoppeliaSim em
        controle de posição não expõe o equivalente. O valor é guardado e
        reportado, e não aplicado — registrar que ele é ignorado é melhor que
        inventar uma conversão que ninguém mediu.
        """
        self.kp = int(message.get('kp', 0))
        return {}

    def op_torque(self, message):
        """Liga e desliga o torque dos servos.

        Desligado, as juntas param de perseguir o alvo e o pato desaba — que é
        o que acontece num robô de verdade quando o torque cai, e é o que o
        daemon espera ao mandar desligar.
        """
        on = bool(message.get('on', True))
        self.torque_on = on
        for handle in self.joints:
            if handle is None:
                continue
            mode = (self.sim.jointdynctrl_position if on
                    else self.sim.jointdynctrl_free)
            self.sim.setObjectInt32Param(
                handle, self.sim.jointintparam_dynctrlmode, mode)
        return {}

    def op_slow(self, message):
        """Tensão e temperaturas: o que um robô lê devagar.

        Valores fixos e plausíveis. Um corpo simulado não tem bateria nem
        servo esquentando, e devolver zeros faria o daemon reportar bateria
        vazia — um alarme falso que esconde os de verdade.
        """
        return {'volts': 8.1, 'temps_c': [32.0] * NUM_JOINTS}

    OPS = {
        'hello': op_hello,
        'read': op_read,
        'write': op_write,
        'gain': op_gain,
        'torque': op_torque,
        'slow': op_slow,
    }

    def handle(self, message):
        op = message.get('op')
        handler = self.OPS.get(op)
        if handler is None:
            return {'error': f'operação desconhecida: {op!r}'}
        return handler(self, message)


# =============================================================================
#  Álgebra
# =============================================================================

def unit(vector):
    norm = math.sqrt(sum(value * value for value in vector))
    return [value / norm for value in vector] if norm else list(vector)


def rotate_into_body(matrix, world_vector):
    """Um vetor do mundo no referencial do corpo.

    `matrix` é a 3x4 do CoppeliaSim em ordem de linhas: as três primeiras
    colunas são a rotação corpo→mundo, então a transposta leva mundo→corpo.
    """
    rows = ((matrix[0], matrix[1], matrix[2]),
            (matrix[4], matrix[5], matrix[6]),
            (matrix[8], matrix[9], matrix[10]))
    return [sum(rows[row][axis] * world_vector[row] for row in range(3))
            for axis in range(3)]


# =============================================================================
#  O servidor
# =============================================================================

class Handler(socketserver.StreamRequestHandler):
    """Uma conexão: uma linha de JSON por vez, resposta na mesma ordem."""

    def setup(self):
        super().setup()
        # Nagle atrasaria uma escrita pequena esperando mais o que enviar, até
        # uns 40 ms — o dobro de um tique. O `sim.rs` do Microduck desliga do
        # lado dele e diz que seria catastrófico e silencioso; vale igual aqui.
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print(f'daemon conectado de {self.client_address[0]}')

    def handle(self):
        body = self.server.body
        for line in self.rfile:
            line = line.strip()
            if not line:
                continue
            try:
                message = json.loads(line)
            except ValueError as error:
                # Sem como ressincronizar um protocolo de linha: cai fora.
                print(f'linha ilegível, encerrando a conexão: {error}')
                return
            try:
                answer = body.handle(message)
            except Exception as error:      # noqa: BLE001
                answer = {'error': f'{type(error).__name__}: {error}'}
            self.wfile.write((json.dumps(answer) + '\n').encode())
            self.wfile.flush()

    def finish(self):
        print(f'daemon desconectado ({self.server.body.ticks} tiques)')
        try:
            super().finish()
        except Exception:       # noqa: BLE001
            pass


class Server(socketserver.TCPServer):
    """Uma conexão por vez, numa thread só — e isso não é simplificação.

    O cliente ZeroMQ do CoppeliaSim guarda um socket, e socket ZeroMQ não é
    thread-safe. Com um `ThreadingTCPServer` o handler roda numa thread nova e
    a primeira chamada à Remote API feita dela TRAVA, sem erro e sem log: foi
    exatamente o que aconteceu aqui, e o sintoma foi o daemon esperando a
    resposta de um `torque` que nunca veio.

    E o robô abre uma conexão só. Servir uma por vez é o que o caso de uso
    pede, e é o que remove a classe inteira de problema.
    """

    allow_reuse_address = True


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--bind', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=7801,
                        help='a porta que o robotd --sim procura')
    parser.add_argument('--sim-host', default='localhost')
    parser.add_argument('--sim-port', type=int, default=23000,
                        help='a ZeroMQ Remote API do CoppeliaSim')
    parser.add_argument('--currents', action='store_true',
                        help='reporta a carga de cada servo; custa catorze '
                             'idas e voltas por tique e o protocolo permite '
                             'omitir o bloco')
    parser.add_argument('--no-display', action='store_true',
                        help='desliga o desenho do CoppeliaSim durante a '
                             'corrida: o passo de física fica 3x mais rápido')
    args = parser.parse_args()

    body = Body(args.sim_host, args.sim_port, currents=args.currents)
    if args.no_display:
        body.sim.setBoolParam(body.sim.boolparam_display_enabled, False)
        print('desenho desligado')
    server = Server((args.bind, args.port), Handler)
    server.body = body
    print(f'corpo do pato em {args.bind}:{args.port} — '
          f'aponte o daemon com: robotd --sim {args.bind}:{args.port}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\nencerrando')
    finally:
        server.server_close()
        try:
            body.sim.setBoolParam(body.sim.boolparam_display_enabled, True)
            body.sim.stopSimulation()
            body.client.setStepping(False)
        except Exception:       # noqa: BLE001
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
