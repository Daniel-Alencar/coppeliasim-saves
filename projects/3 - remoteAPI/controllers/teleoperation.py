"""Teleoperação do /myRobot pelo teclado, direto pela ZeroMQ Remote API (sem ROS).

Lê as teclas no próprio terminal com curses e envia as velocidades das rodas ao
CoppeliaSim. Uso, com a cena aberta no CoppeliaSim:

    python3 teleoperation.py

W/S ou setas para cima/baixo: frente e ré | A/D ou setas laterais: girar
Espaço: parar | Q, Esc ou Ctrl+C: sair

Não rode junto com teleoperation_with_ros2.py: os dois escrevem nos mesmos
motores e um anula o outro.
"""

import curses  # leitura de teclas e desenho de texto no terminal
import time    # time.monotonic: relógio que nunca volta, ideal para prazos

# Coppelia ZeroMQ Remote API: permite chamar as funções sim.* do CoppeliaSim a
# partir de um processo externo, pela porta TCP 23000.
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Velocidades das rodas em rad/s. Com rodas de 5 cm de raio, SPEED dá 0,3 m/s.
SPEED = 6.0
# Nos giros as rodas andam em sentidos opostos, então o robô gira no lugar.
TURN = 3.0
# O terminal não avisa quando uma tecla é solta, então o robô para depois desse
# tempo sem tecla de movimento. O valor precisa ser maior que o atraso de
# repetição do teclado (500 ms por padrão); senão, ao segurar a tecla, o robô
# dá um tranco antes de a repetição começar.
INPUT_TIMEOUT = 0.6
# Código que o terminal entrega para Ctrl+C quando está em modo raw.
CTRL_C = 3


class car:
    def __init__(self):
        # Conecta ao CoppeliaSim (localhost:23000) e obtém o objeto sim, que
        # oferece as mesmas funções sim.* dos scripts da cena.
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        # Handles: identificadores dos objetos da cena usados nas chamadas sim.*.
        self.leftMotorHandle = self.sim.getObject('/myRobot/leftMotor')
        self.rightMotorHandle = self.sim.getObject('/myRobot/rightMotor')
        self.sensorNariz = self.sim.getObject('/myRobot/proximitySensor')
        # Só para a simulação ao sair se foi este programa que deu o play.
        started_here = self.sim.getSimulationState() == self.sim.simulation_stopped
        try:
            if started_here:
                self.sim.startSimulation()
            # curses.wrapper prepara o terminal, chama self.control(screen) e
            # restaura o terminal ao sair, mesmo que ocorra um erro.
            curses.wrapper(self.control)
        finally:
            # try/finally encadeados: cada comando de parada é tentado mesmo que
            # o anterior falhe, para o robô não ficar andando sozinho.
            try:
                self.sim.setJointTargetVelocity(self.leftMotorHandle, 0.0)
            finally:
                try:
                    self.sim.setJointTargetVelocity(self.rightMotorHandle, 0.0)
                finally:
                    if started_here:
                        self.sim.stopSimulation()

    def control(self, screen):
        """Laço principal: lê teclas, aplica as velocidades e mostra o estado."""
        # O modo raw entrega o Ctrl+C como tecla em vez de KeyboardInterrupt,
        # que poderia abortar uma chamada ZeroMQ no meio e deixar o socket
        # inutilizável para os comandos finais de parada.
        curses.raw()
        # Faz as setas chegarem como códigos únicos (curses.KEY_UP etc.).
        screen.keypad(True)
        # getch espera no máximo 50 ms por uma tecla e devolve -1 se nada chegar.
        # Assim o laço continua rodando (~20 Hz) mesmo sem teclas.
        screen.timeout(50)
        # Pares (roda esquerda, roda direita) de cada movimento. Para virar à
        # esquerda, a roda esquerda gira para trás e a direita para frente.
        forward, backward = (SPEED, SPEED), (-SPEED, -SPEED)
        left, right = (-TURN, TURN), (TURN, -TURN)
        # Tecla -> velocidades. As maiúsculas cobrem o Caps Lock ligado.
        commands = {
            ord('w'): forward, ord('W'): forward, curses.KEY_UP: forward,
            ord('s'): backward, ord('S'): backward, curses.KEY_DOWN: backward,
            ord('a'): left, ord('A'): left, curses.KEY_LEFT: left,
            ord('d'): right, ord('D'): right, curses.KEY_RIGHT: right,
        }
        velocity = (0.0, 0.0)  # comando atual (esquerda, direita)
        deadline = 0.0         # instante em que o comando atual expira
        # Roda até o usuário sair ou até a simulação ser parada no CoppeliaSim.
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            key = screen.getch()
            # 27 é o código da tecla Esc.
            if key in (ord('q'), ord('Q'), 27, CTRL_C):
                break
            if key == ord(' '):
                # Parada imediata. flushinp descarta as repetições de tecla que
                # ainda estão na fila, senão o robô voltaria a andar.
                velocity = (0.0, 0.0)
                deadline = 0.0
                curses.flushinp()
            elif key in commands:
                # Cada tecla, inclusive as repetições ao segurar, renova o prazo.
                velocity = commands[key]
                deadline = time.monotonic() + INPUT_TIMEOUT
            # Prazo vencido significa que nenhuma tecla chegou recentemente: parar.
            if time.monotonic() >= deadline:
                velocity = (0.0, 0.0)
            # Envia a velocidade alvo (rad/s) de cada motor a cada volta do laço.
            self.sim.setJointTargetVelocity(self.leftMotorHandle, velocity[0])
            self.sim.setJointTargetVelocity(self.rightMotorHandle, velocity[1])
            # Leitura do sensor: (detectou, distância, ponto detectado,
            # handle do objeto, normal da superfície).
            sensor = self.sim.readProximitySensor(self.sensorNariz)
            # Redesenha a tela do zero a cada ciclo.
            screen.erase()
            lines = [
                'WASD / arrows: drive | Space: stop | Q / Esc / Ctrl+C: quit',
                f'Hold a movement key to repeat. Input timeout: {INPUT_TIMEOUT} seconds.',
                f'Wheel velocities: {velocity}',
                f'Proximity sensor: {sensor}',
            ]
            if self.sim.getSimulationState() == self.sim.simulation_paused:
                lines.append('Simulation is paused: press play in CoppeliaSim.')
            # Corta as linhas e colunas que não cabem no terminal: escrever fora
            # da janela faz o curses lançar erro.
            height, width = screen.getmaxyx()
            for row, line in enumerate(lines[:max(0, height - 1)]):
                screen.addnstr(row, 0, line, max(0, width - 1))
            screen.refresh()


def main(args=None):
    # Todo o trabalho acontece no construtor de car.
    car()


if __name__ == '__main__':
    main()
