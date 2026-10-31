"""Teclado no lugar do microfone: digite o comando em vez de falar.

O que este nó faz é publicar no mesmo tópico que o voice_listener publicaria,
com o mesmo tipo de mensagem. Para o command_parser não há diferença entre
uma frase falada e uma digitada, então toda a interpretação pode ser testada
sem microfone, sem modelo baixado e sem depender do reconhecimento.

    teclado  --->  | command_console |  --->  voice/transcript  (String)

É o mesmo papel que o fake_world tem para o navegador: trocar a fonte real
por uma de mentira para testar o resto da cadeia.

    ros2 run voice_cleaner command_console --ros-args -r __ns:=/myRobot

Digite uma frase e dê enter. 'ajuda' lista os comandos, 'sair' encerra.

A leitura do teclado acontece numa thread separada, porque input() bloqueia e
travaria o spin do ROS; ela só empilha as linhas numa fila que um timer do nó
esvazia.
"""

import queue
import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from voice_cleaner import commands


class CommandConsole(Node):
    """Lê frases do terminal e publica como se tivessem sido faladas."""

    def __init__(self):
        super().__init__('command_console')

        self.declare_parameter('prompt', 'comando> ')
        # Mostrar o que o interpretador faria com a frase, aqui mesmo. Ajuda a
        # ajustar o vocabulário sem ficar olhando dois terminais.
        self.declare_parameter('preview', True)

        self.lines = queue.Queue()
        self.transcript_pub = self.create_publisher(String, 'voice/transcript', 10)

        print('Comandos que o robô entende:')
        for linha in commands.help_lines():
            print(linha)
        print('\nDigite uma frase como você falaria. "sair" encerra.\n')

        self.reader = threading.Thread(target=self.read_lines, daemon=True)
        self.reader.start()
        self.create_timer(0.1, self.drain)

    def read_lines(self):
        """Thread do teclado: só empilha o que o usuário digitar."""
        prompt = self.get_parameter('prompt').value
        while True:
            try:
                linha = input(prompt)
            except (EOFError, KeyboardInterrupt):
                self.lines.put(None)
                return
            self.lines.put(linha)

    def drain(self):
        """Publica as linhas digitadas; None quer dizer fim da entrada."""
        while True:
            try:
                linha = self.lines.get_nowait()
            except queue.Empty:
                return
            if linha is None:
                raise KeyboardInterrupt
            linha = linha.strip()
            if not linha:
                continue
            if commands.normalize(linha) in ('sair', 'quit', 'exit'):
                raise KeyboardInterrupt
            self.transcript_pub.publish(String(data=linha))
            if self.get_parameter('preview').value:
                self.preview(linha)

    def preview(self, linha):
        """Diz, no próprio console, que intenção a frase casou."""
        intent, args, score = commands.parse(linha)
        if intent is None:
            print('  -> nenhuma intenção casou (o parser vai ignorar)')
            return
        detalhe = ', '.join(f'{k}={v}' for k, v in args.items())
        print(f'  -> {intent.name}'
              f'{" (" + detalhe + ")" if detalhe else ""}, {score} palavra(s)')


def main(args=None):
    rclpy.init(args=args)
    node = CommandConsole()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print('', file=sys.stderr)
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
