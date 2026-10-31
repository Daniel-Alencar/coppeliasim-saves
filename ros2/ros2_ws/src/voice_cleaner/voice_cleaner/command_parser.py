"""Texto -> comando: decide qual intenção foi dita e publica o comando.

Este nó fecha a camada de voz. Ele não sabe de microfone nem de simulador: só
recebe texto, procura a intenção no vocabulário de commands.py e publica o
comando estruturado. Quem fala com ele pode ser o voice_listener (microfone)
ou o command_console (teclado) — os dois publicam no mesmo tópico.

    voice/transcript (String)  --->  | command_parser |  ---> voice/command (String, JSON)
                                                        ---> voice/intent  (String)

Ninguém consome voice/command ainda: nesta etapa o robô **recebe** os comandos,
e executá-los é o passo seguinte. Deixar o comando num tópico, e não numa
chamada direta ao navegador, é o que permite plugar a execução depois sem
mexer em nada daqui.

Este nó é o filtro da camada de voz
-----------------------------------
O voice_listener transcreve tudo o que for dito, conversa inclusive. É aqui que
a fala é separada: ``voice/command`` só recebe as intenções de commands.py, e o
que não é tarefa para o robô fica no log ("ouvido, mas não é tarefa minha") e
não vai para tópico nenhum. Então a maior parte da fala parar neste nó é o
funcionamento normal, não falha de reconhecimento — para ver o que o robô de
fato ouviu, ``voice/transcript``.

Com o vocabulário aberto, a conversa da sala passa a ser transcrita, e uma
frase qualquer pode casar com um comando por acidente ("pode parar de falar
nisso" tem "parar"). ``wake_word:=robo`` exige o nome do robô antes e é a
defesa que não custa comando nenhum. ``min_score:=2`` também filtra, exigindo
frase mais completa, mas aí ``pausar`` e ``continuar`` ficam impossíveis de
dizer: nenhuma forma delas tem duas palavras. Os dois vêm desligados.

O formato do comando
--------------------
Um JSON dentro de std_msgs/String:

    {"intent": "limpar_local", "args": {"local": "cozinha"},
     "texto": "robo limpa a cozinha", "score": 2, "stamp": 1730212345.67}

É JSON em String, e não uma mensagem própria, porque o pacote é ament_python:
uma mensagem .msg exigiria passar o pacote para ament_cmake. Para ver o
comando chegando, basta:

    ros2 topic echo /myRobot/voice/command

A palavra de ativação
---------------------
Com ``wake_word`` vazio (o padrão) qualquer frase reconhecida vale como
comando. Pondo ``wake_word:=robo``, o nó passa a ignorar tudo que não venha
depois da palavra: ou na mesma frase ("robô, limpa a sala"), ou numa frase
anterior, dentro de ``wake_timeout`` segundos. Serve para o robô não obedecer
à conversa da sala — com o custo de exigir a palavra, que o reconhecedor
também pode errar.
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from voice_cleaner import commands


class CommandParser(Node):
    """Transforma transcrição em comando estruturado."""

    def __init__(self):
        super().__init__('command_parser')

        # Vazio: sem palavra de ativação, toda frase reconhecida é comando.
        self.declare_parameter('wake_word', '')
        self.declare_parameter('wake_timeout', 8.0)      # [s]
        # Quantas palavras a frase precisa casar. 1 aceita comando de uma
        # palavra ("parar"); 2 exige frase mais completa e erra menos.
        self.declare_parameter('min_score', 1)
        # Repetir o mesmo comando dentro deste tempo é ignorado: o Vosk às
        # vezes fecha a mesma frase duas vezes, e um comando duplicado faria o
        # robô reiniciar a tarefa que acabou de começar.
        self.declare_parameter('repeat_window', 1.5)     # [s]
        # Mostrar a lista de comandos no terminal quando o nó sobe.
        self.declare_parameter('print_help', True)

        self.awake_until = None     # até quando a palavra de ativação vale
        self.last_command = None    # (intenção, instante) do último publicado

        self.command_pub = self.create_publisher(String, 'voice/command', 10)
        self.intent_pub = self.create_publisher(String, 'voice/intent', 10)
        self.create_subscription(String, 'voice/transcript',
                                 self.transcript_callback, 10)

        if self.get_parameter('print_help').value:
            self.print_help()
        wake = commands.normalize(self.get_parameter('wake_word').value)
        if wake:
            self.get_logger().info(
                f'palavra de ativação "{wake}": só obedeço depois dela')
        self.get_logger().info('interpretador pronto, esperando transcrições')

    def print_help(self):
        self.get_logger().info('comandos que o robô entende:')
        for linha in commands.help_lines():
            self.get_logger().info(linha)

    # =========================================================================
    #  Interpretação
    # =========================================================================

    def transcript_callback(self, msg: String):
        """Uma transcrição: filtra pela ativação, interpreta e publica."""
        texto = msg.data.strip()
        if not texto:
            return
        agora = self.get_clock().now().nanoseconds / 1e9
        if not self.is_awake(texto, agora):
            self.get_logger().info(
                f'ignorado (falta a palavra de ativação): "{texto}"')
            return

        intent, args, score = commands.parse(texto)
        if intent is None:
            # O caminho normal, não uma exceção: o voice_listener transcreve
            # toda a fala, e a maior parte dela não é tarefa para o robô.
            self.get_logger().info(f'ouvido, mas não é tarefa minha: "{texto}"')
            return
        minimo = self.get_parameter('min_score').value
        if score < minimo:
            self.get_logger().info(
                f'"{texto}" parece {intent.name}, mas casou só {score} '
                f'palavra(s) de {minimo}: ignorado')
            return
        if self.is_repeat(intent, args, agora):
            return

        self.publish(intent, args, texto, score, agora)

    def is_awake(self, texto, agora):
        """Diz se este texto pode valer como comando.

        Sem palavra de ativação, sempre. Com ela, vale se a palavra estiver
        nesta frase (e então a janela abre para as próximas) ou se uma frase
        recente já a tiver trazido.
        """
        wake = commands.normalize(self.get_parameter('wake_word').value)
        if not wake:
            return True
        if wake in commands.normalize(texto).split():
            self.awake_until = agora + self.get_parameter('wake_timeout').value
            return True
        return self.awake_until is not None and agora < self.awake_until

    def is_repeat(self, intent, args, agora):
        """Mesmo comando outra vez dentro da janela: descarta."""
        chave = (intent.name, tuple(sorted(args.items())))
        if self.last_command is not None:
            anterior, quando = self.last_command
            if (anterior == chave and
                    agora - quando < self.get_parameter('repeat_window').value):
                return True
        return False

    def publish(self, intent, args, texto, score, agora):
        """Publica o comando no formato JSON e o nome da intenção."""
        self.last_command = ((intent.name, tuple(sorted(args.items()))), agora)
        comando = {
            'intent': intent.name,
            'args': args,
            'texto': commands.normalize(texto),
            'score': score,
            'stamp': round(agora, 3),
        }
        self.command_pub.publish(
            String(data=json.dumps(comando, ensure_ascii=False)))
        self.intent_pub.publish(String(data=intent.name))

        detalhe = ''
        if args:
            detalhe = ' ' + ', '.join(f'{k}={v}' for k, v in args.items())
        self.get_logger().info(
            f'comando: {intent.name}{detalhe}  <- "{texto}"')


def main(args=None):
    rclpy.init(args=args)
    node = CommandParser()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # O ros2 launch repassa um segundo Ctrl+C durante a limpeza; sem isto
        # ele interrompe o destroy_node e imprime um traceback inofensivo.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
