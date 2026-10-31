r"""Microfone -> texto: reconhecimento de fala offline com o Vosk.

Este nó é só o ouvido do robô. Ele captura o microfone, passa o áudio pelo
reconhecedor e publica o que entendeu como texto. Quem decide o que o texto
significa é o command_parser, e quem obedeceria é o navegador — nenhum dos
dois mora aqui.

    microfone  --->  | voice_listener |  --->  voice/transcript  (String)
                                       --->  voice/partial     (String)
                                       --->  voice/level       (Float32)

Por que Vosk, e offline
-----------------------
O reconhecimento roda na própria máquina: não depende de internet, não manda
áudio para fora e tem latência previsível, que é o que importa num robô. O
modelo pequeno de português (~40 MB) carrega em poucos segundos e cabe na CPU
junto com o simulador.

Ouvir tudo, obedecer ao que sabe fazer
--------------------------------------
Por padrão o reconhecedor trabalha com o vocabulário inteiro do português:
toda fala que chega ao microfone é transcrita como foi dita, comando ou
conversa. A seleção acontece depois, no command_parser, que publica comando só
para as intenções de commands.py e descarta o resto. As duas coisas ficam
separadas de propósito: este nó é um ouvido, não um filtro, e
``voice/transcript`` é o registro fiel do que foi falado — útil para depurar
("ele ouviu errado" é diferente de "ele não entendeu") e para qualquer coisa
que venha a querer a fala completa.

A alternativa é a **gramática fechada** (``grammar:=true``): o Vosk aceita
receber, em vez do vocabulário do idioma, a lista de palavras que podem
aparecer (``commands.vocabulary()``). O reconhecedor passa a escolher sempre
dentro dela, e "limpa a cozinha" não vira "limpa a buzina"; o preço é que tudo
fora da lista sai como ``[unk]`` ou como a palavra mais parecida, e o robô
deixa de registrar o que de fato foi dito. Vale quando o ambiente é barulhento
e só os comandos importam, ou numa CPU apertada: medido com o modelo pequeno de
português, a gramática decodifica a ~0,07 do tempo real contra ~0,17 do
vocabulário completo.

O modelo
--------
Por padrão o nó pede ``Model(lang='pt')``, e o Vosk baixa o modelo pequeno de
português na primeira execução, para ``~/.cache/vosk``. Com o parâmetro
``model_path`` apontando para uma pasta de modelo já baixada, nada é buscado
na rede:

    ros2 run voice_cleaner voice_listener --ros-args \\
      -p model_path:=$HOME/vosk-model-small-pt-0.3

As duas threads
---------------
O PortAudio entrega o áudio numa thread dele, que não pode bloquear nem
chamar o ROS. Ela só empilha os blocos numa fila; um timer do nó esvazia a
fila e alimenta o reconhecedor. Assim o reconhecimento acontece na thread do
ROS, junto com as publicações, sem lock nenhum escrito à mão.
"""

import json
import math
import queue

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, String

from voice_cleaner import commands


class VoiceListener(Node):
    """Captura o microfone e publica o texto reconhecido."""

    def __init__(self):
        super().__init__('voice_listener')

        # --- Modelo e reconhecimento ----------------------------------------
        # Vazio: pede o modelo do idioma ao Vosk, que baixa se precisar.
        self.declare_parameter('model_path', '')
        self.declare_parameter('lang', 'pt')
        # O Vosk trabalha a 16 kHz; o servidor de som reamostra se o microfone
        # só tiver 44,1 ou 48 kHz.
        self.declare_parameter('sample_rate', 16000)
        # Falso por padrão: o robô transcreve tudo o que for dito, e é o
        # command_parser que decide o que vale como comando (ver o cabeçalho).
        self.declare_parameter('grammar', False)
        # Transcrição com confiança média abaixo disto é descartada. 0.0 aceita
        # tudo, que é o padrão: com o vocabulário aberto, descartar por
        # confiança jogaria fora justamente a fala livre que se quer registrar.
        self.declare_parameter('min_confidence', 0.0)
        # Publicar os resultados parciais (o que o Vosk tem no meio da frase).
        # Úteis para ver que o microfone está vivo; não viram comando.
        self.declare_parameter('publish_partial', True)

        # --- Captura ---------------------------------------------------------
        # Vazio: dispositivo padrão do sistema. Pode ser o índice ("3") ou um
        # pedaço do nome ("ALC236"); 'ros2 run voice_cleaner voice_listener
        # --ros-args -p list_devices:=true' mostra a lista e sai.
        self.declare_parameter('list_devices', False)
        self.declare_parameter('device', '')
        # 0,25 s de áudio por bloco: curto o bastante para a resposta não
        # atrasar, longo o bastante para não acordar a thread sem parar.
        self.declare_parameter('block_size', 4000)
        self.declare_parameter('level_rate', 2.0)        # publicação do nível [Hz]
        # Sem áudio nenhum por este tempo, avisa: é quase sempre o microfone
        # mudo ou o dispositivo errado, e sem o aviso o nó fica calado.
        self.declare_parameter('silent_warn', 10.0)      # [s]

        self.audio = queue.Queue()
        self.stream = None
        self.recognizer = None
        self.last_partial = ''
        self.level = 0.0
        self.blocks = 0
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.warned_silent = False

        self.transcript_pub = self.create_publisher(String, 'voice/transcript', 10)
        self.partial_pub = self.create_publisher(String, 'voice/partial', 10)
        self.level_pub = self.create_publisher(Float32, 'voice/level', 10)

        if self.get_parameter('list_devices').value:
            self.print_devices()
            raise SystemExit(0)

        self.open_recognizer()
        self.open_microphone()

        # Esvazia a fila de áudio bem mais rápido do que ela enche, para o
        # reconhecedor nunca acumular atraso.
        self.create_timer(0.05, self.process_audio)
        self.create_timer(1.0 / self.get_parameter('level_rate').value,
                          self.publish_level)

    # =========================================================================
    #  Preparação
    # =========================================================================

    def print_devices(self):
        """Lista as entradas de áudio que o PortAudio vê."""
        import sounddevice as sd
        self.get_logger().info('entradas de áudio disponíveis:')
        for i, dev in enumerate(sd.query_devices()):
            if dev['max_input_channels'] > 0:
                self.get_logger().info(
                    f"  [{i}] {dev['name']} "
                    f"({dev['max_input_channels']} canais, "
                    f"{dev['default_samplerate']:.0f} Hz)")

    def open_recognizer(self):
        """Carrega o modelo e monta o reconhecedor, com gramática se pedido."""
        try:
            from vosk import KaldiRecognizer, Model, SetLogLevel
        except ImportError:
            self.get_logger().error(
                'o pacote vosk não está instalado. Instale no Python do '
                'sistema (o ros2 run não usa o .venv):\n'
                '  /usr/bin/python3 -m pip install --user '
                '--break-system-packages vosk sounddevice')
            raise SystemExit(1)

        SetLogLevel(-1)   # o Kaldi é falante demais no terminal
        path = self.get_parameter('model_path').value
        lang = self.get_parameter('lang').value
        try:
            if path:
                self.get_logger().info(f'carregando o modelo de {path}')
                model = Model(model_path=path)
            else:
                self.get_logger().info(
                    f'carregando o modelo do idioma "{lang}" '
                    '(baixado em ~/.cache/vosk na primeira vez)')
                model = Model(lang=lang)
        except Exception as erro:
            self.get_logger().error(
                f'não consegui carregar o modelo: {erro}\n'
                'Baixe um modelo de https://alphacephei.com/vosk/models '
                '(vosk-model-small-pt-0.3 serve), descompacte e passe a pasta '
                'em model_path.')
            raise SystemExit(1)

        rate = float(self.get_parameter('sample_rate').value)
        if self.get_parameter('grammar').value:
            vocabulario = commands.vocabulary()
            # O [unk] dá ao reconhecedor onde jogar o que não é comando; sem
            # ele, qualquer conversa é forçada para dentro do vocabulário.
            grammar = json.dumps(vocabulario + ['[unk]'], ensure_ascii=False)
            try:
                self.recognizer = KaldiRecognizer(model, rate, grammar)
                self.get_logger().info(
                    f'gramática fechada com {len(vocabulario)} palavras')
            except Exception as erro:
                # Modelo grande (baseado em grafo dinâmico) não aceita
                # gramática; aí vale o vocabulário inteiro do idioma.
                self.get_logger().warn(
                    f'este modelo não aceita gramática ({erro}); '
                    'reconhecendo com o vocabulário completo')
                self.recognizer = KaldiRecognizer(model, rate)
        else:
            self.recognizer = KaldiRecognizer(model, rate)
            self.get_logger().info(
                'vocabulário completo do português: transcrevo tudo o que for '
                'dito, e o command_parser separa o que é comando '
                '(grammar:=true restringe o reconhecedor aos comandos)')
        self.recognizer.SetWords(True)   # confiança por palavra

    def open_microphone(self):
        """Abre a captura. O callback só empilha; nada de ROS nele."""
        try:
            import sounddevice as sd
        except ImportError:
            self.get_logger().error(
                'o pacote sounddevice não está instalado:\n'
                '  /usr/bin/python3 -m pip install --user '
                '--break-system-packages sounddevice')
            raise SystemExit(1)

        device = self.get_parameter('device').value or None
        if device is not None and device.lstrip('-').isdigit():
            device = int(device)

        def callback(indata, frames, time_info, status):
            if status:
                # Overflow acontece se a thread do ROS demorar a esvaziar a
                # fila. Vira aviso, não erro: o áudio perdido é o antigo.
                self.get_logger().warn(f'captura: {status}',
                                       throttle_duration_sec=5.0)
            self.audio.put(bytes(indata))

        try:
            self.stream = sd.RawInputStream(
                samplerate=self.get_parameter('sample_rate').value,
                blocksize=self.get_parameter('block_size').value,
                device=device,
                dtype='int16',
                channels=1,
                callback=callback,
            )
            self.stream.start()
        except Exception as erro:
            self.get_logger().error(
                f'não consegui abrir o microfone: {erro}\n'
                'Veja as entradas com -p list_devices:=true e escolha uma em '
                '-p device:=<índice ou nome>. Confira também se a entrada não '
                'está muda no controle de volume do sistema.')
            raise SystemExit(1)

        nome = device if device is not None else 'dispositivo padrão'
        self.get_logger().info(f'ouvindo o microfone ({nome}); fale um comando')

    # =========================================================================
    #  Reconhecimento
    # =========================================================================

    def process_audio(self):
        """Passa o áudio acumulado pelo reconhecedor e publica o que sair."""
        while True:
            try:
                data = self.audio.get_nowait()
            except queue.Empty:
                break
            self.blocks += 1
            self.measure(data)
            if self.recognizer.AcceptWaveform(data):
                self.publish_result(self.recognizer.Result())
            elif self.get_parameter('publish_partial').value:
                self.publish_partial(self.recognizer.PartialResult())
        self.check_silence()

    def measure(self, data):
        """Nível do bloco, em dBFS, para saber se o microfone está captando."""
        amostras = np.frombuffer(data, dtype=np.int16).astype(np.float32)
        if amostras.size == 0:
            return
        rms = float(np.sqrt(np.mean(amostras * amostras)))
        self.level = 20.0 * math.log10(max(rms, 1.0) / 32768.0)

    def publish_result(self, payload):
        """Resultado final de uma frase: publica se a confiança bastar."""
        resultado = json.loads(payload)
        texto = resultado.get('text', '').strip()
        if not texto:
            return
        confianca = self.average_confidence(resultado)
        minima = self.get_parameter('min_confidence').value
        if confianca < minima:
            self.get_logger().info(
                f'descartado por confiança ({confianca:.2f} < {minima:.2f}): '
                f'"{texto}"')
            return
        self.get_logger().info(f'ouvi: "{texto}" (confiança {confianca:.2f})')
        self.transcript_pub.publish(String(data=texto))

    @staticmethod
    def average_confidence(resultado):
        """Média das confianças por palavra; 1.0 quando o Vosk não as dá."""
        palavras = [p for p in resultado.get('result', []) if 'conf' in p]
        if not palavras:
            return 1.0
        return sum(p['conf'] for p in palavras) / len(palavras)

    def publish_partial(self, payload):
        """Hipótese do meio da frase, só quando muda."""
        parcial = json.loads(payload).get('partial', '').strip()
        if parcial and parcial != self.last_partial:
            self.last_partial = parcial
            self.partial_pub.publish(String(data=parcial))

    # =========================================================================
    #  Saúde da captura
    # =========================================================================

    def publish_level(self):
        self.level_pub.publish(Float32(data=float(self.level)))

    def check_silence(self):
        """Avisa uma vez se nenhum áudio chegou desde que o nó subiu."""
        if self.blocks or self.warned_silent:
            return
        agora = self.get_clock().now().nanoseconds / 1e9
        if agora - self.start_time > self.get_parameter('silent_warn').value:
            self.warned_silent = True
            self.get_logger().warn(
                'nenhum bloco de áudio chegou ainda. O dispositivo de entrada '
                'está certo (-p list_devices:=true) e com volume?')

    def close(self):
        if self.stream is not None:
            self.stream.stop()
            self.stream.close()
            self.stream = None


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VoiceListener()
    except SystemExit as saida:
        rclpy.try_shutdown()
        return int(saida.code or 0)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
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
