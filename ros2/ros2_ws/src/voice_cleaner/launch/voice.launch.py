"""Só a camada de voz: ouvir, entender e publicar o comando.

Não abre o simulador nem move o robô. Serve para conferir o reconhecimento e
o vocabulário, e é incluído pelos outros launches do pacote.

    ros2 launch voice_cleaner voice.launch.py
    ros2 launch voice_cleaner voice.launch.py console:=1          # digitar, sem microfone
    ros2 launch voice_cleaner voice.launch.py wake_word:=robo     # só obedece depois da palavra
    ros2 launch voice_cleaner voice.launch.py model_path:=$HOME/vosk-model-small-pt-0.3
    ros2 launch voice_cleaner voice.launch.py device:=3           # outra entrada de áudio

Com console:=1 o microfone sai da cadeia e entra o teclado, publicando no
mesmo tópico: o interpretador não vê diferença.

    microfone (ou teclado) --> voice/transcript --> command_parser --> voice/command
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='myRobot',
        description='Namespace dos tópicos; o mesmo dos outros nós do robô')
    console_arg = DeclareLaunchArgument(
        'console', default_value='0',
        description='1 troca o microfone pelo teclado (não precisa de modelo)')
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='',
        description='Pasta do modelo Vosk; vazio baixa o modelo do idioma')
    lang_arg = DeclareLaunchArgument(
        'lang', default_value='pt',
        description='Idioma do modelo baixado quando model_path está vazio')
    device_arg = DeclareLaunchArgument(
        'device', default_value='',
        description='Entrada de áudio (índice ou parte do nome); vazio usa a padrão')
    grammar_arg = DeclareLaunchArgument(
        'grammar', default_value='true',
        description='true restringe o reconhecedor ao vocabulário dos comandos')
    min_confidence_arg = DeclareLaunchArgument(
        'min_confidence', default_value='0.0',
        description='Descarta transcrição com confiança média abaixo disto')
    wake_word_arg = DeclareLaunchArgument(
        'wake_word', default_value='',
        description='Palavra de ativação; vazio aceita qualquer frase')
    min_score_arg = DeclareLaunchArgument(
        'min_score', default_value='1',
        description='Palavras que a frase precisa casar para virar comando')

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        namespace_arg,
        console_arg,
        model_path_arg,
        lang_arg,
        device_arg,
        grammar_arg,
        min_confidence_arg,
        wake_word_arg,
        min_score_arg,

        # O ouvido: microfone -> texto.
        Node(
            package='voice_cleaner',
            namespace=namespace,
            executable='voice_listener',
            name='voice_listener',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('console')),
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'lang': LaunchConfiguration('lang'),
                'device': LaunchConfiguration('device'),
                'grammar': ParameterValue(
                    LaunchConfiguration('grammar'), value_type=bool),
                'min_confidence': ParameterValue(
                    LaunchConfiguration('min_confidence'), value_type=float),
            }]
        ),

        # O substituto do ouvido, quando não se quer usar o microfone. Precisa
        # do terminal, por isso roda sem buffer e com a saída na tela.
        Node(
            package='voice_cleaner',
            namespace=namespace,
            executable='command_console',
            name='command_console',
            output='screen',
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration('console')),
        ),

        # O entendimento: texto -> comando.
        Node(
            package='voice_cleaner',
            namespace=namespace,
            executable='command_parser',
            name='command_parser',
            output='screen',
            parameters=[{
                'wake_word': LaunchConfiguration('wake_word'),
                'min_score': ParameterValue(
                    LaunchConfiguration('min_score'), value_type=int),
            }]
        ),
    ])
