"""O robô inteiro: ponte com o CoppeliaSim, navegador e camada de voz.

    ros2 launch voice_cleaner voice_cleaner.launch.py
    ros2 launch voice_cleaner voice_cleaner.launch.py console:=1    # digitar em vez de falar
    ros2 launch voice_cleaner voice_cleaner.launch.py navigator:=0  # só a voz, robô parado
    ros2 launch voice_cleaner voice_cleaner.launch.py fake:=1       # sem o simulador

A voz e a navegação sobem lado a lado e não se falam ainda: o comando aparece
em voice/command e o navegador continua fazendo o que já fazia. Ligar um ao
outro é a etapa seguinte do projeto.

    microfone --> voice_listener --> command_parser --> voice/command   (nova camada)
    cena      --> coppelia_bridge --> navigator --> cmd_vel             (base herdada)

Para ver o comando chegando, noutro terminal:

    ros2 topic echo /myRobot/voice/command
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NAMESPACE = 'myRobot'


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot', default_value='/myRobot',
        description='Caminho do robô na cena do CoppeliaSim')
    port_arg = DeclareLaunchArgument(
        'port', default_value='23000',
        description='Porta da ZeroMQ Remote API do CoppeliaSim')
    navigator_arg = DeclareLaunchArgument(
        'navigator', default_value='1',
        description='1 sobe o navegador por campos potenciais; 0 deixa o robô parado')
    fake_arg = DeclareLaunchArgument(
        'fake', default_value='0',
        description='1 troca a ponte pelo mundo de mentira (não usa o CoppeliaSim)')
    voice_arg = DeclareLaunchArgument(
        'voice', default_value='1',
        description='1 sobe a camada de voz')
    console_arg = DeclareLaunchArgument(
        'console', default_value='0',
        description='1 digita os comandos em vez de falar')
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='',
        description='Pasta do modelo Vosk; vazio baixa o modelo do idioma')
    device_arg = DeclareLaunchArgument(
        'device', default_value='',
        description='Entrada de áudio (índice ou parte do nome)')
    wake_word_arg = DeclareLaunchArgument(
        'wake_word', default_value='',
        description='Palavra de ativação; vazio aceita qualquer frase')

    voice_launch = os.path.join(
        get_package_share_directory('voice_cleaner'), 'launch', 'voice.launch.py')

    return LaunchDescription([
        robot_arg,
        port_arg,
        navigator_arg,
        fake_arg,
        voice_arg,
        console_arg,
        model_path_arg,
        device_arg,
        wake_word_arg,

        # A ponte com o simulador. O namespace faz os tópicos relativos virarem
        # /myRobot/pose, /myRobot/cmd_vel etc.
        Node(
            package='voice_cleaner',
            namespace=NAMESPACE,
            executable='coppelia_bridge',
            name='coppelia_bridge',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('fake')),
            parameters=[{
                'robot': LaunchConfiguration('robot'),
                # Argumentos de launch são texto; ParameterValue converte a
                # porta para inteiro antes de chegar ao nó.
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            }]
        ),

        # O substituto da ponte, quando não se quer abrir o simulador.
        Node(
            package='voice_cleaner',
            namespace=NAMESPACE,
            executable='fake_world',
            name='fake_world',
            output='screen',
            condition=IfCondition(LaunchConfiguration('fake')),
        ),

        Node(
            package='voice_cleaner',
            namespace=NAMESPACE,
            executable='navigator',
            name='navigator',
            output='screen',
            condition=IfCondition(LaunchConfiguration('navigator')),
        ),

        # A camada de voz, no mesmo namespace dos outros nós.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(voice_launch),
            condition=IfCondition(LaunchConfiguration('voice')),
            launch_arguments={
                'namespace': NAMESPACE,
                'console': LaunchConfiguration('console'),
                'model_path': LaunchConfiguration('model_path'),
                'device': LaunchConfiguration('device'),
                'wake_word': LaunchConfiguration('wake_word'),
            }.items(),
        ),
    ])
