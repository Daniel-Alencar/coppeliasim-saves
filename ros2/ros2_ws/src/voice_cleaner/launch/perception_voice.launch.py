"""Tudo junto: percepção pela câmera, campo potencial e camada de voz.

É o launch herdado do potential_field (mapa construído pelo YOLO na imagem da
Kinect, em vez de lido dos sinais da cena) com a camada de voz somada.

    ros2 launch voice_cleaner perception_voice.launch.py
    ros2 launch voice_cleaner perception_voice.launch.py console:=1   # digitar os comandos
    ros2 launch voice_cleaner perception_voice.launch.py navigator:=0 # só perceber e ouvir

Cadeia que sobe:

    coppelia_bridge  -- rgb/image, depth/image, pose, /tf -->  yolo_vision
    yolo_vision      -- detections/bananas, detections/poops -->  perception_map
    perception_map   -- bananas, poops -->  navigator
    navigator        -- cmd_vel -->  coppelia_bridge

    microfone -->  voice_listener -- voice/transcript -->  command_parser
    command_parser  -- voice/command -->  (ninguém ainda)

A ponte entra com map_source=perception, ou seja, ela não lê as posições dos
sinais 'banana' e 'poop' da cena: o mapa é construído pelo que o robô vê.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Todos no mesmo namespace: os tópicos relativos se encontram sem remapear.
NAMESPACE = 'myRobot'


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot', default_value='/myRobot',
        description='Caminho do robô na cena do CoppeliaSim')
    port_arg = DeclareLaunchArgument(
        'port', default_value='23000',
        description='Porta da ZeroMQ Remote API')
    navigator_arg = DeclareLaunchArgument(
        'navigator', default_value='1',
        description='1 sobe o navegador; 0 só percebe, sem mover o robô')
    model_arg = DeclareLaunchArgument(
        'model', default_value='yolo11s.pt',
        description='Modelo do ultralytics; baixado na primeira execução se faltar')
    # Mesmos padrões da ponte: raio medido no modelo da cena (cilindro de 0,1 m
    # de diâmetro) e separação medida entre os dois motores ao iniciar.
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.05',
        description='Raio da roda (m), medido na cena')
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.0',
        description='Distância entre rodas (m); 0.0 mede na própria cena (0,2 m)')
    torch_threads_arg = DeclareLaunchArgument(
        'torch_threads', default_value='4',
        description='Threads do PyTorch; sem limite o YOLO tira CPU do simulador')
    stop_after_arg = DeclareLaunchArgument(
        'stop_after_bananas', default_value='20',
        description='Para a simulação e encerra tudo ao recolher este número (0 desliga)')
    unknown_as_poop_arg = DeclareLaunchArgument(
        'unknown_as_poop', default_value='true',
        description='Tratar toda detecção que não é banana como obstáculo')

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
        model_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        torch_threads_arg,
        stop_after_arg,
        unknown_as_poop_arg,
        voice_arg,
        console_arg,
        model_path_arg,
        device_arg,
        wake_word_arg,

        Node(
            package='voice_cleaner',
            namespace=NAMESPACE,
            executable='coppelia_bridge',
            name='coppelia_bridge',
            output='screen',
            parameters=[{
                'robot': LaunchConfiguration('robot'),
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                # Nada de posições prontas da simulação: o mapa vem da câmera.
                'map_source': 'perception',
                'wheel_radius': ParameterValue(
                    LaunchConfiguration('wheel_radius'), value_type=float),
                'wheel_separation': ParameterValue(
                    LaunchConfiguration('wheel_separation'), value_type=float),
                'stop_after_bananas': ParameterValue(
                    LaunchConfiguration('stop_after_bananas'), value_type=int),
            }],
            # Quando a ponte sai (meta atingida ou Ctrl+C), o launch inteiro
            # encerra junto, em vez de deixar os outros nós rodando sem simulador.
            on_exit=Shutdown(reason='ponte encerrada'),
        ),

        Node(
            package='voice_cleaner',
            namespace=NAMESPACE,
            executable='yolo_vision',
            name='yolo_vision',
            output='screen',
            parameters=[{
                'model': LaunchConfiguration('model'),
                'torch_threads': ParameterValue(
                    LaunchConfiguration('torch_threads'), value_type=int),
                'unknown_as_poop': ParameterValue(
                    LaunchConfiguration('unknown_as_poop'), value_type=bool),
            }]
        ),

        Node(
            package='voice_cleaner',
            namespace=NAMESPACE,
            executable='perception_map',
            name='perception_map',
            output='screen',
        ),

        Node(
            package='voice_cleaner',
            namespace=NAMESPACE,
            executable='navigator',
            name='navigator',
            output='screen',
            condition=IfCondition(LaunchConfiguration('navigator')),
        ),

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
