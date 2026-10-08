"""Campo potencial com o mapa vindo da câmera, e não dos sinais da cena.

    ros2 launch potential_field perception_field.launch.py
    ros2 launch potential_field perception_field.launch.py navigator:=0   # só perceber
    ros2 launch potential_field perception_field.launch.py model:=yolo11n-seg.pt

Cadeia que sobe:

    coppelia_bridge  -- rgb/image, depth/image, pose, /tf -->  yolo_vision
    yolo_vision      -- detections/bananas, detections/poops -->  perception_map
    perception_map   -- bananas, poops -->  navigator
    navigator        -- cmd_vel -->  coppelia_bridge

A ponte entra com map_source=perception, ou seja, ela não lê mais as posições
dos sinais 'banana' e 'poop' da cena: o mapa é construído pelo que o robô vê.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    # Todos no mesmo namespace: os tópicos relativos se encontram sem remapear.
    namespace = 'myRobot'

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

        Node(
            package='potential_field',
            namespace=namespace,
            executable='coppelia_bridge',
            name='coppelia_bridge',
            output='screen',
            parameters=[{
                'robot': LaunchConfiguration('robot'),
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                # O ponto do exercício: nada de posições prontas da simulação.
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
            package='potential_field',
            namespace=namespace,
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
            package='potential_field',
            namespace=namespace,
            executable='perception_map',
            name='perception_map',
            output='screen',
        ),

        Node(
            package='potential_field',
            namespace=namespace,
            executable='navigator',
            name='navigator',
            output='screen',
            condition=IfCondition(LaunchConfiguration('navigator')),
        ),
    ])
