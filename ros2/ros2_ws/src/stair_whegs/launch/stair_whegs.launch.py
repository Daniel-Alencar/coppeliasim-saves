"""Sobe a ponte, o gerador de marcha e o escalador.

    ros2 launch stair_whegs stair_whegs.launch.py
    ros2 launch stair_whegs stair_whegs.launch.py climb_speed:=2.0
    ros2 launch stair_whegs stair_whegs.launch.py climb_gait:=alternada
    ros2 launch stair_whegs stair_whegs.launch.py window:=0.4

Antes de rodar, a cena tem de existir:

    python3 scene/build_scene.py

Cadeia que sobe:

    coppelia_bridge -- probes, attitude, pose --> climber
    climber         -- drive, gait            --> gait
    climber         -- wheel_mode             --> coppelia_bridge
    gait            -- wheg_joints            --> coppelia_bridge

A marcha gira as rodas; o `wheel_mode` decide se elas são RODA (aro fechado,
para andar no plano) ou GARRA (pernas abertas em catavento, para subir). Quem
escolhe é o `climber`, a partir das sondas — ver o cabeçalho dele.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('rover', default_value='/rover',
                              description='Caminho do rover na cena'),
        DeclareLaunchArgument('port', default_value='23000',
                              description='Porta da ZeroMQ Remote API'),
        DeclareLaunchArgument('search_gait', default_value='alternada',
                              description='Marcha no plano: alternada, '
                                          'dianteira ou junta'),
        DeclareLaunchArgument('climb_gait', default_value='junta',
                              description='Marcha na escada. junta porque a '
                                          'alternada balança o corpo em '
                                          'rolagem e capota com a garra '
                                          'aberta'),
        DeclareLaunchArgument('search_speed', default_value='3.0',
                              description='Velocidade dos whegs no plano [rad/s]'),
        DeclareLaunchArgument('climb_speed', default_value='3.0',
                              description='Velocidade dos whegs na escada [rad/s]'),
        DeclareLaunchArgument('window', default_value='0.7',
                              description='Janela de decisão [s de simulação]; '
                                          'cerca de um passo da marcha'),
        DeclareLaunchArgument('step_ratio', default_value='0.70',
                              description='Fração da leitura de piso plano '
                                          'abaixo da qual há degrau'),
        DeclareLaunchArgument('transform_time', default_value='0.8',
                              description='Tempo parado enquanto as pernas '
                                          'abrem ou fecham [s de simulação]'),
        DeclareLaunchArgument('stop_when_done', default_value='true'),
    ]

    namespace = 'whegs'

    bridge = Node(
        package='stair_whegs', namespace=namespace,
        executable='coppelia_bridge', name='coppelia_bridge', output='screen',
        parameters=[{
            'rover': LaunchConfiguration('rover'),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'stop_on_done': ParameterValue(LaunchConfiguration('stop_when_done'),
                                           value_type=bool),
        }],
        on_exit=Shutdown(reason='ponte encerrada'),
    )

    gait = Node(
        package='stair_whegs', namespace=namespace,
        executable='gait', name='gait', output='screen',
        parameters=[{
            'gait': LaunchConfiguration('search_gait'),
        }],
    )

    climber = Node(
        package='stair_whegs', namespace=namespace,
        executable='climber', name='climber', output='screen',
        parameters=[{
            'search_speed': ParameterValue(LaunchConfiguration('search_speed'),
                                           value_type=float),
            'climb_speed': ParameterValue(LaunchConfiguration('climb_speed'),
                                          value_type=float),
            'window': ParameterValue(LaunchConfiguration('window'),
                                     value_type=float),
            'step_ratio': ParameterValue(LaunchConfiguration('step_ratio'),
                                         value_type=float),
            'transform_time': ParameterValue(
                LaunchConfiguration('transform_time'), value_type=float),
            'search_gait': LaunchConfiguration('search_gait'),
            'climb_gait': LaunchConfiguration('climb_gait'),
            'stop_when_done': ParameterValue(LaunchConfiguration('stop_when_done'),
                                             value_type=bool),
        }],
    )

    return LaunchDescription(args + [bridge, gait, climber])
