"""Sobe a ponte, o gerador de marcha e o escalador.

    ros2 launch stair_whegs stair_whegs.launch.py
    ros2 launch stair_whegs stair_whegs.launch.py climb_speed:=2.0
    ros2 launch stair_whegs stair_whegs.launch.py gait:=junta     # a que falha
    ros2 launch stair_whegs stair_whegs.launch.py window:=0.4

Antes de rodar, a cena tem de existir:

    python3 scene/build_scene.py

Cadeia que sobe:

    coppelia_bridge -- probes, attitude, pose --> climber
    climber         -- drive, gait            --> gait
    gait            -- wheg_joints            --> coppelia_bridge
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
        DeclareLaunchArgument('gait', default_value='alternada',
                              description='Marcha: alternada, dianteira ou '
                                          'junta (esta última não sobe)'),
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
            'gait': LaunchConfiguration('gait'),
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
            'search_gait': LaunchConfiguration('gait'),
            'climb_gait': LaunchConfiguration('gait'),
            'stop_when_done': ParameterValue(LaunchConfiguration('stop_when_done'),
                                             value_type=bool),
        }],
    )

    return LaunchDescription(args + [bridge, gait, climber])
