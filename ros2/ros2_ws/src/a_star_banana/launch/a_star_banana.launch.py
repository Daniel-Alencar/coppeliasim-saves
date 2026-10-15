"""Sobe a ponte, o controlador de posição, o planejador A* e o rviz.

    ros2 launch a_star_banana a_star_banana.launch.py
    ros2 launch a_star_banana a_star_banana.launch.py goal:='/bananas/Banana[4]'
    ros2 launch a_star_banana a_star_banana.launch.py resolution:=0.1 inflate:=0.3
    ros2 launch a_star_banana a_star_banana.launch.py goal:='2.3,-2.3'   # inalcançável
    ros2 launch a_star_banana a_star_banana.launch.py collect_all:=false  # só uma
    ros2 launch a_star_banana a_star_banana.launch.py rviz:=0

Cadeia que sobe:

    coppelia_bridge  -- poops, bananas, pose -->  planner
    planner          -- goal                 -->  position_control
    position_control -- cmd_vel              -->  coppelia_bridge
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('a_star_banana')

    args = [
        DeclareLaunchArgument('robot', default_value='/myRobot',
                              description='Caminho do robô na cena'),
        DeclareLaunchArgument('port', default_value='23000',
                              description='Porta da ZeroMQ Remote API'),
        DeclareLaunchArgument('goal', default_value='',
                              description="Alvo: vazio (banana mais barata), "
                                          "'/bananas/Banana[4]' ou 'x,y'"),
        DeclareLaunchArgument('resolution', default_value='0.2',
                              description='Lado da célula do mapa [m]'),
        DeclareLaunchArgument('inflate', default_value='0.09',
                              description='Margem de segurança além do raio '
                                          'do robô [m]'),
        DeclareLaunchArgument('robot_radius', default_value='0.16',
                              description='Raio do robô [m], medido na cena'),
        DeclareLaunchArgument('tolerance', default_value='0.15',
                              description='Raio para dar um waypoint como '
                                          'alcançado [m]'),
        DeclareLaunchArgument('diagonal', default_value='true',
                              description='true = vizinhança de 8; false = de 4'),
        DeclareLaunchArgument('collect_all', default_value='true',
                              description='true replaneja para a próxima banana '
                                          'até acabarem; false para na primeira'),
        DeclareLaunchArgument('v_const', default_value='0.22',
                              description='Velocidade de cruzeiro [m/s]'),
        DeclareLaunchArgument('stop_on_done', default_value='true',
                              description='Parar a simulação ao terminar'),
        DeclareLaunchArgument('rviz', default_value='1',
                              description='1 abre o rviz com mapa e caminho'),
    ]

    # Todos no mesmo namespace: os tópicos relativos se encontram sem remapear,
    # e o goal fica em /myRobot/goal, como o enunciado pede.
    namespace = 'myRobot'

    bridge = Node(
        package='a_star_banana',
        namespace=namespace,
        executable='coppelia_bridge',
        name='coppelia_bridge',
        output='screen',
        parameters=[{
            'robot': LaunchConfiguration('robot'),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'stop_on_done': ParameterValue(LaunchConfiguration('stop_on_done'),
                                           value_type=bool),
        }],
        # Quando a ponte sai (alvo alcançado ou Ctrl+C), o launch inteiro
        # encerra junto, em vez de deixar os outros nós sem simulador.
        on_exit=Shutdown(reason='ponte encerrada'),
    )

    position_control = Node(
        package='a_star_banana',
        namespace=namespace,
        executable='position_control',
        name='position_control',
        output='screen',
        parameters=[{
            'v_const': ParameterValue(LaunchConfiguration('v_const'),
                                      value_type=float),
        }],
    )

    planner = Node(
        package='a_star_banana',
        namespace=namespace,
        executable='planner',
        name='planner',
        output='screen',
        parameters=[{
            'goal': ParameterValue(LaunchConfiguration('goal'), value_type=str),
            'resolution': ParameterValue(LaunchConfiguration('resolution'),
                                         value_type=float),
            'safety_margin': ParameterValue(LaunchConfiguration('inflate'),
                                            value_type=float),
            'robot_radius': ParameterValue(LaunchConfiguration('robot_radius'),
                                           value_type=float),
            'waypoint_tolerance': ParameterValue(LaunchConfiguration('tolerance'),
                                                 value_type=float),
            'diagonal': ParameterValue(LaunchConfiguration('diagonal'),
                                       value_type=bool),
            'collect_all': ParameterValue(LaunchConfiguration('collect_all'),
                                          value_type=bool),
            'stop_on_done': ParameterValue(LaunchConfiguration('stop_on_done'),
                                           value_type=bool),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share, 'config', 'a_star_banana.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [bridge, position_control, planner, rviz])
