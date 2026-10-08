"""Sobe a ponte, o SLAM Toolbox, o navegador e o rviz.

    ros2 launch slam_field slam_field.launch.py                    # gabarito como odometria
    ros2 launch slam_field slam_field.launch.py odom:=wheel        # odometria das rodas
    ros2 launch slam_field slam_field.launch.py navigator:=0       # só mapear (teleop)
    ros2 launch slam_field slam_field.launch.py banana_ids:=4      # uma banana só

Com navigator:=0 o cmd_vel pode vir de um teleop:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('slam_field')
    slam_share = get_package_share_directory('slam_toolbox')

    args = [
        DeclareLaunchArgument('odom', default_value='ground_truth',
                              description='Fonte da odometria: ground_truth ou wheel'),
        DeclareLaunchArgument('robot', default_value='/myRobot',
                              description='Caminho do robô na cena'),
        DeclareLaunchArgument('port', default_value='23000',
                              description='Porta da ZeroMQ Remote API'),
        DeclareLaunchArgument('navigator', default_value='1',
                              description='1 sobe o navegador; 0 só ponte + SLAM'),
        DeclareLaunchArgument('rviz', default_value='1',
                              description='1 abre o rviz com mapa, scan e forças'),
        DeclareLaunchArgument('banana_ids', default_value='',
                              description='Bananas a buscar, ex. "4" ou "4,9"; vazio = todas'),
        DeclareLaunchArgument('v_max', default_value='0.15',
                              description='Velocidade máxima de avanço [m/s]'),
        DeclareLaunchArgument('unknown_is_obstacle', default_value='false',
                              description='true trata célula desconhecida como obstáculo'),
        DeclareLaunchArgument('stop_on_done', default_value='true',
                              description='Parar a simulação quando acabarem as bananas'),
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(share, 'config', 'slam_toolbox.yaml'),
                              description='Parâmetros do SLAM Toolbox'),
    ]

    bridge = Node(
        package='slam_field',
        executable='coppelia_bridge',
        name='coppelia_bridge',
        output='screen',
        parameters=[{
            'robot': LaunchConfiguration('robot'),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'odom_source': LaunchConfiguration('odom'),
            'stop_on_done': ParameterValue(LaunchConfiguration('stop_on_done'),
                                           value_type=bool),
        }],
    )

    # O launch do próprio SLAM Toolbox: sobe o async_slam_toolbox_node, que é
    # um nó com ciclo de vida, e já o configura e ativa (autostart).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            # O relógio é o do computador: a ponte carimba com o tempo real.
            'use_sim_time': 'false',
        }.items(),
    )

    navigator = Node(
        package='slam_field',
        executable='navigator',
        name='navigator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('navigator')),
        parameters=[{
            'banana_ids': ParameterValue(LaunchConfiguration('banana_ids'), value_type=str),
            'v_max': ParameterValue(LaunchConfiguration('v_max'), value_type=float),
            'unknown_is_obstacle': ParameterValue(
                LaunchConfiguration('unknown_is_obstacle'), value_type=bool),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share, 'config', 'slam_field.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [bridge, slam, navigator, rviz])
