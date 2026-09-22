"""Sobe a ponte de docking com o CoppeliaSim.

Uso:
ros2 launch robot_docking robot_docking.launch.py
ros2 launch robot_docking robot_docking.launch.py robot:=/meuRobo port:=23000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='/myRobot',
        description='Caminho do robô na cena do CoppeliaSim'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='23000',
        description='Porta da ZeroMQ Remote API do CoppeliaSim'
    )

    return LaunchDescription([
        robot_arg,
        port_arg,
        Node(
            package='robot_docking',
            # O namespace faz os tópicos relativos da ponte virarem
            # /myRobot/cmd_vel, /myRobot/charging_base/strengthSignal etc.
            namespace='myRobot',
            executable='coppelia_bridge',
            name='remoteAPI_ROS2_bridge',
            output='screen',
            parameters=[{
                'robot': LaunchConfiguration('robot'),
                # Argumentos de launch são texto; ParameterValue converte a
                # porta para inteiro antes de chegar ao nó.
                'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            }]
        ),
    ])
