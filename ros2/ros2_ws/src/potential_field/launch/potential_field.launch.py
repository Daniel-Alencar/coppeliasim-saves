"""Sobe a ponte com o CoppeliaSim e o navegador por campos potenciais.

    ros2 launch potential_field potential_field.launch.py
    ros2 launch potential_field potential_field.launch.py navigator:=0
    ros2 launch potential_field potential_field.launch.py robot:=/meuRobo port:=23010

O argumento navigator escolhe se o algoritmo sobe junto. Com navigator:=0 fica
só a ponte, e o cmd_vel pode vir de um teleop:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -r cmd_vel:=/myRobot/cmd_vel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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
    navigator_arg = DeclareLaunchArgument(
        'navigator',
        default_value='1',
        description='1 para subir também o navegador por campos potenciais'
    )
    fake_arg = DeclareLaunchArgument(
        'fake',
        default_value='0',
        description='1 troca a ponte pelo mundo de mentira (não usa o CoppeliaSim)'
    )

    return LaunchDescription([
        robot_arg,
        port_arg,
        navigator_arg,
        fake_arg,

        # A ponte com o simulador. O namespace faz os tópicos relativos virarem
        # /myRobot/pose, /myRobot/bananas, /myRobot/cmd_vel etc.
        Node(
            package='potential_field',
            namespace='myRobot',
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
            package='potential_field',
            namespace='myRobot',
            executable='fake_world',
            name='fake_world',
            output='screen',
            condition=IfCondition(LaunchConfiguration('fake'))
        ),

        # O algoritmo. Mesmo namespace, então ele fala com a ponte sem remapear.
        Node(
            package='potential_field',
            namespace='myRobot',
            executable='navigator',
            name='navigator',
            output='screen',
            condition=IfCondition(LaunchConfiguration('navigator'))
        ),
    ])
