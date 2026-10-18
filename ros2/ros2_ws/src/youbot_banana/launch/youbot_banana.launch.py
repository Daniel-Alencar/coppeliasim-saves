"""Sobe a ponte, o controle da base, o controle do braço, a missão e o rviz.

    ros2 launch youbot_banana youbot_banana.launch.py
    ros2 launch youbot_banana youbot_banana.launch.py resolution:=0.15
    ros2 launch youbot_banana youbot_banana.launch.py robot_radius:=0.30
    ros2 launch youbot_banana youbot_banana.launch.py grasp_reach:=0.18
    ros2 launch youbot_banana youbot_banana.launch.py cartesian_speed:=0.04
    ros2 launch youbot_banana youbot_banana.launch.py rviz:=0

Cadeia que sobe:

    coppelia_bridge  -- bananas, poops, pose, arm_state -->  mission
    mission          -- goal                            -->  base_control
    mission          -- arm_goal, gripper               -->  arm_control
    base_control     -- cmd_vel                         -->  coppelia_bridge
    arm_control      -- arm_joints                      -->  coppelia_bridge

Antes de rodar, a cena tem de existir:

    python3 scene/build_scene.py
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
    share = get_package_share_directory('youbot_banana')

    args = [
        DeclareLaunchArgument('robot', default_value='/youBot',
                              description='Caminho do robô na cena'),
        DeclareLaunchArgument('port', default_value='23000',
                              description='Porta da ZeroMQ Remote API'),
        DeclareLaunchArgument('resolution', default_value='0.10',
                              description='Lado da célula do mapa [m]'),
        DeclareLaunchArgument('robot_radius', default_value='0.36',
                              description='Raio circunscrito do youBot [m], '
                                          'medido na cena (footprint de '
                                          '0,42 x 0,57 m)'),
        DeclareLaunchArgument('inflate', default_value='0.05',
                              description='Margem além do raio do robô [m]'),
        DeclareLaunchArgument('grasp_reach', default_value='0.233',
                              description='Raio de preensão, do eixo da junta 0 '
                                          '[m]. O envelope medido vai de 0,15 '
                                          'a 0,315 m'),
        DeclareLaunchArgument('grasp_height', default_value='0.028',
                              description='Altura da ponta da garra ao agarrar '
                                          '[m]. Menos que isso raspa o chão'),
        DeclareLaunchArgument('cartesian_speed', default_value='0.08',
                              description='Velocidade da ponta da garra [m/s]. '
                                          'Mais rápido arranca a banana da mão'),
        DeclareLaunchArgument('cmd_gain', default_value='1.56',
                              description='Compensação do escorregamento das '
                                          'rodas suecas (1/0,64, medido em '
                                          'scene/calibrate_base.py)'),
        DeclareLaunchArgument('v_max', default_value='0.125',
                              description='Teto de avanço da base [m/s]. Acima '
                                          'disso o motor da roda satura'),
        DeclareLaunchArgument('attempts', default_value='2',
                              description='Tentativas por banana antes de '
                                          'descartá-la'),
        DeclareLaunchArgument('show_map', default_value='true',
                              description='Desenhar o mapa em texto no log'),
        DeclareLaunchArgument('stepping', default_value='true',
                              description='true: a ponte dá os passos da '
                                          'simulação, um ciclo de controle por '
                                          'passo'),
        DeclareLaunchArgument('stop_when_done', default_value='true',
                              description='Parar a simulação ao terminar'),
        DeclareLaunchArgument('rviz', default_value='1',
                              description='1 abre o rviz com mapa e caminho'),
    ]

    # Todos no mesmo namespace: os tópicos relativos se encontram sem remapear.
    namespace = 'youbot'

    bridge = Node(
        package='youbot_banana',
        namespace=namespace,
        executable='coppelia_bridge',
        name='coppelia_bridge',
        output='screen',
        parameters=[{
            'robot': LaunchConfiguration('robot'),
            'port': ParameterValue(LaunchConfiguration('port'), value_type=int),
            'cmd_gain': ParameterValue(LaunchConfiguration('cmd_gain'),
                                       value_type=float),
            'stepping': ParameterValue(LaunchConfiguration('stepping'),
                                       value_type=bool),
            'stop_on_done': ParameterValue(LaunchConfiguration('stop_when_done'),
                                           value_type=bool),
        }],
        # Quando a ponte sai (missão terminada ou Ctrl+C), o launch inteiro
        # encerra junto, em vez de deixar os outros nós sem simulador.
        on_exit=Shutdown(reason='ponte encerrada'),
    )

    base_control = Node(
        package='youbot_banana',
        namespace=namespace,
        executable='base_control',
        name='base_control',
        output='screen',
        parameters=[{
            'v_max': ParameterValue(LaunchConfiguration('v_max'),
                                    value_type=float),
        }],
    )

    arm_control = Node(
        package='youbot_banana',
        namespace=namespace,
        executable='arm_control',
        name='arm_control',
        output='screen',
        parameters=[{
            'cartesian_speed': ParameterValue(
                LaunchConfiguration('cartesian_speed'), value_type=float),
        }],
    )

    mission = Node(
        package='youbot_banana',
        namespace=namespace,
        executable='mission',
        name='mission',
        output='screen',
        parameters=[{
            'resolution': ParameterValue(LaunchConfiguration('resolution'),
                                         value_type=float),
            'robot_radius': ParameterValue(LaunchConfiguration('robot_radius'),
                                           value_type=float),
            'safety_margin': ParameterValue(LaunchConfiguration('inflate'),
                                            value_type=float),
            'grasp_reach': ParameterValue(LaunchConfiguration('grasp_reach'),
                                          value_type=float),
            'grasp_height': ParameterValue(LaunchConfiguration('grasp_height'),
                                           value_type=float),
            'attempts_per_banana': ParameterValue(
                LaunchConfiguration('attempts'), value_type=int),
            'show_map': ParameterValue(LaunchConfiguration('show_map'),
                                       value_type=bool),
            'stop_when_done': ParameterValue(
                LaunchConfiguration('stop_when_done'), value_type=bool),
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(share, 'config', 'youbot_banana.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        args + [bridge, base_control, arm_control, mission, rviz])
