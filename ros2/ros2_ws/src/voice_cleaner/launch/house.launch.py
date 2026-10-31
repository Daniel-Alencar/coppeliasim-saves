"""Sobe a casa inteira: voz -> navegação -> CoppeliaSim, com o rviz aberto.

    ros2 launch voice_cleaner house.launch.py

Cinco nós, cada um com um papel:

    voice_listener   microfone -> texto
    command_parser   texto -> comando
    house_map        a planta: mapa, rótulos e em que cômodo o robô está
    house_navigator  comando -> A* -> waypoints
    position_control waypoint -> cmd_vel
    coppelia_bridge  cmd_vel -> motores da cena, e a pose de volta

Argumentos mais úteis:

    console:=1    teclado em vez de microfone (não precisa de áudio nenhum)
    fake:=1       mundo de mentira em vez do CoppeliaSim (não precisa da cena)
    rviz:=0       sem abrir o rviz
    save_map:=/tmp/casa   grava o mapa como .pgm + .yaml ao subir

O launch NÃO constrói as paredes na cena: isso é um passo separado, feito uma
vez por cena com o house_builder (ver o README). Separado porque construir
mexe na cena aberta, e isso não é coisa para acontecer a cada vez que se sobe
o sistema.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='myRobot',
        description='Namespace dos tópicos')
    console_arg = DeclareLaunchArgument(
        'console', default_value='0',
        description='1 troca o microfone pelo teclado')
    fake_arg = DeclareLaunchArgument(
        'fake', default_value='0',
        description='1 usa o mundo de mentira em vez do CoppeliaSim')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='1',
        description='0 não abre o rviz')
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='',
        description='Pasta do modelo Vosk; vazio baixa o modelo do idioma')
    device_arg = DeclareLaunchArgument(
        'device', default_value='',
        description='Entrada de áudio (índice ou parte do nome)')
    grammar_arg = DeclareLaunchArgument(
        'grammar', default_value='false',
        description='true restringe o reconhecedor ao vocabulário dos comandos')
    min_score_arg = DeclareLaunchArgument(
        'min_score', default_value='1',
        description='Palavras que a frase precisa casar para virar comando')
    wake_word_arg = DeclareLaunchArgument(
        'wake_word', default_value='',
        description='Palavra de ativação; vazio aceita qualquer frase')
    resolution_arg = DeclareLaunchArgument(
        'resolution', default_value='0.05',
        description='Lado da célula do mapa, em metros')
    safety_margin_arg = DeclareLaunchArgument(
        'safety_margin', default_value='0.06',
        description='Folga somada ao raio do robô na inflação das paredes')
    save_map_arg = DeclareLaunchArgument(
        'save_map', default_value='',
        description='Caminho para gravar o mapa como .pgm + .yaml')
    show_map_arg = DeclareLaunchArgument(
        'show_map', default_value='0',
        description='1 desenha a grade e o caminho no terminal a cada plano')
    host_arg = DeclareLaunchArgument(
        'host', default_value='localhost',
        description='Onde está o CoppeliaSim')
    port_arg = DeclareLaunchArgument(
        'port', default_value='23000',
        description='Porta da ZeroMQ Remote API. Mude se houver mais de uma '
                    'instância do CoppeliaSim aberta, para não dirigir a errada')

    namespace = LaunchConfiguration('namespace')
    usa_console = PythonExpression(["'", LaunchConfiguration('console'),
                                    "' not in ('0', 'false', 'False')"])
    usa_fake = PythonExpression(["'", LaunchConfiguration('fake'),
                                 "' not in ('0', 'false', 'False')"])
    abre_rviz = PythonExpression(["'", LaunchConfiguration('rviz'),
                                  "' not in ('0', 'false', 'False')"])

    # --- A voz ---------------------------------------------------------------
    voice_listener = Node(
        package='voice_cleaner', executable='voice_listener',
        name='voice_listener', namespace=namespace, output='screen',
        condition=IfCondition(PythonExpression(['not (', usa_console, ')'])),
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': LaunchConfiguration('device'),
            'grammar': ParameterValue(LaunchConfiguration('grammar'),
                                      value_type=bool),
        }],
    )
    command_console = Node(
        package='voice_cleaner', executable='command_console',
        name='command_console', namespace=namespace, output='screen',
        condition=IfCondition(usa_console),
    )
    command_parser = Node(
        package='voice_cleaner', executable='command_parser',
        name='command_parser', namespace=namespace, output='screen',
        parameters=[{
            'min_score': ParameterValue(LaunchConfiguration('min_score'),
                                        value_type=int),
            'wake_word': LaunchConfiguration('wake_word'),
        }],
    )

    # --- A casa e a navegação ------------------------------------------------
    house_map = Node(
        package='voice_cleaner', executable='house_map',
        name='house_map', namespace=namespace, output='screen',
        parameters=[{
            'resolution': ParameterValue(LaunchConfiguration('resolution'),
                                         value_type=float),
            'save_path': LaunchConfiguration('save_map'),
        }],
    )
    house_navigator = Node(
        package='voice_cleaner', executable='house_navigator',
        name='house_navigator', namespace=namespace, output='screen',
        parameters=[{
            'safety_margin': ParameterValue(
                LaunchConfiguration('safety_margin'), value_type=float),
            'show_map': ParameterValue(LaunchConfiguration('show_map'),
                                       value_type=bool),
        }],
    )
    position_control = Node(
        package='voice_cleaner', executable='position_control',
        name='position_control', namespace=namespace, output='screen',
    )

    # --- O mundo: a cena, ou o mundo de mentira ------------------------------
    bridge = Node(
        package='voice_cleaner', executable='coppelia_bridge',
        name='coppelia_bridge', namespace=namespace, output='screen',
        condition=IfCondition(PythonExpression(['not (', usa_fake, ')'])),
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': ParameterValue(LaunchConfiguration('port'),
                                   value_type=int),
            # Esta demonstração não usa a câmera: as imagens só custariam CPU.
            'publish_images': False,
            # O mapa de bananas e poops não entra aqui; a casa vem do
            # house_map. Sem isto a ponte ficaria publicando um mapa de
            # bananas e poops que ninguém consome.
            'map_source': 'none',
        }],
    )
    fake_world = Node(
        package='voice_cleaner', executable='fake_world',
        name='fake_world', namespace=namespace, output='screen',
        condition=IfCondition(usa_fake),
        # Começa na base, no meio da sala, como a cena construída. As bananas
        # e os poops do mundo de mentira ficam como estão: nesta cadeia
        # ninguém os consome, e lista vazia não é parâmetro válido no ROS 2
        # (sem elemento, o tipo não dá para inferir).
        parameters=[{'start': [0.05, -1.525, 0.0]}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(abre_rviz),
        arguments=['-d', os.path.join(
            get_package_share_directory('voice_cleaner'),
            'config', 'house.rviz')],
    )

    return LaunchDescription([
        namespace_arg, console_arg, fake_arg, rviz_arg,
        model_path_arg, device_arg, grammar_arg,
        min_score_arg, wake_word_arg,
        resolution_arg, safety_margin_arg, save_map_arg, show_map_arg,
        host_arg, port_arg,
        voice_listener, command_console, command_parser,
        house_map, house_navigator, position_control,
        bridge, fake_world, rviz,
    ])
