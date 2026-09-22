from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # O ros2 launch procura esta função pelo nome e usa o que ela devolve.

    # --- Referenciar os valores ------------------------------------------
    # LaunchConfiguration NÃO é o valor: é uma promessa de valor, resolvida só
    # quando o launch roda. Por isso não dá para escrever `if dummy == '1'`
    # aqui — neste ponto `dummy` é um objeto de substituição, não uma string.
    robot = LaunchConfiguration('robot')

    # --- Descrever o que subir -------------------------------------------
    return LaunchDescription([
        # A ponte, que sempre sobe.
        Node(
            package='diff_robot',
            namespace='diff_robot',
            executable='coppelia_bridge',
            name='coppelia_bridge',
            output='screen',
            # `parameters` é o caminho de entrada para DENTRO do nó: este dict
            # chega ao processo e é lido por get_parameter('robot') em
            # coppelia_bridge.py
            parameters=[{'robot': robot}]
        )
    ])
