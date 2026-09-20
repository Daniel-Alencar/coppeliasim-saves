r"""Sobe a ponte com o CoppeliaSim e, opcionalmente, o motorista de teste.

    ros2 launch diff_robot diff_robot.launch.py              # só a ponte
    ros2 launch diff_robot diff_robot.launch.py dummy:=1     # ponte + roteiro de teste
    ros2 launch diff_robot diff_robot.launch.py robot:=/meuRobo

Os dois argumentos deste launch têm naturezas bem diferentes, e vale entender a
distinção porque ela se repete em todo projeto ROS 2:

    robot  -> vira PARÂMETRO ROS do nó coppelia_bridge. Entra no processo, é
              lido por declare_parameter/get_parameter dentro do código e
              aparece em `ros2 param list`.

    dummy  -> NÃO entra em nenhum nó. É uma decisão de composição: escolhe
              quais processos serão iniciados. Não existe `declare_parameter`
              de dummy em lugar nenhum, e ele não aparece no grafo ROS.

Atenção: dummy NÃO é um interruptor entre "roteiro" e "teclado". Ele apenas
liga ou desliga o nó dummy_driver. Este launch nunca sobe teleop algum — com
dummy:=0 ninguém publica em cmd_vel e o robô fica parado até você abrir outro
terminal e rodar, por exemplo:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -r cmd_vel:=/diff_robot/cmd_vel

E nada impede os dois ao mesmo tempo: com dummy:=1 e um teleop aberto, ambos
publicam no mesmo tópico e os comandos se atropelam.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # O ros2 launch procura esta função pelo nome e usa o que ela devolve.

    # --- 1. Declarar os argumentos ------------------------------------------
    # DeclareLaunchArgument registra que este launch aceita `dummy:=...` na
    # linha de comando. Sem isso o argumento seria recusado. O default_value é
    # string porque tudo que vem da linha de comando é texto.
    # Aparece em: ros2 launch diff_robot diff_robot.launch.py --show-args
    dummy_arg = DeclareLaunchArgument(
        'dummy',
        default_value='0',
        description='1 para subir também o dummy_driver (roteiro de teste)'
    )
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='/myRobot',
        description='Caminho do robô diferencial na cena do CoppeliaSim'
    )

    # --- 2. Referenciar os valores ------------------------------------------
    # LaunchConfiguration NÃO é o valor: é uma promessa de valor, resolvida só
    # quando o launch roda. Por isso não dá para escrever `if dummy == '1'`
    # aqui — neste ponto `dummy` é um objeto de substituição, não uma string.
    dummy = LaunchConfiguration('dummy')
    robot = LaunchConfiguration('robot')

    # --- 3. Descrever o que subir -------------------------------------------
    return LaunchDescription([
        # Os Declare... precisam estar na lista para valer.
        dummy_arg,
        robot_arg,

        # A ponte, que sempre sobe.
        Node(
            package='diff_robot',
            namespace='diff_robot',
            executable='coppelia_bridge',
            name='coppelia_bridge',
            output='screen',
            # `parameters` é o caminho de entrada para DENTRO do nó: este dict
            # chega ao processo e é lido por get_parameter('robot') em
            # coppelia_bridge.py. É o que diferencia `robot` de `dummy`.
            parameters=[{'robot': robot}]
        ),

        # O motorista de teste, que sobe só quando pedido.
        Node(
            package='diff_robot',
            namespace='diff_robot',
            executable='dummy_driver',
            name='dummy_driver',
            output='screen',
            # `condition` é avaliado na hora de montar o sistema. Se der falso,
            # este processo simplesmente NÃO é iniciado — não é que ele rode
            # desligado, ele não chega a existir. É aqui, e só aqui, que o
            # argumento dummy tem efeito.
            # IfCondition aceita '1', 'true', 'True' como verdadeiro;
            # '0', 'false', 'False' como falso.
            condition=IfCondition(dummy)
        ),
    ])
