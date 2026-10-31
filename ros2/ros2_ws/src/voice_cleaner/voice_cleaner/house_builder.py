"""Constrói as paredes da casa dentro da cena do CoppeliaSim e sai.

Ferramenta de uma tacada: conecta na cena pela ZeroMQ Remote API, cria um
cuboide por trecho de parede de house.py e põe o robô na base. Não é um nó que
fica rodando — ele faz o serviço, informa o que criou e termina.

    ros2 run voice_cleaner house_builder --ros-args -p robot:=/myRobot

Por que construir por script, e não guardar um .ttt pronto
----------------------------------------------------------
Porque as paredes do simulador e o mapa do rviz precisam ser a mesma casa. Num
.ttt elas seriam medidas congeladas num arquivo binário, e qualquer mudança na
planta teria de ser repetida à mão em dois lugares — até o dia em que
divergissem e o robô planejasse por uma porta que a cena não tem. Vindo de
house.py, as duas saem sempre iguais, e mudar a casa é mudar um módulo Python.

O que ele cria
--------------
Um dummy /House com um cuboide filho por trecho de parede, estáticos e
respondíveis (o robô bate neles, não os empurra). Rodar de novo apaga o /House
anterior antes de construir, então o script é idempotente: dá para ajustar a
planta e reconstruir sem acumular paredes velhas na cena.

E o robô: se a cena não tiver um, ele carrega o modelo myRobot.ttm que vem no
pacote e o põe na base, no meio da sala. Assim dá para partir de uma cena nova
e vazia — só o chão —, que é bem mais limpo do que herdar a cena das bananas
com as texturas de sujeira e os scripts dela. Se a cena já tiver o robô, ele é
só reposicionado, e nada é carregado.

Os scripts que brigam pelos motores
-----------------------------------
A cena das bananas traz dois scripts no robô (`controler` e
`python_controler`) que escrevem velocidade nas juntas a cada passo. Com eles
ligados o `cmd_vel` da ponte é sobrescrito e o robô não obedece — é o defeito
mais comum de montar isto. O builder os desabilita e diz quais desabilitou. O
modelo que vem no pacote já sai com eles desligados.

O que ele NÃO faz
-----------------
Não cria o chão: ele vem da cena (uma cena nova do CoppeliaSim já tem um). E
não mexe em scripts fora da árvore do robô — o `/buildScene` da cena das
bananas, que espalha sujeira dentro da casa, continua sendo manual.
"""

import os

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node

from voice_cleaner import house


class HouseBuilder(Node):
    """Cria as paredes da casa na cena aberta do CoppeliaSim."""

    def __init__(self):
        super().__init__('house_builder')

        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 23000)
        self.declare_parameter('robot', '/myRobot')
        # Nome do dummy que segura as paredes. Reconstruir apaga este e os
        # filhos dele, e nada mais da cena.
        self.declare_parameter('parent', 'House')
        # Mover o robô para a base (meio da sala) depois de construir. É o que
        # faz a demonstração começar com o robô dentro de um cômodo conhecido.
        self.declare_parameter('place_robot', True)
        # Altura em que o robô é posto; negativo mantém a z atual dele (ou a
        # do modelo, quando ele acabou de ser carregado).
        self.declare_parameter('robot_z', -1.0)
        # Modelo a carregar quando a cena não tem robô. Vazio usa o myRobot.ttm
        # instalado com o pacote.
        self.declare_parameter('robot_model', '')
        # Desabilitar os scripts do robô que escrevem nos motores. Ligados,
        # eles sobrescrevem o cmd_vel e o robô não obedece ao comando de voz.
        self.declare_parameter('disable_motor_scripts', True)
        self.declare_parameter('wall_color', [0.75, 0.75, 0.78])

    # =========================================================================
    #  Construção
    # =========================================================================

    def build(self):
        """Conecta, limpa o que havia e cria as paredes. Devolve 0 se deu certo."""
        try:
            from coppeliasim_zmqremoteapi_client import RemoteAPIClient
        except ImportError:
            self.get_logger().error(
                'falta o cliente da Remote API no Python do sistema:\n'
                '  /usr/bin/python3 -m pip install --user '
                '--break-system-packages coppeliasim-zmqremoteapi-client')
            return 1

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        try:
            sim = RemoteAPIClient(host=host, port=port).require('sim')
            sim.getSimulationTime()     # primeira chamada: confirma a conexão
        except Exception as erro:
            self.get_logger().error(
                f'não consegui falar com o CoppeliaSim em {host}:{port} '
                f'({erro}).\nA cena está aberta? A Remote API usa a porta '
                '23000 e vem ligada por padrão no CoppeliaSim 4.x.')
            return 1

        self.clear(sim)
        parent = self.create_parent(sim)
        criadas = self.create_walls(sim, parent)
        self.get_logger().info(
            f'{criadas} trechos de parede criados sob /'
            f'{self.get_parameter("parent").value}')
        self.place_robot(sim)
        self.get_logger().info(
            'casa construída. Dê play e suba o launch: '
            'ros2 launch voice_cleaner house.launch.py')
        return 0

    def clear(self, sim):
        """Apaga um /House de uma construção anterior, se existir."""
        nome = self.get_parameter('parent').value
        handle = sim.getObject(f'/{nome}', {'noError': True})
        if handle is None or handle < 0:
            return
        # A árvore inteira: o dummy e as paredes penduradas nele.
        sim.removeObjects(sim.getObjectsInTree(handle))
        self.get_logger().info(f'/{nome} anterior removido')

    def create_parent(self, sim):
        """O dummy que segura as paredes, para a cena ficar organizada."""
        nome = self.get_parameter('parent').value
        handle = sim.createDummy(0.02)
        sim.setObjectAlias(handle, nome)
        sim.setObjectPosition(handle, [0.0, 0.0, 0.0], sim.handle_world)
        return handle

    def create_walls(self, sim, parent):
        """Um cuboide por trecho de parede, estático e respondível."""
        cor = list(self.get_parameter('wall_color').value)
        criadas = 0
        for i, (cx, cy, sx, sy, sz) in enumerate(house.wall_boxes()):
            handle = sim.createPrimitiveShape(
                sim.primitiveshape_cuboid, [sx, sy, sz], 0)
            sim.setObjectAlias(handle, f'Wall{i:02d}')
            # Metade da altura em z: o cuboide nasce centrado, e a parede tem
            # de apoiar no chão, não afundar nele.
            sim.setObjectPosition(handle, [cx, cy, sz / 2.0], sim.handle_world)
            sim.setObjectParent(handle, parent, True)
            # Estática: não cai nem é empurrada. Respondível: o robô bate.
            sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 1)
            sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse,
                              cor)
            criadas += 1
        return criadas

    def place_robot(self, sim):
        """Garante um robô na cena e o põe na base, no meio da sala."""
        if not self.get_parameter('place_robot').value:
            return
        caminho = self.get_parameter('robot').value
        handle = sim.getObject(caminho, {'noError': True})
        if handle is None or handle < 0:
            handle = self.load_robot(sim, caminho)
            if handle is None:
                return

        z = self.get_parameter('robot_z').value
        if z < 0.0:
            z = sim.getObjectPosition(handle, sim.handle_world)[2]
        x, y = house.BASE
        sim.setObjectPosition(handle, [x, y, z], sim.handle_world)
        sim.setObjectOrientation(handle, [0.0, 0.0, 0.0], sim.handle_world)
        self.get_logger().info(
            f'{caminho} posto na base ({x:+.2f}, {y:+.2f}), no meio da sala')
        self.quiet_motor_scripts(sim, handle)

    def load_robot(self, sim, caminho):
        """Carrega o modelo do robô numa cena que ainda não tem um.

        É o que permite partir de uma cena nova e vazia. Devolve o handle, ou
        None se não houver modelo para carregar.
        """
        modelo = self.get_parameter('robot_model').value
        if not modelo:
            modelo = os.path.join(
                get_package_share_directory('voice_cleaner'),
                'coppeliasim', 'robot', 'myRobot.ttm')
        if not os.path.isfile(modelo):
            self.get_logger().warn(
                f'não achei {caminho} na cena nem o modelo em {modelo}: as '
                'paredes foram criadas, mas a casa ficou sem robô. Abra uma '
                'cena que tenha o robô, ou passe -p robot_model:=<arquivo>.')
            return None
        try:
            handle = sim.loadModel(modelo)
        except Exception as erro:
            self.get_logger().error(f'não consegui carregar {modelo}: {erro}')
            return None
        # O alias do modelo carregado vira o nome que a ponte procura; sem
        # isto, uma cena com outro robô geraria "myRobot#0" e a ponte não o
        # encontraria.
        sim.setObjectAlias(handle, caminho.lstrip('/'))
        self.get_logger().info(f'robô carregado de {modelo}')
        return handle

    def quiet_motor_scripts(self, sim, robot):
        """Desabilita os scripts do robô que escrevem nos motores.

        Ligados, eles sobrescrevem o cmd_vel a cada passo de simulação e o
        robô ignora o comando de voz. Dá para religá-los na árvore da cena.
        """
        if not self.get_parameter('disable_motor_scripts').value:
            return
        desabilitados = []
        for h in sim.getObjectsInTree(robot, sim.sceneobject_script):
            if 'setJointTargetVelocity' in sim.getStringProperty(h, 'code'):
                sim.setObjectInt32Param(h, sim.scriptintparam_enabled, 0)
                desabilitados.append(sim.getObjectAlias(h, 2))
        if desabilitados:
            self.get_logger().info(
                'scripts desabilitados (escreviam nos motores e anulariam o '
                'cmd_vel): ' + ', '.join(desabilitados))


def main(args=None):
    rclpy.init(args=args)
    node = HouseBuilder()
    try:
        codigo = node.build()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return codigo


if __name__ == '__main__':
    raise SystemExit(main())
