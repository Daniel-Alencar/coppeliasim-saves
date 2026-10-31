r"""Comando de voz -> caminho: "vá para a cozinha" vira waypoints até lá.

É aqui que a camada de voz encontra a navegação. O nó assina o comando que o
command_parser publica, descobre o ponto do cômodo pedido, planeja com A* no
mapa da casa e manda um waypoint de cada vez para o controlador de posição.

    voice/command (String, JSON) --->  |                 | ---> goal (PoseStamped)
    map          (OccupancyGrid) --->  | house_navigator | ---> path (nav_msgs/Path)
    pose         (PoseStamped)   --->  |                 | ---> house/task (String)

Ele não mexe nos motores: quem converte o waypoint em velocidade é o
position_control, e quem fala com a cena é o coppelia_bridge. E não conhece a
planta: o mapa chega pelo tópico, então o robô planeja **no mapa que você vê no
rviz**, e não numa cópia própria da casa que poderia divergir.

O ciclo
-------
    1. chega um comando com um lugar ("cozinha")
    2. monta a grade de planejamento: o mapa do tópico, com as paredes infladas
       pelo raio do robô (house_grid.py)
    3. roda o A* da célula do robô até a do cômodo (a_star.py)
    4. simplifica o caminho nos pontos onde ele muda de direção
    5. publica o waypoint atual em `goal`; a menos de waypoint_tolerance dele,
       publica o próximo
    6. no último, anuncia a chegada e pega o próximo cômodo da fila

Os comandos que movem o robô
----------------------------
    ir_para      local   vai até o cômodo
    limpar_local local   idem (limpar de verdade é a etapa seguinte; o robô vai
                         até lá e anuncia)
    voltar_base          vai até a base, no meio da sala
    limpar_tudo          passa por todos os cômodos, um a um
    mapear               idem: percorrer a casa é o mesmo trajeto
    parar                cancela a fila e para onde está
    pausar / continuar   para sem perder o caminho, e retoma de onde parou
    status               diz onde está, para onde vai e quanto falta

Pedir "a casa toda" em limpar_local cai no mesmo percurso de limpar_tudo: é o
que a pessoa quis dizer.

Como ele para
-------------
O nó não publica cmd_vel. Para parar o robô ele publica como `goal` a pose
**atual** dele: o controlador de posição vê que já chegou e zera as rodas. Isso
evita um segundo caminho de comando para os motores, que poderia brigar com o
controlador.
"""

import json
import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from voice_cleaner import a_star, house
from voice_cleaner.house_grid import Grid


def yaw_quaternion(yaw):
    """Quaternion (z, w) de uma rotação só em torno do eixo vertical."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class HouseNavigator(Node):
    """Leva o robô ao cômodo pedido por voz, planejando com A*."""

    def __init__(self):
        super().__init__('house_navigator')

        # --- Grade de planejamento -------------------------------------------
        # Raio do robô (0,16 m: 0,32 m de diâmetro, medido na cena) mais a
        # folga. É o que o A* infla em volta das paredes; apertar demais fecha
        # as portas e o sintoma é "sem caminho" com o mapa aberto na tela.
        self.declare_parameter('robot_radius', 0.16)       # [m]
        self.declare_parameter('safety_margin', 0.06)      # [m]
        self.declare_parameter('diagonal', True)           # vizinhança de 8

        # --- Execução do caminho ---------------------------------------------
        self.declare_parameter('waypoint_tolerance', 0.15)  # [m]
        self.declare_parameter('goal_tolerance', 0.12)      # último ponto [m]
        # Sem chegar ao waypoint neste tempo, replaneja: o robô pode ter
        # escorregado ou a pose ter dado um salto.
        self.declare_parameter('replan_timeout', 30.0)      # [s]
        self.declare_parameter('max_replans', 3)
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('show_map', False)

        self.frame_id = self.get_parameter('frame_id').value

        self.grid = None          # grade inflada, montada quando o mapa chega
        self.robot = None         # (x, y, yaw) da última pose
        self.queue = []           # cômodos ainda por visitar
        self.waypoints = []       # caminho atual, em metros
        self.index = 0            # waypoint sendo perseguido
        self.since = 0.0          # quando o waypoint atual foi publicado
        self.replans = 0
        self.paused = False

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal', 10)
        self.path_pub = self.create_publisher(Path, 'path', latched)
        self.task_pub = self.create_publisher(String, 'house/task', latched)

        self.create_subscription(OccupancyGrid, 'map', self.map_callback,
                                 latched)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)
        self.create_subscription(String, 'voice/command',
                                 self.command_callback, 10)

        self.announce('parado')
        self.get_logger().info(
            'navegador da casa pronto. cômodos: ' + ', '.join(house.room_names()))

    # =========================================================================
    #  Entradas
    # =========================================================================

    def map_callback(self, msg: OccupancyGrid):
        """Monta a grade de planejamento a partir do mapa publicado."""
        width, height = msg.info.width, msg.info.height
        cells = [[1 if msg.data[row * width + col] > 50 else 0
                  for col in range(width)]
                 for row in range(height)]
        inflation = (self.get_parameter('robot_radius').value
                     + self.get_parameter('safety_margin').value)
        self.grid = Grid(
            cells,
            (msg.info.origin.position.x, msg.info.origin.position.y),
            msg.info.resolution,
            inflation_radius=inflation)
        self.get_logger().info(
            f'mapa recebido: {width}x{height} células de '
            f'{msg.info.resolution:.2f} m, inflação {inflation:.2f} m')

    def pose_callback(self, msg: PoseStamped):
        """Guarda a pose e, se há caminho, avança nele."""
        self.robot = (msg.pose.position.x, msg.pose.position.y,
                      2.0 * math.atan2(msg.pose.orientation.z,
                                       msg.pose.orientation.w))
        if self.waypoints and not self.paused:
            self.follow()

    def command_callback(self, msg: String):
        """Um comando de voz: decide o que fazer com ele."""
        try:
            comando = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'comando ilegível: {msg.data!r}')
            return
        intent = comando.get('intent')
        local = (comando.get('args') or {}).get('local')
        self.dispatch(intent, local)

    # =========================================================================
    #  O que fazer com cada comando
    # =========================================================================

    def dispatch(self, intent, local):
        """Traduz a intenção em ação de navegação."""
        if intent == 'parar':
            self.cancel('comando: parar')
        elif intent == 'pausar':
            self.pause()
        elif intent == 'continuar':
            self.resume()
        elif intent == 'status':
            self.report()
        elif intent == 'voltar_base':
            self.go_to_rooms(['sala'], 'voltando para a base')
        elif intent in ('limpar_tudo', 'mapear'):
            self.go_to_rooms(house.room_names(), f'percorrendo a casa ({intent})')
        elif intent in ('ir_para', 'limpar_local'):
            self.go_to_place(intent, local)
        elif intent in ('limpar_aqui', 'ajuda', 'ajustar_velocidade'):
            # Intenções que o robô entende mas que não mudam o destino. Dizer
            # isso em voz alta evita a dúvida "ele ouviu e não fez nada?".
            self.get_logger().info(
                f'{intent}: nada a navegar (este nó só cuida de deslocamento)')

    def go_to_place(self, intent, local):
        """ir_para / limpar_local: resolve o lugar pedido e manda ir."""
        if not local:
            self.get_logger().warn(
                f'{intent} sem lugar: diga "vá para a cozinha", por exemplo')
            return
        if local == 'casa toda':
            self.go_to_rooms(house.room_names(), 'percorrendo a casa toda')
            return
        if local not in house.BY_NAME:
            self.get_logger().warn(
                f'"{local}" não é um cômodo desta casa. Tenho: '
                + ', '.join(house.room_names()))
            return
        verbo = 'indo para' if intent == 'ir_para' else 'indo limpar'
        self.go_to_rooms([local], f'{verbo} o(a) {local}')

    def go_to_rooms(self, rooms, motivo):
        """Enfileira cômodos e começa a ir ao primeiro."""
        if self.grid is None:
            self.get_logger().warn(
                'ainda não recebi o mapa; o house_map está rodando?')
            return
        if self.robot is None:
            self.get_logger().warn(
                'ainda não recebi a pose; a ponte está rodando e a simulação '
                'em play?')
            return
        self.queue = list(rooms)
        self.paused = False
        self.get_logger().info(motivo)
        self.plan()

    def pause(self):
        if not self.waypoints:
            self.get_logger().info('pausar: não estou indo a lugar nenhum')
            return
        self.paused = True
        self.hold()
        self.announce(f'pausado a caminho do(a) {self.queue[0]}')
        self.get_logger().info(
            f'pausado no waypoint {self.index + 1}/{len(self.waypoints)}; '
            'diga "continuar" para retomar')

    def resume(self):
        if not self.paused:
            self.get_logger().info('continuar: não estava pausado')
            return
        self.paused = False
        self.get_logger().info(f'retomando para o(a) {self.queue[0]}')
        self.send_goal()

    def cancel(self, motivo):
        self.queue = []
        self.waypoints = []
        self.index = 0
        self.paused = False
        self.hold()
        self.publish_path([])
        self.announce('parado')
        self.get_logger().info(f'{motivo}: parei e esvaziei a fila')

    def report(self):
        """O status, que é o que o comando de voz "como está" pede."""
        onde = (house.room_at(self.robot[0], self.robot[1])
                if self.robot else None)
        if not self.waypoints:
            self.get_logger().info(
                f'estou no(a) {onde or "?"}, sem destino')
            return
        faltam = len(self.waypoints) - self.index
        estado = 'pausado' if self.paused else 'indo'
        self.get_logger().info(
            f'{estado} para o(a) {self.queue[0]}, agora no(a) {onde or "?"}, '
            f'{faltam} waypoint(s) restante(s); fila: '
            + (', '.join(self.queue) or 'vazia'))

    # =========================================================================
    #  Planejamento
    # =========================================================================

    def plan(self):
        """Roda o A* até o primeiro cômodo da fila."""
        if not self.queue:
            self.finish()
            return
        destino = self.queue[0]
        start = self.grid.nearest_free(self.grid.to_cell(self.robot[:2]))
        alvo_xy = house.goal_of(destino)
        alvo = self.grid.nearest_free(self.grid.to_cell(alvo_xy))

        if start is None:
            self.get_logger().error(
                f'o robô está em ({self.robot[0]:+.2f}, {self.robot[1]:+.2f}), '
                'fora do mapa ou sem célula livre por perto. A pose da cena e a '
                'planta estão no mesmo referencial?')
            self.cancel('partida inválida')
            return
        if alvo is None:
            self.get_logger().error(
                f'o centro do(a) {destino} não tem célula livre por perto: a '
                'inflação fechou o cômodo. Reduza safety_margin.')
            self.skip(destino)
            return

        caminho, stats = a_star.search(
            self.grid, start, alvo,
            diagonal=self.get_parameter('diagonal').value)
        if caminho is None:
            self.get_logger().error(
                f'sem caminho até o(a) {destino} (expandi {stats["expanded"]} '
                'células). A porta pode estar fechada pela inflação: veja '
                'safety_margin, ou show_map:=true para olhar a grade.')
            self.skip(destino)
            return

        celulas = a_star.simplify(caminho)
        self.waypoints = [self.grid.to_world(row, col) for row, col in celulas]
        self.index = 0
        self.replans = 0
        if self.get_parameter('show_map').value:
            self.get_logger().info('\n' + self.grid.render(
                path=caminho, robot=start, goal=alvo))
        self.get_logger().info(
            f'caminho até o(a) {destino}: {len(caminho)} células, '
            f'{len(self.waypoints)} waypoints, '
            f'{stats["cost_cells"] * self.grid.resolution:.2f} m')
        self.publish_path(self.waypoints)
        self.announce(f'indo para o(a) {destino}')
        self.send_goal()

    def skip(self, destino):
        """Desiste deste cômodo e tenta o próximo da fila."""
        self.queue = self.queue[1:]
        self.waypoints = []
        self.index = 0
        if self.queue:
            self.get_logger().info(f'pulando o(a) {destino}; vou ao próximo')
            self.plan()
        else:
            self.cancel(f'não consegui chegar ao(à) {destino}')

    # =========================================================================
    #  Execução
    # =========================================================================

    def follow(self):
        """Avança no caminho quando o robô chega ao waypoint atual."""
        ultimo = self.index == len(self.waypoints) - 1
        tolerancia = (self.get_parameter('goal_tolerance').value if ultimo
                      else self.get_parameter('waypoint_tolerance').value)
        alvo = self.waypoints[self.index]
        distancia = math.hypot(alvo[0] - self.robot[0], alvo[1] - self.robot[1])

        if distancia <= tolerancia:
            if ultimo:
                self.arrive()
            else:
                self.index += 1
                self.send_goal()
            return

        atrasado = (self.now() - self.since
                    > self.get_parameter('replan_timeout').value)
        if atrasado:
            self.replans += 1
            if self.replans > self.get_parameter('max_replans').value:
                self.cancel(f'desisti do(a) {self.queue[0]} depois de '
                            f'{self.replans - 1} replanejamentos')
                return
            self.get_logger().warn(
                f'waypoint {self.index + 1} não alcançado em '
                f'{self.get_parameter("replan_timeout").value:.0f} s; '
                f'replanejando ({self.replans})')
            self.plan()

    def arrive(self):
        """Chegou ao último waypoint do cômodo da vez."""
        destino = self.queue[0]
        onde = house.room_at(self.robot[0], self.robot[1])
        if onde == destino:
            self.get_logger().info(f'cheguei no(a) {destino}')
        else:
            # Acontece se o alvo do cômodo caiu perto da divisória: avisa em
            # vez de afirmar uma chegada que o mapa desmente.
            self.get_logger().warn(
                f'cheguei ao ponto do(a) {destino}, mas a pose diz '
                f'{onde or "fora da casa"}')
        self.queue = self.queue[1:]
        self.waypoints = []
        self.index = 0
        self.hold()
        if self.queue:
            self.plan()
        else:
            self.finish()

    def finish(self):
        self.publish_path([])
        self.announce('parado')
        self.get_logger().info('fila vazia, esperando o próximo comando')

    def send_goal(self):
        """Publica o waypoint atual para o controlador de posição.

        O rumo pedido é a direção do trecho que começa nesse ponto (no último,
        a do trecho que chega nele): assim a lei de controle já alinha o robô
        com a curva seguinte, em vez de parar e girar a cada ponto.
        """
        x, y = self.waypoints[self.index]
        if self.index + 1 < len(self.waypoints):
            proximo = self.waypoints[self.index + 1]
            yaw = math.atan2(proximo[1] - y, proximo[0] - x)
        elif self.index > 0:
            anterior = self.waypoints[self.index - 1]
            yaw = math.atan2(y - anterior[1], x - anterior[0])
        else:
            yaw = self.robot[2]
        self.publish_goal(x, y, yaw)
        self.since = self.now()

    def hold(self):
        """Para o robô mandando a pose atual como alvo."""
        if self.robot is not None:
            self.publish_goal(self.robot[0], self.robot[1], self.robot[2])

    # =========================================================================
    #  Saídas
    # =========================================================================

    def publish_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z, msg.pose.orientation.w = yaw_quaternion(yaw)
        self.goal_pub.publish(msg)

    def publish_path(self, waypoints):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for x, y in waypoints:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def announce(self, texto):
        self.task_pub.publish(String(data=texto))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = HouseNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # O ros2 launch repassa um segundo Ctrl+C durante a limpeza; sem isto
        # ele interrompe o destroy_node e imprime um traceback inofensivo.
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
