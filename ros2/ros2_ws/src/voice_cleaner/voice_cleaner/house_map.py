"""A casa como o robô a conhece: publica o mapa e diz em que cômodo ele está.

Este nó é o dono da planta. Ele não planeja e não mexe nos motores: publica a
casa para o rviz e, a cada pose que chega, responde a pergunta "onde estou".

    pose (PoseStamped)  --->  | house_map |  --->  map         OccupancyGrid
                                             --->  house/rooms MarkerArray
                                             --->  house/room  String

O mapa sai com QoS **transient local**: a última mensagem fica guardada no
publicador, então o rviz e o planejador recebem a casa mesmo subindo depois
deste nó. Sem isso o mapa só apareceria se tudo subisse na ordem certa, e um
rviz aberto depois ficaria com a tela vazia.

O mapa publicado é o **cru**: as paredes com a espessura que elas têm. A
inflação pelo raio do robô é assunto do planejador, que a aplica no mapa que
recebe daqui. Publicar o mapa já inflado deixaria a casa com paredes gordas na
tela e esconderia a largura real das portas.

Em que cômodo ele está
----------------------
É uma consulta geométrica, não localização probabilística: a pose vem da cena
pelo coppelia_bridge, e house.room_at diz qual retângulo contém o ponto. Os
cômodos ladrilham a casa inteira, então a resposta existe sempre que o robô
está dentro das paredes; fora delas o tópico publica "fora da casa", que é o
que se vê se a cena e a planta estiverem desalinhadas.

O tópico só é publicado quando a resposta **muda**, e também com QoS transient
local: assim `ros2 topic echo` mostra o cômodo atual na hora, em vez de esperar
o robô atravessar uma porta.
"""

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from voice_cleaner import house

#: O que o tópico diz quando a pose cai fora das paredes da casa.
OUTSIDE = 'fora da casa'


class HouseMap(Node):
    """Publica a planta da casa e o cômodo onde o robô está."""

    def __init__(self):
        super().__init__('house_map')

        # Célula de 5 cm: o bastante para a porta de 90 cm ter 18 células, e
        # para a parede de 8 cm não sumir no arredondamento.
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('frame_id', 'world')
        # Vazio não grava nada. Com um caminho, grava .pgm + .yaml ao subir,
        # para o mapa poder ser aberto fora do rviz (nav2_map_server, ou só
        # olhado como imagem).
        self.declare_parameter('save_path', '')
        # Altura dos nomes dos cômodos no rviz, em metros.
        self.declare_parameter('label_size', 0.28)

        self.frame_id = self.get_parameter('frame_id').value
        self.room = None

        latched = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', latched)
        self.rooms_pub = self.create_publisher(MarkerArray, 'house/rooms',
                                               latched)
        self.room_pub = self.create_publisher(String, 'house/room', latched)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)

        self.publish_map()
        self.publish_labels()
        self.save_if_asked()

    # =========================================================================
    #  O mapa
    # =========================================================================

    def publish_map(self):
        """Monta e publica a OccupancyGrid da planta."""
        resolution = self.get_parameter('resolution').value
        cells, info = house.occupancy(resolution)

        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = float(resolution)
        msg.info.width = info['cols']
        msg.info.height = info['rows']
        msg.info.origin.position.x = float(info['origin'][0])
        msg.info.origin.position.y = float(info['origin'][1])
        msg.info.origin.orientation.w = 1.0
        # Linhas de baixo para cima, igual à matriz: a OccupancyGrid usa a
        # mesma ordem, então não há transposição a fazer.
        msg.data = [100 if value else 0 for row in cells for value in row]
        self.map_pub.publish(msg)

        livres = sum(1 for value in msg.data if value == 0)
        self.get_logger().info(
            f'casa publicada: {info["cols"]}x{info["rows"]} células de '
            f'{resolution:.2f} m ({livres} livres), '
            f'{len(house.ROOMS)} cômodos, {len(house.walls())} trechos de parede')

    def publish_labels(self):
        """Um rótulo com o nome de cada cômodo, no centro dele."""
        array = MarkerArray()
        size = self.get_parameter('label_size').value
        for i, room in enumerate(house.ROOMS):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.ns = 'rooms'
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.text = room.name
            marker.pose.position.x, marker.pose.position.y = room.center
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            marker.scale.z = float(size)
            marker.color.r, marker.color.g = 0.1, 0.1
            marker.color.b, marker.color.a = 0.1, 1.0
            array.markers.append(marker)
        self.rooms_pub.publish(array)

    def save_if_asked(self):
        destino = self.get_parameter('save_path').value
        if not destino:
            return
        try:
            pgm, yaml = house.save_pgm(
                destino, self.get_parameter('resolution').value)
        except OSError as erro:
            self.get_logger().error(f'não consegui gravar o mapa: {erro}')
            return
        self.get_logger().info(f'mapa gravado em {pgm} e {yaml}')

    # =========================================================================
    #  Onde o robô está
    # =========================================================================

    def pose_callback(self, msg: PoseStamped):
        """Traduz a pose em nome de cômodo e publica quando muda."""
        room = house.room_at(msg.pose.position.x, msg.pose.position.y) or OUTSIDE
        if room == self.room:
            return
        anterior = self.room
        self.room = room
        self.room_pub.publish(String(data=room))
        if anterior is None:
            self.get_logger().info(f'o robô está no(a) {room}')
        else:
            self.get_logger().info(f'{anterior} -> {room}')


def main(args=None):
    rclpy.init(args=args)
    node = HouseMap()
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
