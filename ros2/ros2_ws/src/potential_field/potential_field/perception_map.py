"""Monta o mapa de bananas e poops a partir do que a câmera vê.

É este nó que substitui a leitura dos sinais da cena. Ele recebe as detecções
do yolo_vision, que vêm no referencial da câmera e valem só para aquele quadro,
e constrói um mapa no referencial do mundo que persiste enquanto o robô anda.

    detections/bananas (câmera)  --->  |                |
    detections/poops   (câmera)  --->  | perception_map |  ---> bananas (mundo)
    pose  (PoseStamped)          --->  |                |  ---> poops   (mundo)
    /tf                          --->  |                |

Três coisas acontecem aqui:

1. Cada detecção é levada da câmera para o mundo pela TF que a ponte publica.
2. Detecções próximas entre si são o mesmo objeto, visto em quadros
   diferentes: elas se juntam num só ponto, que é a média das observações.
   Um objeto só entra no mapa depois de ser visto min_hits vezes, o que
   descarta detecção solta e falso positivo.
3. A banana some do mapa quando o robô passa por cima dela, que é quando a
   cena a recolhe. A posição fica numa lista de coletadas para não voltar.

Os tópicos de saída são os mesmos que a ponte publicava com os sinais, então o
navigator funciona sem saber de onde veio o mapa.
"""

import math

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
import tf2_ros


class Landmark:
    """Um objeto do mapa: posição média e quantas vezes já foi visto."""

    def __init__(self, x, y, stamp):
        self.x = x
        self.y = y
        self.hits = 1
        self.last_seen = stamp

    def update(self, x, y, stamp, weight_cap):
        """Média corrida das observações, com peso que para de crescer."""
        n = min(self.hits, weight_cap)
        self.x = (self.x * n + x) / (n + 1)
        self.y = (self.y * n + y) / (n + 1)
        self.hits += 1
        self.last_seen = stamp


class PerceptionMap(Node):
    """Junta as detecções da câmera num mapa do mundo."""

    def __init__(self):
        super().__init__('perception_map')

        self.declare_parameter('world_frame', 'world')
        # Detecções a menos disto uma da outra são o mesmo objeto.
        self.declare_parameter('merge_radius', 0.35)      # [m]
        # Quantas vezes um objeto precisa ser visto para entrar no mapa.
        self.declare_parameter('min_hits', 2)
        # Acima disto, uma observação nova quase não move o ponto já estimado.
        self.declare_parameter('weight_cap', 10)
        # O robô passou por cima: a cena recolheu a banana, tira do mapa.
        self.declare_parameter('collect_radius', 0.12)    # [m]
        # Raio em que uma banana coletada bloqueia a criação de outra.
        self.declare_parameter('collected_radius', 0.30)  # [m]
        self.declare_parameter('publish_rate', 4.0)       # [Hz]
        self.declare_parameter('tf_timeout', 0.2)         # [s]
        # Girando rápido, a mesma coisa vista em quadros seguidos cai em pontos
        # diferentes: o atraso entre a imagem e a pose vira erro lateral. Nessa
        # situação as detecções ainda melhoram os marcos que já existem, mas não
        # criam marcos novos, senão o mapa se enche de cópias do mesmo objeto.
        self.declare_parameter('max_spin_to_create', 0.5)   # [rad/s]

        self.world_frame = self.get_parameter('world_frame').value
        self.bananas = []       # [Landmark]
        self.poops = []         # [Landmark]
        self.collected = []     # [(x, y)] bananas que já foram recolhidas
        self.robot = None       # (x, y) da última pose
        self.spin = 0.0         # velocidade de giro estimada [rad/s]
        self.ultima_pose = None  # (x, y, rumo, instante) anterior
        self.last_counts = (0, 0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            PoseArray, 'detections/bananas',
            lambda msg: self.detections_callback(msg, self.bananas), 10)
        self.create_subscription(
            PoseArray, 'detections/poops',
            lambda msg: self.detections_callback(msg, self.poops), 10)
        self.create_subscription(PoseStamped, 'pose', self.pose_callback, 10)

        # Mesmo QoS da ponte: quem subir depois recebe o último mapa.
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.banana_pub = self.create_publisher(PoseArray, 'bananas', map_qos)
        self.poop_pub = self.create_publisher(PoseArray, 'poops', map_qos)

        self.create_timer(
            1.0 / self.get_parameter('publish_rate').value, self.publish_map)
        self.get_logger().info('mapa por percepção pronto, esperando detecções')

    # =========================================================================
    #  Entradas
    # =========================================================================

    def pose_callback(self, msg: PoseStamped):
        """Guarda a pose, estima o giro e recolhe as bananas por onde passou."""
        self.robot = (msg.pose.position.x, msg.pose.position.y)
        rumo = 2.0 * math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)
        agora = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        if self.ultima_pose is not None:
            _, _, rumo0, t0 = self.ultima_pose
            dt = agora - t0
            if dt > 1e-3:
                d = math.atan2(math.sin(rumo - rumo0), math.cos(rumo - rumo0))
                self.spin = abs(d) / dt
        self.ultima_pose = (self.robot[0], self.robot[1], rumo, agora)
        radius = self.get_parameter('collect_radius').value
        restantes = []
        for mark in self.bananas:
            if math.hypot(mark.x - self.robot[0], mark.y - self.robot[1]) <= radius:
                self.collected.append((mark.x, mark.y))
                self.get_logger().info(
                    f'banana recolhida em ({mark.x:.2f}, {mark.y:.2f}); '
                    f'{len(self.collected)} no total'
                )
            else:
                restantes.append(mark)
        self.bananas = restantes

    def detections_callback(self, msg: PoseArray, alvo):
        """Leva as detecções do quadro para o mundo e funde com o mapa."""
        if not msg.poses:
            return
        espera = rclpy.duration.Duration(
            seconds=self.get_parameter('tf_timeout').value)
        try:
            # Com o robô andando, a pose certa é a do instante da imagem, não a
            # mais recente: a 0,25 m/s e 1 rad/s, 100 ms de atraso já desloca o
            # objeto alguns centímetros e gira o referencial vários graus.
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame, msg.header.frame_id,
                    Time.from_msg(msg.header.stamp), timeout=espera)
            except tf2_ros.ExtrapolationException:
                # Imagem mais nova que a última TF: usa a mais recente.
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame, msg.header.frame_id, Time(), timeout=espera)
        except Exception as erro:
            self.get_logger().warn(
                f'sem TF de {msg.header.frame_id} para {self.world_frame}: {erro}',
                throttle_duration_sec=5.0)
            return

        for pose in msg.poses:
            x, y = self.to_world(tf, pose)
            self.merge(alvo, x, y, msg.header.stamp)

    def to_world(self, tf, pose):
        """Aplica a transformada (rotação por quaternion + translação)."""
        q = tf.transform.rotation
        t = tf.transform.translation
        px, py, pz = pose.position.x, pose.position.y, pose.position.z
        # v' = q * v * q^-1, escrito sem biblioteca para não depender de tf2_geometry_msgs.
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        vx = (1 - 2 * (yy * yy + zz * zz)) * px + 2 * (xx * yy - zz * ww) * py \
            + 2 * (xx * zz + yy * ww) * pz
        vy = 2 * (xx * yy + zz * ww) * px + (1 - 2 * (xx * xx + zz * zz)) * py \
            + 2 * (yy * zz - xx * ww) * pz
        return vx + t.x, vy + t.y

    def merge(self, alvo, x, y, stamp):
        """Junta a detecção ao objeto mais próximo, ou cria um novo."""
        if alvo is self.bananas and self.is_collected(x, y):
            return   # já recolhida: não ressuscita

        merge_radius = self.get_parameter('merge_radius').value
        perto, menor = None, merge_radius
        for mark in alvo:
            d = math.hypot(mark.x - x, mark.y - y)
            if d <= menor:
                perto, menor = mark, d
        if perto is None:
            if self.spin > self.get_parameter('max_spin_to_create').value:
                return   # girando rápido demais para confiar num ponto novo
            alvo.append(Landmark(x, y, stamp))
        else:
            perto.update(x, y, stamp, self.get_parameter('weight_cap').value)

    def is_collected(self, x, y):
        radius = self.get_parameter('collected_radius').value
        return any(math.hypot(cx - x, cy - y) <= radius for cx, cy in self.collected)

    # =========================================================================
    #  Saída
    # =========================================================================

    def confirmed(self, marks):
        """Só os objetos vistos vezes suficientes entram no mapa publicado."""
        min_hits = self.get_parameter('min_hits').value
        return [m for m in marks if m.hits >= min_hits]

    def publish_map(self):
        bananas = self.confirmed(self.bananas)
        poops = self.confirmed(self.poops)
        self.banana_pub.publish(self.to_pose_array(bananas))
        self.poop_pub.publish(self.to_pose_array(poops))

        counts = (len(bananas), len(poops))
        if counts != self.last_counts:
            self.last_counts = counts
            self.get_logger().info(
                f'mapa: {counts[0]} bananas e {counts[1]} poops '
                f'({len(self.collected)} bananas já recolhidas)'
            )

    def to_pose_array(self, marks):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        for mark in marks:
            pose = Pose()
            pose.position.x = mark.x
            pose.position.y = mark.y
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionMap()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
