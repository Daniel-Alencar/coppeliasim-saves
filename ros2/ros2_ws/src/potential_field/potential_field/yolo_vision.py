"""Detecta bananas e poops com o YOLO e estima a posição 3D de cada um.

Entra imagem, sai detecção: este nó não conhece o CoppeliaSim nem o campo
potencial. Ele recebe a imagem colorida e a de profundidade da Kinect, roda o
YOLO, decide se cada caixa é banana ou poop e projeta o centro dela para o
referencial da câmera usando a profundidade.

    rgb/image    (Image)  --->  |             |  --->  detections/bananas (PoseArray)
    depth/image  (Image)  --->  | yolo_vision |  --->  detections/poops   (PoseArray)
    rgb/camera_info       --->  |             |  --->  yolo/annotated     (Image)

As duas PoseArray saem no referencial óptico da câmera (parâmetro
camera_frame). Quem as leva para o mundo é o perception_map, usando a TF que a
ponte publica no mesmo instante da imagem.

Quem detecta é o YOLO. Quem decide a classe:

1. A classe do COCO, quando é a nominal: 46 ('banana') ou 54 ('donut', que é
   como o YOLO enxerga o poop desta cena).
2. Senão, a cor dentro da caixa do YOLO: amarelo é banana, marrom é poop.
   Medido nesta cena, o YOLO também chama a banana de frisbee, bird, sports
   ball, kite... e o poop de cow, cake, dining table. Essas classes mudam de
   um modelo para outro, então em vez de listar todas a cor desempata.

Com o yolo11s, em dois mapas sorteados e 28 quadros, a regra acertou 140 de 140
detecções, sem nenhum falso positivo.

Projeção: com a profundidade d e os intrínsecos (fx, fy, cx, cy) do CameraInfo,

    X = (u - cx) * d / fx        Y = (v - cy) * d / fy        Z = d

no referencial óptico do ROS: z para a frente, x para a direita, y para baixo.
"""

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
import message_filters
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class YoloVision(Node):
    """Roda o YOLO na imagem da Kinect e devolve pontos 3D por classe."""

    def __init__(self):
        super().__init__('yolo_vision')

        # --- Modelo ----------------------------------------------------------
        # Comparados nesta cena (yolov8n, yolo11n-seg, yolo11s, yolo11m), o
        # yolo11s achou mais objetos distintos sem nenhum falso positivo, a
        # ~0,2 s por quadro em CPU. Se o arquivo não existir, o ultralytics
        # baixa na primeira execução.
        self.declare_parameter('model', 'yolo11s.pt')
        # Os objetos ocupam poucos pixels a 320x240: confiança baixa é o que
        # garante cobertura, e mesmo a 0,10 não houve falso positivo.
        self.declare_parameter('confidence', 0.10)
        # O ultralytics amplia a imagem para este tamanho antes da inferência.
        self.declare_parameter('imgsz', 640)
        # Threads do PyTorch. Sem limite ele ocupa todos os núcleos, e o
        # CoppeliaSim e a ponte ficam sem CPU: medido, o laço de controle caiu
        # de 14 Hz para 5 Hz com o YOLO rodando.
        self.declare_parameter('torch_threads', 4)

        # --- Classe ----------------------------------------------------------
        self.declare_parameter('banana_classes', [46])   # 'banana' no COCO
        self.declare_parameter('poop_classes', [54])     # 'donut' no COCO
        # Para as demais classes, decide pela cor dentro da caixa.
        self.declare_parameter('classify_by_color', True)
        # Faixas medidas nesta cena contra as posições verdadeiras, em HSV do
        # OpenCV (H de 0 a 179): banana amarela e saturada; poop marrom e mais
        # escuro que o piso, que fica logo acima do limite de V.
        self.declare_parameter('banana_hsv_low', [20, 120, 100])
        self.declare_parameter('banana_hsv_high', [40, 255, 255])
        self.declare_parameter('poop_hsv_low', [5, 60, 40])
        self.declare_parameter('poop_hsv_high', [18, 200, 100])
        # Detecção sem cor reconhecível e de classe desconhecida: obstáculo.
        # Nesta cena não aconteceu, mas desviar de algo é o lado seguro.
        self.declare_parameter('unknown_as_poop', True)

        # Alternativa fora do pipeline do enunciado: manchas de cor também
        # viram detecções, sem passar pelo YOLO. Desligado por padrão.
        self.declare_parameter('color_proposals', False)
        self.declare_parameter('min_blob_area', 12)      # px

        # --- Câmera ----------------------------------------------------------
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('rgb_topic', 'rgb/image')
        self.declare_parameter('depth_topic', 'depth/image')
        self.declare_parameter('camera_info_topic', 'rgb/camera_info')
        # Longe, o erro de ângulo vira erro grande de posição, e a profundidade
        # satura perto do limite do sensor (3,5 m nesta cena).
        self.declare_parameter('max_range', 2.5)      # [m]
        self.declare_parameter('min_range', 0.05)     # [m]
        # Profundidade do objeto: percentil dos pixels da caixa. A caixa também
        # pega o piso atrás dele, que está mais longe; um percentil baixo fica
        # com o objeto. Medido: erro mediano de 3 cm nas bananas e 9 cm nos
        # poops, contra 8 cm e 12 cm usando só o pixel central.
        self.declare_parameter('depth_percentile', 20.0)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('log_classes', True)

        self.camera_frame = self.get_parameter('camera_frame').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.banana_classes = set(self.get_parameter('banana_classes').value)
        self.poop_classes = set(self.get_parameter('poop_classes').value)
        self.unknown_as_poop = self.get_parameter('unknown_as_poop').value
        self.classify_by_color = self.get_parameter('classify_by_color').value
        self.color_proposals = self.get_parameter('color_proposals').value
        self.faixas = {
            nome: np.array(self.get_parameter(nome).value, dtype=np.uint8)
            for nome in ('banana_hsv_low', 'banana_hsv_high',
                         'poop_hsv_low', 'poop_hsv_high')
        }

        # --- Modelo carregado uma vez ----------------------------------------
        import torch                   # importados aqui: demoram e só servem aqui
        from ultralytics import YOLO
        torch.set_num_threads(int(self.get_parameter('torch_threads').value))
        cv2.setNumThreads(1)
        self.model = YOLO(self.get_parameter('model').value)
        self.names = getattr(self.model, 'names', {})
        self.seen_classes = set()

        self.bridge = CvBridge()
        self.info = None   # intrínsecos vindos do CameraInfo

        # --- Comunicação ROS -------------------------------------------------
        # A cor e a profundidade chegam em mensagens separadas, com o mesmo
        # carimbo de tempo. O sincronizador só entrega os dois juntos: sem ele,
        # a cor de agora seria combinada com a profundidade do quadro anterior,
        # o que, com o robô girando, desloca o objeto.
        rgb_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('rgb_topic').value)
        depth_sub = message_filters.Subscriber(
            self, Image, self.get_parameter('depth_topic').value)
        self.sync = message_filters.TimeSynchronizer([rgb_sub, depth_sub], 2)
        self.sync.registerCallback(self.images_callback)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.info_callback, 1)

        self.banana_pub = self.create_publisher(PoseArray, 'detections/bananas', 10)
        self.poop_pub = self.create_publisher(PoseArray, 'detections/poops', 10)
        self.annotated_pub = self.create_publisher(Image, 'yolo/annotated', 1)

        self.get_logger().info(
            f"modelo {self.get_parameter('model').value} carregado, esperando imagens"
        )

    # =========================================================================
    #  Entradas
    # =========================================================================

    def info_callback(self, msg: CameraInfo):
        """Intrínsecos: fx, fy, cx, cy da matriz k."""
        self.info = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def images_callback(self, rgb_msg: Image, depth_msg: Image):
        """Um ciclo de detecção, com a cor e a profundidade do mesmo instante."""
        if self.info is None:
            return   # sem intrínsecos não dá para projetar

        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = np.asarray(
            self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'),
            dtype=np.float32)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        anotada = frame.copy()
        bananas, poops = [], []

        # --- Detecção pelo YOLO -----------------------------------------------
        results = self.model(
            frame, verbose=False,
            conf=self.get_parameter('confidence').value,
            imgsz=self.get_parameter('imgsz').value)
        boxes = results[0].boxes
        if boxes is not None:
            for xyxy, cls, conf in zip(boxes.xyxy, boxes.cls, boxes.conf):
                caixa = [int(v) for v in xyxy]
                cls_id = int(cls)
                classe, via = self.classify(cls_id, hsv, caixa)
                self.log_class(cls_id, float(conf), classe, via)
                ponto = self.locate(caixa, depth, frame.shape)
                if classe is None or ponto is None:
                    continue
                (bananas if classe == 'banana' else poops).append(ponto)
                self.draw(anotada, caixa, classe,
                          f'{self.names.get(cls_id, cls_id)} {float(conf):.2f}')

        # --- Alternativa: manchas de cor, sem YOLO ----------------------------
        if self.color_proposals:
            for classe, caixa in self.color_blobs(hsv):
                ponto = self.locate(caixa, depth, frame.shape)
                if ponto is not None:
                    (bananas if classe == 'banana' else poops).append(ponto)
                    self.draw(anotada, caixa, classe, 'cor')

        stamp = rgb_msg.header.stamp
        self.banana_pub.publish(self.to_pose_array(bananas, stamp))
        self.poop_pub.publish(self.to_pose_array(poops, stamp))

        if self.get_parameter('publish_annotated').value:
            out = self.bridge.cv2_to_imgmsg(anotada, encoding='bgr8')
            out.header = rgb_msg.header
            self.annotated_pub.publish(out)

    # =========================================================================
    #  Classe
    # =========================================================================

    def classify(self, cls_id, hsv, caixa):
        """(classe, como decidiu): primeiro a classe nominal do COCO, depois a cor."""
        if cls_id in self.banana_classes:
            return 'banana', 'classe'
        if cls_id in self.poop_classes:
            return 'poop', 'classe'
        if self.classify_by_color:
            x1, y1, x2, y2 = caixa
            roi = hsv[max(0, y1):max(y1 + 1, y2), max(0, x1):max(x1 + 1, x2)]
            if roi.size:
                amarelo = int(np.count_nonzero(cv2.inRange(
                    roi, self.faixas['banana_hsv_low'], self.faixas['banana_hsv_high'])))
                marrom = int(np.count_nonzero(cv2.inRange(
                    roi, self.faixas['poop_hsv_low'], self.faixas['poop_hsv_high'])))
                if amarelo or marrom:
                    return ('banana' if amarelo >= marrom else 'poop'), 'cor'
        return ('poop' if self.unknown_as_poop else None), 'desconhecida'

    def log_class(self, cls_id, conf, classe, via):
        """Mostra uma vez cada classe do YOLO, e o que ela virou."""
        if not self.get_parameter('log_classes').value or cls_id in self.seen_classes:
            return
        self.seen_classes.add(cls_id)
        self.get_logger().info(
            f'YOLO classe {cls_id} ({self.names.get(cls_id, "?")}), confiança '
            f'{conf:.2f} -> {classe} (pela {via})'
        )

    def color_blobs(self, hsv):
        """Manchas de cor contíguas: (classe, caixa) para cada uma."""
        achados = []
        area_min = self.get_parameter('min_blob_area').value
        for classe in ('banana', 'poop'):
            mask = cv2.inRange(hsv, self.faixas[f'{classe}_hsv_low'],
                               self.faixas[f'{classe}_hsv_high'])
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            n, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
            for i in range(1, n):
                if stats[i, cv2.CC_STAT_AREA] < area_min:
                    continue
                x, y = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
                achados.append((classe, [x, y, x + stats[i, cv2.CC_STAT_WIDTH],
                                         y + stats[i, cv2.CC_STAT_HEIGHT]]))
        return achados

    # =========================================================================
    #  Posição
    # =========================================================================

    def locate(self, caixa, depth, shape):
        """(x, y, z) do centro da caixa no referencial óptico da câmera, ou None."""
        h, w = shape[:2]
        dh, dw = depth.shape[:2]
        # Se a profundidade tiver outro tamanho que a colorida, ajusta a escala.
        sx, sy = dw / w, dh / h
        x1, y1, x2, y2 = caixa
        roi = depth[max(0, int(y1 * sy)):max(1, int(y2 * sy)),
                    max(0, int(x1 * sx)):max(1, int(x2 * sx))]
        roi = roi[np.isfinite(roi) & (roi > self.min_range)]
        if roi.size == 0:
            return None
        d = float(np.percentile(roi, self.get_parameter('depth_percentile').value))
        if not (self.min_range <= d <= self.max_range):
            return None

        fx, fy, cx, cy = self.info
        if fx == 0.0 or fy == 0.0:
            return None
        u = (x1 + x2) / 2.0 * sx
        v = (y1 + y2) / 2.0 * sy
        return ((u - cx * sx) * d / (fx * sx), (v - cy * sy) * d / (fy * sy), d)

    # =========================================================================
    #  Saída
    # =========================================================================

    @staticmethod
    def draw(img, caixa, classe, texto):
        cor = (0, 220, 255) if classe == 'banana' else (40, 60, 140)
        cv2.rectangle(img, (caixa[0], caixa[1]), (caixa[2], caixa[3]), cor, 1)
        cv2.putText(img, f'{classe}: {texto}', (caixa[0], max(8, caixa[1] - 2)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, cor, 1)

    def to_pose_array(self, points, stamp):
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame
        for x, y, z in points:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = float(z)
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = YoloVision()
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
