"""Detecta bananas e poops na imagem da Kinect e estima a posição 3D de cada um.

Entra imagem, sai detecção: este nó não conhece o CoppeliaSim nem o campo
potencial. Ele assina as imagens que a ponte publica, roda o YOLO na imagem
colorida, pega a profundidade no centro de cada caixa e projeta o pixel para o
referencial da câmera.

    rgb/image    (Image)  --->  |             |  --->  detections/bananas (PoseArray)
    depth/image  (Image)  --->  | yolo_vision |  --->  detections/poops   (PoseArray)
    rgb/camera_info       --->  |             |  --->  yolo/annotated     (Image)

As duas PoseArray saem no referencial óptico da câmera (parâmetro
camera_frame). Quem as leva para o mundo é o perception_map, usando a TF que a
ponte publica.

Projeção: com a profundidade d do pixel (u, v) e os intrínsecos (fx, fy, cx, cy)
do CameraInfo,

    X = (u - cx) * d / fx        Y = (v - cy) * d / fy        Z = d

no referencial óptico do ROS: z para a frente, x para a direita, y para baixo.
"""


import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class YoloVision(Node):
    """Roda o YOLO na imagem da câmera e devolve pontos 3D por classe."""

    def __init__(self):
        super().__init__('yolo_vision')

        # --- Modelo ----------------------------------------------------------
        # Qualquer modelo do ultralytics serve. O "n" é o menor, que roda em CPU.
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('confidence', 0.25)   # confiança mínima
        # Classes do COCO: 46 é 'banana'. Medido nesta cena, o YOLO treinado no
        # COCO não reconhece nem a banana nem o poop do CoppeliaSim: a 320x240
        # ele responde frisbee, kite, donut, sports ball, cow. Por isso a classe
        # do COCO entra só como atalho, e quem decide de fato é a cor.
        self.declare_parameter('banana_classes', [46])
        self.declare_parameter('poop_classes', [-1])
        # Sem cor nem classe conhecida: tratar a detecção como obstáculo.
        self.declare_parameter('unknown_as_poop', True)

        # --- Cor --------------------------------------------------------------
        # A classificação que funciona nesta cena: banana é amarela, poop é
        # marrom. Faixas em HSV do OpenCV (H de 0 a 179).
        self.declare_parameter('classify_by_color', True)
        # Faixas medidas nesta cena, comparando as manchas encontradas com as
        # posições verdadeiras: a banana é amarela e saturada; o poop é marrom
        # e mais escuro que o piso, que fica logo acima do limite de V.
        self.declare_parameter('banana_hsv_low', [20, 120, 100])
        self.declare_parameter('banana_hsv_high', [40, 255, 255])
        self.declare_parameter('poop_hsv_low', [5, 60, 40])
        self.declare_parameter('poop_hsv_high', [18, 200, 100])
        # O YOLO sozinho perde quase tudo nesta resolução, então a cor também
        # propõe objetos: cada mancha contígua da cor vira uma detecção.
        self.declare_parameter('color_proposals', True)
        self.declare_parameter('min_blob_area', 12)      # px
        self.declare_parameter('use_yolo', True)

        # --- Câmera ----------------------------------------------------------
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('rgb_topic', 'rgb/image')
        self.declare_parameter('depth_topic', 'depth/image')
        self.declare_parameter('camera_info_topic', 'rgb/camera_info')
        # Longe, o erro de ângulo vira erro grande de posição, e a
        # profundidade satura perto do limite do sensor (3,5 m nesta cena).
        self.declare_parameter('max_range', 2.5)      # ignora detecção além disso [m]
        self.declare_parameter('min_range', 0.05)     # e aquém disso [m]
        # Quantos pixels em volta do centro entram na mediana da profundidade.
        self.declare_parameter('depth_patch', 3)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('log_classes', True)   # ajuda a descobrir os ids

        self.camera_frame = self.get_parameter('camera_frame').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.banana_classes = set(self.get_parameter('banana_classes').value)
        self.poop_classes = set(self.get_parameter('poop_classes').value)
        self.unknown_as_poop = self.get_parameter('unknown_as_poop').value
        self.classify_by_color = self.get_parameter('classify_by_color').value
        self.color_proposals = self.get_parameter('color_proposals').value

        # --- Modelo carregado uma vez ----------------------------------------
        from ultralytics import YOLO   # importado aqui: demora e só é preciso aqui
        self.model = YOLO(self.get_parameter('model').value)
        self.names = getattr(self.model, 'names', {})
        self.seen_classes = set()

        self.bridge = CvBridge()
        self.depth = None          # última imagem de profundidade, em metros
        self.info = None           # intrínsecos vindos do CameraInfo

        # --- Comunicação ROS -------------------------------------------------
        self.create_subscription(
            Image, self.get_parameter('rgb_topic').value, self.rgb_callback, 1)
        self.create_subscription(
            Image, self.get_parameter('depth_topic').value, self.depth_callback, 1)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self.info_callback, 1)

        self.banana_pub = self.create_publisher(PoseArray, 'detections/bananas', 10)
        self.poop_pub = self.create_publisher(PoseArray, 'detections/poops', 10)
        self.annotated_pub = self.create_publisher(Image, 'yolo/annotated', 1)

        self.get_logger().info(
            f"modelo {self.get_parameter('model').value} carregado, "
            'esperando imagens'
        )

    # =========================================================================
    #  Entradas
    # =========================================================================

    def info_callback(self, msg: CameraInfo):
        """Intrínsecos: fx, fy, cx, cy da matriz k."""
        self.info = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def depth_callback(self, msg: Image):
        """Profundidade em metros, no mesmo tamanho da imagem colorida."""
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth = np.asarray(depth, dtype=np.float32)

    def depth_at(self, u, v):
        """Mediana da profundidade num quadrado em volta do pixel.

        A mediana evita o valor solto de uma borda, onde o pixel cai no fundo
        em vez de cair no objeto.
        """
        if self.depth is None:
            return None
        h, w = self.depth.shape[:2]
        r = int(self.get_parameter('depth_patch').value)
        u0, u1 = max(0, u - r), min(w, u + r + 1)
        v0, v1 = max(0, v - r), min(h, v + r + 1)
        patch = self.depth[v0:v1, u0:u1]
        patch = patch[np.isfinite(patch) & (patch > 0.0)]
        if patch.size == 0:
            return None
        return float(np.median(patch))

    # =========================================================================
    #  Detecção
    # =========================================================================

    def rgb_callback(self, msg: Image):
        """Um ciclo de detecção, disparado por cada imagem colorida."""
        if self.depth is None or self.info is None:
            return   # sem profundidade ou sem intrínsecos não dá para projetar

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        bananas, poops = [], []
        anotada = frame

        # --- Propostas do YOLO ------------------------------------------------
        if self.get_parameter('use_yolo').value:
            results = self.model(
                frame, verbose=False, conf=self.get_parameter('confidence').value)
            boxes = results[0].boxes
            anotada = results[0].plot()
            if boxes is not None:
                for xyxy, cls, conf in zip(boxes.xyxy, boxes.cls, boxes.conf):
                    caixa = [int(v) for v in xyxy]
                    classe = self.classify(int(cls), hsv, caixa)
                    self.log_class(int(cls), float(conf), classe)
                    self.append(classe, self.locate_box(caixa, frame.shape),
                                bananas, poops)

        # --- Propostas por cor ------------------------------------------------
        # O objeto é pequeno na imagem e o YOLO do COCO não conhece nenhuma das
        # duas coisas; a mancha de cor é a detecção que de fato sustenta o mapa.
        if self.color_proposals:
            for classe, caixa in self.color_blobs(hsv):
                self.append(classe, self.locate_box(caixa, frame.shape),
                            bananas, poops)
                if self.get_parameter('publish_annotated').value:
                    cor = (0, 255, 255) if classe == 'banana' else (60, 60, 160)
                    cv2.rectangle(anotada, (caixa[0], caixa[1]),
                                  (caixa[2], caixa[3]), cor, 1)

        stamp = msg.header.stamp
        self.banana_pub.publish(self.to_pose_array(bananas, stamp))
        self.poop_pub.publish(self.to_pose_array(poops, stamp))

        if self.get_parameter('publish_annotated').value:
            out = self.bridge.cv2_to_imgmsg(anotada, encoding='bgr8')
            out.header = msg.header
            self.annotated_pub.publish(out)

    @staticmethod
    def append(classe, ponto, bananas, poops):
        if ponto is None or classe is None:
            return
        (bananas if classe == 'banana' else poops).append(ponto)

    def log_class(self, cls_id, conf, classe):
        """Mostra uma vez cada classe que o YOLO devolve, para ajudar a ajustar."""
        if not self.get_parameter('log_classes').value or cls_id in self.seen_classes:
            return
        self.seen_classes.add(cls_id)
        self.get_logger().info(
            f'YOLO classe {cls_id} ({self.names.get(cls_id, "?")}), '
            f'confiança {conf:.2f} -> tratada como {classe}'
        )

    def classify(self, cls_id, hsv, caixa):
        """Decide entre banana e poop: primeiro a classe do COCO, depois a cor."""
        if cls_id in self.banana_classes:
            return 'banana'
        if cls_id in self.poop_classes:
            return 'poop'
        if self.classify_by_color:
            x1, y1, x2, y2 = caixa
            roi = hsv[max(0, y1):max(y1 + 1, y2), max(0, x1):max(x1 + 1, x2)]
            if roi.size:
                amarelo = cv2.inRange(roi, self.faixa('banana_hsv_low'),
                                      self.faixa('banana_hsv_high')).sum()
                marrom = cv2.inRange(roi, self.faixa('poop_hsv_low'),
                                     self.faixa('poop_hsv_high')).sum()
                if amarelo > marrom and amarelo > 0:
                    return 'banana'
                if marrom > 0:
                    return 'poop'
        return 'poop' if self.unknown_as_poop else None

    def faixa(self, nome):
        import numpy as _np
        return _np.array(self.get_parameter(nome).value, dtype=_np.uint8)

    def color_blobs(self, hsv):
        """Manchas de cor contíguas: (classe, caixa) para cada uma."""
        achados = []
        area_min = self.get_parameter('min_blob_area').value
        for classe, baixo, alto in (
            ('banana', 'banana_hsv_low', 'banana_hsv_high'),
            ('poop', 'poop_hsv_low', 'poop_hsv_high'),
        ):
            mask = cv2.inRange(hsv, self.faixa(baixo), self.faixa(alto))
            # Fecha buracos pequenos para a mancha não virar vários pedaços.
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
            n, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
            for i in range(1, n):
                if stats[i, cv2.CC_STAT_AREA] < area_min:
                    continue
                x, y = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP]
                achados.append((classe, [x, y, x + stats[i, cv2.CC_STAT_WIDTH],
                                         y + stats[i, cv2.CC_STAT_HEIGHT]]))
        return achados

    def locate_box(self, caixa, shape):
        """(x, y, z) do objeto no referencial óptico da câmera, ou None."""
        x1, y1, x2, y2 = (float(v) for v in caixa)
        u = int(round((x1 + x2) / 2.0))
        # O centro vertical da caixa costuma cair no objeto; para algo no chão
        # a parte de baixo é mais confiável, mas também pega o piso. Fica no meio.
        v = int(round((y1 + y2) / 2.0))
        h, w = shape[:2]
        u = min(max(u, 0), w - 1)
        v = min(max(v, 0), h - 1)

        d = self.depth_at(u, v)
        if d is None or not (self.min_range <= d <= self.max_range):
            return None

        fx, fy, cx, cy = self.info
        if fx == 0.0 or fy == 0.0:
            return None
        # Se a profundidade tiver outro tamanho que a colorida, ajusta a escala.
        dh, dw = self.depth.shape[:2]
        if (dw, dh) != (w, h):
            fx *= dw / w
            fy *= dh / h
            cx *= dw / w
            cy *= dh / h
            u = int(u * dw / w)
            v = int(v * dh / h)
        return ((u - cx) * d / fx, (v - cy) * d / fy, d)

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
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
