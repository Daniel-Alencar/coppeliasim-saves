# tf_demo

Detecção de objetos com YOLO numa cena do CoppeliaSim, com as posições
publicadas como **TF** do ROS 2. Um Pioneer P3-DX parado, com uma Kinect, olha
para uma tigela (`/Bowl`) e um copo (`/Cup`). Os objetos detectados voltam para
a cena como *dummies* vermelhos.

```
CoppeliaSim ──(ZeroMQ)──▶ kinect.py ──(/rgb/image, /depth/image)──▶ yolo_3d_detection.py
     │                                                                  │
     ├──(ZeroMQ)──▶ tf_node.py ──(/tf: map → base_link → camera)        ├──▶ /yolo/annotated
     │                                                                  ├──▶ /tf: camera → object_<classe>
     │                                                                  └──▶ /yolo/object_3d_point
     │                                                                              │
     ◀──(ZeroMQ: cria e move dummies)── dummy_creation.py ◀─────────────────────────┘
```

Não é um pacote ROS: são quatro scripts Python soltos, em [scripts/](scripts),
todos falando com o simulador pela ZeroMQ Remote API (porta 23000). A cena não
usa o plugin `simROS2`.

| Script | Nó | Papel |
|---|---|---|
| `kinect.py` | `kinect_node` | Lê as câmeras RGB e de profundidade da Kinect e publica a 10 Hz. Dá o *play* na simulação |
| `tf_node.py` | `coppelia_tf_publisher` | Publica a 20 Hz a árvore TF `map → base_footprint`, `map → base_link` e `base_link → camera_color_optical_frame` |
| `yolo_3d_detection.py` | `yolo_3d_publisher` | Roda o YOLOv8 na imagem RGB, calcula a posição 3D de cada objeto e publica imagem anotada, marcador e TF |
| `dummy_creation.py` | `coppelia_marker_sync` | Converte os marcadores para o frame `map` e cria, move ou remove um *dummy* na cena para cada classe detectada |

---

## 1. Pré-requisitos

- ROS 2 Jazzy (`/opt/ros/jazzy`), com `tf2_ros`, `tf2_geometry_msgs`,
  `cv_bridge` e `rviz2`
- CoppeliaSim **com interface gráfica**: as câmeras da Kinect precisam de
  OpenGL para renderizar, e o modo headless (`-h`) derruba o simulador quando o
  `kinect.py` lê a imagem
- Pacotes Python no Python **do sistema**:
  `coppeliasim-zmqremoteapi-client`, `ultralytics`, `numpy` e `opencv`

### Os scripts rodam com o Python do sistema

Os scripts não são executados por `ros2 run`, então use sempre
`/usr/bin/python3`. Com o Anaconda na frente do `PATH`, o `python3` puro é o do
Anaconda, que não consegue importar o `rclpy`
(`GLIBCXX_3.4.30' not found`).

Confira o que já está disponível:

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -c "import rclpy, cv_bridge, tf2_geometry_msgs, cv2, coppeliasim_zmqremoteapi_client; print('ok')"
/usr/bin/python3 -c "import ultralytics; print('ok')"
```

### Instalando o YOLO

O Ubuntu 24.04 bloqueia `pip install` no Python do sistema (PEP 668), por isso
o `--break-system-packages`:

```bash
/usr/bin/python3 -m pip install --user --break-system-packages ultralytics "numpy<2"
```

O `"numpy<2"` evita que o `ultralytics` atualize o NumPy para a versão 2, que
quebra o `cv_bridge` do ROS Jazzy (compilado contra o NumPy 1). A instalação
baixa o PyTorch e ocupa alguns GB.

Se faltar o cliente da Remote API:

```bash
/usr/bin/python3 -m pip install --user --break-system-packages coppeliasim-zmqremoteapi-client
```

### A cena

Abra [tf_scene.ttt](tf_scene.ttt). Os scripts procuram estes objetos:

```
/Pioneer_p3dx
├── Ground_truth                      (dummy — referência do robô)
└── Pioneer_p3dx_connection4
    └── kinect
        ├── camera_ref                (dummy — referência da câmera)
        └── joint/body
            ├── rgb                   (vision sensor 640×480)
            └── depth                 (vision sensor 640×480)
/Bowl
/Cup
```

Os caminhos curtos usados nos scripts (`/Pioneer_p3dx/kinect/rgb`,
`./camera_ref` etc.) funcionam porque o CoppeliaSim encontra o objeto pelo nome
em qualquer nível da hierarquia.

---

## 2. Executar

Com o CoppeliaSim aberto e a `tf_scene.ttt` carregada, rode cada script em um
terminal, **a partir da pasta `scripts/`** e nesta ordem. Em todos os
terminais:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/ros/jazzy/setup.zsh
cd "projects/5 - tf_demo/scripts"
```

**Terminal 1 — a câmera:**

```bash
/usr/bin/python3 kinect.py
```

Ele dá o *play* na simulação. Espere aparecer
`Connected to CoppeliaSim successfully`.

**Terminal 2 — a árvore TF:**

```bash
/usr/bin/python3 tf_node.py
```

**Terminal 3 — a detecção:**

```bash
/usr/bin/python3 yolo_3d_detection.py
```

Na primeira execução o `ultralytics` baixa os pesos `yolov8n.pt` para a pasta
atual; por isso é bom rodar de dentro de `scripts/`.

**Terminal 4 — os dummies na cena:**

```bash
/usr/bin/python3 dummy_creation.py
```

Na cena devem aparecer *dummies* vermelhos com nomes como `class_41`, na
posição dos objetos detectados. Um *dummy* é removido quando a classe fica 1 s
sem ser detectada.

**Terminal 5 — visualização:**

```bash
ros2 run rviz2 rviz2
```

No RViz:

1. Em *Global Options*, troque o **Fixed Frame** para `map`.
2. *Add* → **TF**: mostra `map`, `base_link`, `camera_color_optical_frame` e
   um `object_<classe>` para cada detecção.
3. *Add* → *By topic* → `/yolo/annotated` → **Image**: a imagem com as caixas
   do YOLO.
4. *Add* → *By topic* → `/yolo/object_3d_point` → **Marker**: uma esfera
   vermelha em cada objeto.

---

## 3. Tópicos e frames

| Tópico | Tipo | Publicado por |
|---|---|---|
| `/rgb/image` | `sensor_msgs/Image` (`rgb8`, 640×480) | `kinect.py` |
| `/depth/image` | `sensor_msgs/Image` (`16UC1`, mm) | `kinect.py` |
| `/rgb/camera_info` | `sensor_msgs/CameraInfo` | `kinect.py` |
| `/yolo/annotated` | `sensor_msgs/Image` (`bgr8`) | `yolo_3d_detection.py` |
| `/yolo/object_3d_point` | `visualization_msgs/Marker` | `yolo_3d_detection.py` |
| `/tf` | `tf2_msgs/TFMessage` | `tf_node.py` e `yolo_3d_detection.py` |

Árvore TF:

```
map
├── base_footprint
└── base_link
    └── camera_color_optical_frame
        └── object_<classe>          (um por classe detectada)
```

Conferindo:

```bash
ros2 topic hz /rgb/image                 # ~10 Hz
ros2 topic echo /yolo/object_3d_point
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
ros2 run tf2_tools view_frames           # gera frames.pdf com a árvore TF
```

---

## 4. Como a posição 3D é calculada

Para cada caixa detectada, o `yolo_3d_detection.py` pega o pixel central,
remove a distorção da lente, lê a profundidade nesse pixel e projeta para 3D
com o modelo *pinhole* e os parâmetros da Kinect:

```
X = (u − cx) · Z / fx
Y = (v − cy) · Z / fy
Z = profundidade
```

A exceção são as duas classes da cena: para elas o script **não usa a
projeção**, e sim a posição real do objeto lida do simulador em relação à
câmera (`sim.getObjectPosition(objeto, camera_ref)`). Assim a posição fica
exata e a demonstração se concentra na árvore TF, não na precisão da
profundidade. Veja na seção 5 um problema na projeção usada pelas demais
classes.

---

## 5. Observações sobre o código

Pontos a conhecer antes de alterar ou apresentar o projeto:

| Onde | O que acontece |
|---|---|
| `yolo_3d_detection.py`, classes 41 e 45 | No COCO, a classe **41 é `cup`** e a **45 é `bowl`**, mas o script associa 41 à `/Bowl` e 45 à `/Cup`. Quando o YOLO detecta o copo, publica a posição da tigela, e vice-versa |
| `yolo_3d_detection.py`, projeção 3D | Chamado com `P=None`, o `cv2.undistortPoints` devolve coordenadas **normalizadas** (já divididas por `fx`/`fy` e sem `cx`/`cy`), e não em pixels. O script aplica `(u − cx) / fx` de novo sobre elas, então a posição das classes que usam a projeção sai errada. Passar `P=` com a mesma matriz da câmera faz a função devolver pixels e corrige a conta |
| `yolo_3d_detection.py` × `kinect.py` | O `cy` usado na projeção é `235.313989`, mas o `CameraInfo` publicado pela Kinect usa `255.313989` |
| `kinect.py` | As imagens saem com `frame_id = "kinect"`, que não existe na árvore TF. A imagem aparece no RViz, mas displays que dependem do frame (como *Camera*) reclamam |
| `dummy_creation.py` | Cria objetos de verdade na cena. Não salve a cena com a simulação rodando, ou os *dummies* ficam gravados nela |

---

## 6. Problemas comuns

| Sintoma | O que fazer |
|---|---|
| `GLIBCXX_3.4.30' not found` ao importar `rclpy` | O script rodou com o Python do Anaconda; use `/usr/bin/python3` |
| `ModuleNotFoundError: ultralytics` | Instale no Python do sistema (seção 1) |
| `numpy.core.multiarray failed to import` ou erro no `cv_bridge` | O NumPy foi atualizado para a versão 2; reinstale com `"numpy<2"` |
| `externally-managed-environment` no `pip` | Falta o `--break-system-packages` |
| `Failed connecting to CoppeliaSim` | Abra o simulador. Se já estiver aberto, algum script da cena pode estar travando a thread principal |
| O CoppeliaSim fecha sozinho (*signal 11*) ao rodar o `kinect.py` | O simulador está em modo headless; abra com interface gráfica |
| `/yolo/annotated` não publica nada | O `yolo_3d_detection.py` só processa uma imagem RGB depois de receber uma de profundidade; confira se o `kinect.py` está rodando |
| `TF lookup failed` no `dummy_creation.py` | O `tf_node.py` não está rodando, ou ainda não publicou a árvore; aparece só nos primeiros instantes |
| Nenhum *dummy* aparece na cena | Confira com `ros2 topic echo /yolo/object_3d_point` se o YOLO está detectando algo; a câmera precisa estar vendo os objetos |
