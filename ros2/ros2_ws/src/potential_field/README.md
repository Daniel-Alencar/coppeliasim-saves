# potential_field

Navegação por **campos potenciais** com mapa construído pela **câmera**: o robô
detecta bananas e poops com o YOLO na imagem da Kinect, estima a posição 3D de
cada um pela profundidade, mantém um mapa persistente e anda até as bananas
desviando dos poops. É o trabalho descrito em [requirements.md](requirements.md).

```
Kinect RGB-D → YOLO → posição 3D → mapa persistente → campo potencial → rodas
```

As posições de bananas e poops **não** vêm dos sinais da cena nem de
coordenadas fixas: o mapa começa vazio e cresce com o que a câmera vê. A
trajetória também não é traçada: ela sai da soma das forças.

```
F = F_atrativa(banana alvo) + Σ F_repulsiva(poop)
```

```
              rgb, depth, pose, /tf                 detections/*
CoppeliaSim ──ZeroMQ──▶ coppelia_bridge ──────▶ yolo_vision ──────▶ perception_map
     ▲                       ▲                                           │ bananas, poops
     │ motores               │ cmd_vel                                   ▼
     └───────────────────────┴─────────────────────────────────────── navigator
```

| Executável | Papel |
|---|---|
| `coppelia_bridge` | Só comunicação: publica imagem, profundidade, pose e TF da cena; aplica o `cmd_vel` nas rodas |
| `yolo_vision` | Roda o YOLO, decide banana ou poop e projeta cada detecção em 3D no referencial da câmera |
| `perception_map` | Leva as detecções para o mundo, funde as repetidas e mantém as listas de bananas e poops |
| `navigator` | Campo potencial: escolhe o alvo, soma as forças, publica `cmd_vel`; procura quando não conhece banana |
| `fake_world` | Substitui a ponte para testar o navegador sem o simulador |

---

## 1. Pré-requisitos

- ROS 2 Jazzy (`/opt/ros/jazzy`), com `cv_bridge`, `tf2_ros` e `message_filters`
- CoppeliaSim aberto com a cena
  [coppeliasim/robot/pega_banana_potential_field.ttt](coppeliasim/robot/pega_banana_potential_field.ttt),
  **com interface gráfica**: a Kinect precisa de OpenGL para renderizar
- No Python do sistema (o `ros2 run` não usa o `.venv/`):

```bash
/usr/bin/python3 -m pip install --user --break-system-packages \
  coppeliasim-zmqremoteapi-client ultralytics "numpy<2"
```

O `"numpy<2"` evita que o `ultralytics` atualize o NumPy para a versão 2, que
quebra o `cv_bridge` do ROS Jazzy. O modelo `yolo11s.pt` é baixado pelo
`ultralytics` na primeira execução, na pasta de onde o launch foi chamado.

### Os scripts da cena

Exportados em [coppeliasim/scripts/](coppeliasim/scripts) — idênticos aos que
estão na cena. Nenhum deles atrapalha o controle:

| Script | Faz | Interfere? |
|---|---|---|
| `/buildScene` | Sorteia e cria 20 bananas e 20 poops no *play*; publica os sinais `banana`/`poop` | Não. O modo câmera não lê os sinais |
| `/dirt_script` | Quando o sensor `poopximity` (sob o robô) toca um objeto, o manda para z = 1000 e conta pontos | Não. É o placar: banana recolhida soma ponto, poop atropelado é penalidade |
| `/myRobot/battery` | Publica o nível de bateria num sinal; o desgaste está em 0 | Não |
| `/myRobot/odometry` e `encoder` | Calculam odometria e gravam numa propriedade do robô | Não; só leem as juntas |
| `/myRobot/controler`, `/myRobot/python_controler` | **Escrevem nos motores** a cada passo | Estão **desabilitados** na cena. Se forem habilitados, anulam o `cmd_vel`; a ponte avisa no terminal |

---

## 2. Compilar

```bash
cd ros2/ros2_ws
colcon build --packages-select potential_field --symlink-install
source install/setup.zsh     # no zsh; use setup.bash se a sua shell for bash
ros2 pkg executables potential_field
```

---

## 3. Executar

Com o CoppeliaSim aberto e a cena carregada. A ponte dá o *play* sozinha se a
simulação estiver parada, e a para ao sair se foi ela quem iniciou.

**Terminal 1 — tudo:**

```bash
ros2 launch potential_field perception_field.launch.py
```

**Terminal 2 — o que o YOLO está vendo:**

```bash
ros2 run rqt_image_view rqt_image_view /myRobot/yolo/annotated
```

A caixa amarela é banana, a marrom é poop, e o texto mostra a classe que o
YOLO deu e a confiança. No terminal 1 aparecem o mapa crescendo e as coletas:

```
[navigator]: nenhuma banana conhecida: procurando
[perception_map]: mapa: 4 bananas e 5 poops (0 bananas já recolhidas)
[navigator]: banana 1/4 alcançada
[perception_map]: banana recolhida em (0.59, 0.01); 1 no total
```

### Fim da corrida

Quando a cena recolhe **20 bananas** (o total que o `/buildScene` cria), a ponte
para a simulação e o launch inteiro encerra sozinho:

```
[coppelia_bridge]: placar da cena: 20/20 bananas
[coppelia_bridge]: 20 bananas recolhidas: meta atingida, parando a simulação
```

O critério é o **placar da cena**, e não a contagem do navegador. O robô às
vezes passa por cima de uma banana que não era o alvo, e o navegador não fica
sabendo — nos testes ele contou 16 quando a cena já tinha 19. O `/dirt_script`
soma o ponto e manda a banana para z = 1000 no mesmo instante, então a ponte
conta as bananas lá em cima. Isso só decide quando terminar: a navegação
continua sem ver posição nenhuma da cena.

### Argumentos do launch

| Argumento | Padrão | O que faz |
|---|---|---|
| `model` | `yolo11s.pt` | Modelo do ultralytics (nome ou caminho do `.pt`) |
| `navigator` | `1` | `0` só percebe e mapeia, sem mover o robô |
| `stop_after_bananas` | `20` | Para a simulação e encerra tudo ao recolher este número; `0` desliga |
| `unknown_as_poop` | `true` | Detecção sem classe nem cor reconhecível vira obstáculo |
| `wheel_radius` | `0.05` | Raio da roda (m) |
| `wheel_separation` | `0.0` | Distância entre rodas (m); `0.0` mede na cena (0,2 m) |
| `robot` / `port` | `/myRobot` / `23000` | Robô e porta da Remote API |

### Outros modos

```bash
# Gabarito: mapa lido dos sinais da cena (o exercício anterior)
ros2 launch potential_field potential_field.launch.py

# Sem o simulador: mundo de mentira para testar o navegador
ros2 launch potential_field potential_field.launch.py fake:=1
```

---

## 4. Tópicos

Todos no namespace `myRobot`, aplicado pelo launch.

| Tópico | Tipo | Quem publica | Conteúdo |
|---|---|---|---|
| `rgb/image` | `sensor_msgs/Image` (`rgb8`) | ponte | Imagem da Kinect, 320×240, 5 Hz |
| `depth/image` | `sensor_msgs/Image` (`32FC1`) | ponte | Profundidade em metros, alinhada com a colorida |
| `rgb/camera_info` | `sensor_msgs/CameraInfo` | ponte | Intrínsecos tirados do ângulo de visão da cena |
| `/tf` | `tf2_msgs/TFMessage` | ponte | `world → base_link → camera_color_optical_frame` |
| `pose` | `geometry_msgs/PoseStamped` | ponte | Pose do robô no mundo |
| `detections/bananas`, `detections/poops` | `geometry_msgs/PoseArray` | `yolo_vision` | Detecções do quadro, no referencial da câmera |
| `yolo/annotated` | `sensor_msgs/Image` | `yolo_vision` | Imagem com as caixas e a classe decidida |
| `bananas`, `poops` | `geometry_msgs/PoseArray` | `perception_map` | Mapa persistente, no mundo (transient local) |
| `cmd_vel` | `geometry_msgs/Twist` | navegador | `linear.x` (m/s), `angular.z` (rad/s) |
| `target`, `force`, `collected` | `PointStamped`, `Vector3Stamped`, `Int32` | navegador | Alvo atual, resultante do campo, bananas coletadas |

---

## 5. Como funciona

### Detecção e classe (`yolo_vision`)

O **YOLO detecta** os objetos. A classe é decidida assim:

1. Se o YOLO disser `banana` (46) ou `donut` (54), vale a classe dele.
2. Senão, vale a **cor dentro da caixa**: amarelo é banana, marrom é poop.

O segundo passo existe porque o YOLO treinado no COCO, nesta cena e a 320×240,
também chama a banana de `frisbee`, `bird`, `sports ball`, `kite`… e o poop de
`cow`, `cake`, `dining table`. Essas classes mudam de um modelo para outro, então
listar todas seria frágil; a cor desempata sem depender do modelo.

### Posição 3D

Para cada caixa, a profundidade é o **percentil 20** dos pixels dela (a caixa
também pega o piso atrás do objeto, que está mais longe). O centro da caixa é
projetado pelo modelo *pinhole* do ROS:

```
X = (u − cx)·d/fx      Y = (v − cy)·d/fy      Z = d
```

Duas coisas tiveram de ser acertadas na ponte para isso valer:

- **O referencial do sensor do CoppeliaSim é o óptico do ROS girado 180° em
  torno do eixo óptico** (o +x do sensor aponta para a esquerda da imagem). A
  ponte publica a TF já com esse giro.
- **A TF sai junto com a imagem, com o mesmo carimbo de tempo**, e o
  `yolo_vision` só junta cor e profundidade de carimbos iguais. Com o robô
  girando, combinar instantes diferentes jogava a detecção para o lado e
  enchia o mapa de cópias do mesmo objeto.

### Mapa persistente (`perception_map`)

- Cada detecção vai para o mundo pela TF do instante da imagem.
- Detecções a menos de `merge_radius` de um objeto já conhecido são o mesmo
  objeto: atualizam a média da posição em vez de criar outro.
- Um objeto só entra no mapa depois de visto `min_hits` vezes.
- Girando acima de `max_spin_to_create`, as detecções ainda refinam os
  objetos conhecidos, mas não criam novos.
- Quando o robô passa por cima de uma banana, ela sai do mapa e entra numa
  lista de recolhidas, para não voltar se for vista de novo.

### Campo potencial (`navigator`)

1. **Alvo:** a banana pendente mais próxima. Só ela atrai.
2. **Atração** para o alvo, saturada em `k_att`.
3. **Repulsão** de cada poop a menos de `d0_poop`: `k_rep · (1/d − 1/d0) / d²`.
4. **Resultante** saturada em `f_max`; a direção dela é o rumo desejado.
5. **Comando:** o erro de rumo vira `angular.z`; o avanço é `v_max · cos(erro)`.
   A ponte converte em velocidade de cada roda.
6. **Coleta:** a menos de `collect_radius` do alvo, a banana conta como
   alcançada e ele escolhe a próxima.
7. **Busca:** sem banana conhecida, gira uma volta inteira devagar (a câmera vê
   o entorno), anda um pouco desviando dos poops conhecidos e gira de novo. Só
   para depois de `search_timeout` sem achar nada.

Mínimos locais (repulsão anulando a atração) são resolvidos com uma componente
tangencial à repulsão, ligada quando a distância ao alvo para de cair.

---

## 6. Resultados medidos

Avaliação feita contra as posições verdadeiras da cena, lidas **só pelo
roteiro de teste**, nunca pelos nós.

**Modelos**, em 14 quadros de uma volta completa do robô:

| Modelo | Bananas distintas | Poops distintos | Falsos positivos | Tempo/quadro (CPU) |
|---|---|---|---|---|
| `yolov8n` | 15 | 6 | 0 | ~0,1 s |
| `yolo11n-seg` | 15 | 9 | 0 | ~0,1 s |
| **`yolo11s`** | **16** | **10** | **0** | ~0,2 s |
| `yolo11m` | 16 | 6 | 1 | ~0,3 s |

**Classe** (regra classe nominal + cor), com o `yolo11s` em dois mapas
sorteados: **140 de 140 detecções corretas**.

**Posição:** erro mediano de 3 cm nas bananas e 9 cm nos poops, num quadro
parado.

**Execução completa**, 4 minutos:

| Bananas recolhidas | Poops atropelados | Mapa de bananas | Mapa de poops |
|---|---|---|---|
| 8 de 20 | 0 | erro 10 cm, 1 duplicata | 16 poops, erro 8 cm, 1 duplicata |

---

## 7. Parâmetros

### `yolo_vision`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `model` | `yolo11s.pt` | Modelo do ultralytics |
| `confidence` | `0.10` | Confiança mínima (objetos pequenos pedem valor baixo) |
| `imgsz` | `640` | Tamanho de entrada do YOLO |
| `banana_classes` / `poop_classes` | `[46]` / `[54]` | Classes do COCO que decidem direto |
| `classify_by_color` | `true` | Demais classes decididas pela cor da caixa |
| `banana_hsv_low/high` | `[20,120,100]` / `[40,255,255]` | Amarelo da banana (HSV do OpenCV) |
| `poop_hsv_low/high` | `[5,60,40]` / `[18,200,100]` | Marrom do poop |
| `unknown_as_poop` | `true` | Sem classe nem cor: obstáculo |
| `depth_percentile` | `20` | Percentil da profundidade dentro da caixa |
| `max_range` | `2.5` | Ignora detecções mais longe (m) |
| `color_proposals` | `false` | Também detectar por mancha de cor, sem YOLO |

### `perception_map`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `merge_radius` | `0.35` | Distância para considerar a mesma detecção (m) |
| `min_hits` | `2` | Vezes visto antes de entrar no mapa |
| `collect_radius` | `0.12` | Robô a menos disto: banana recolhida (m) |
| `collected_radius` | `0.30` | Raio em que uma recolhida bloqueia recriação (m) |
| `max_spin_to_create` | `0.5` | Giro máximo para criar objeto novo (rad/s) |

### `navigator`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `k_att` / `att_range` | `1.0` / `1.0` | Ganho e saturação da atração |
| `k_rep` / `d0_poop` / `d_min` | `0.08` / `0.55` / `0.08` | Repulsão |
| `f_max` | `3.0` | Saturação da resultante |
| `v_max` / `w_max` / `k_heading` | `0.25` / `1.8` / `3.0` | Velocidades e ganho de rumo |
| `turn_in_place` | 75° | Acima disso gira parado |
| `collect_radius` | `0.10` | Distância que conta como banana alcançada (m) |
| `search_w` / `search_v` | `0.4` / `0.18` | Giro e avanço da busca |
| `search_spin_time` / `search_move_time` | `16` / `3` | Duração das fases da busca (s) |
| `search_timeout` | `120` | Desiste de procurar depois disso (s) |
| `pose_timeout` | `1.0` | Sem pose nova por este tempo: para o robô (s) |
| `stall_time` / `swirl_time` / `swirl_gain` | `2.0` / `2.5` / `1.2` | Mínimos locais |

### `coppelia_bridge`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `map_source` | `signals` | `perception` no launch da câmera: não publica o mapa dos sinais |
| `stop_after_bananas` | `20` | Meta de bananas; atingida, a ponte para a simulação e o launch encerra |
| `image_rate` | `5.0` | Imagens e TF por segundo (Hz) |
| `wheel_radius` / `wheel_separation` | `0.05` / `0.0` | Geometria; `0.0` mede a separação na cena |
| `motor_sign` | `-1.0` | Nesta cena, velocidade negativa leva o robô para a frente |
| `cmd_timeout` | `0.5` | Sem `cmd_vel` por este tempo: para |

---

## 8. Problemas comuns

| Sintoma | O que fazer |
|---|---|
| `ModuleNotFoundError: ultralytics` ou erro no `cv_bridge` | Instale como na seção 1, com `"numpy<2"` |
| `Não consegui falar com o CoppeliaSim` | Abra o simulador; se já estiver aberto, algum script da cena trava a thread principal |
| A ponte não imprime nada e a simulação não começa | O CoppeliaSim pode estar noutra porta: ele usa a 23001 se a 23000 estiver ocupada quando sobe. Confira com `ss -ltnp \| grep coppeliaSim` e passe `port:=23001` |
| O CoppeliaSim fecha sozinho (*signal 11*) | Está em modo headless; a Kinect precisa de interface gráfica |
| `yolo/annotated` não aparece | O `yolo_vision` só processa quando chegam cor, profundidade e `camera_info`; confira a ponte |
| `sem TF de camera_color_optical_frame para world` | A ponte não está publicando imagens; no launch da câmera isso é automático |
| O mapa enche de cópias do mesmo objeto | Aumente `merge_radius`, ou reduza `max_spin_to_create` |
| Bananas viram poop no mapa (ou o contrário) | Ajuste as faixas HSV; confira as caixas em `yolo/annotated` |
| O robô atravessa poops | Os poops ainda não estavam no mapa: aumente `k_rep`/`d0_poop` ou reduza `v_max` |
| `estes scripts da cena escrevem nos motores` | Desabilite `/myRobot/python_controler` e `/myRobot/controler` |
| O robô anda para trás | Troque `motor_sign` para `1.0` |
| `Package 'potential_field' not found` | Faltou sourcear o workspace; no zsh, `install/setup.zsh` |
