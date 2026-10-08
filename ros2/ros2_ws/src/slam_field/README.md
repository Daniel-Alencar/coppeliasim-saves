# slam_field

Navegação por **campos potenciais** sobre o mapa do **SLAM Toolbox**: o robô
vai até as bananas desviando dos obstáculos que o laser 2D colocou no `/map`, e
o scan atual faz uma **parada de emergência** contra o que o mapa ainda não
tem. É o exercício descrito em
[coppeliasim/requirements.md](../../../../coppeliasim/requirements.md), com a
cena [coppeliasim/p3_slam_toolbox.ttt](../../../../coppeliasim/p3_slam_toolbox.ttt).

```
F = F_atrativa(banana alvo) + Σ_setores F_repulsiva(borda inflada mais próxima)
```

As paredes **não** são posições fixas no código: saem do `/map`. As posições
das bananas são o gabarito da cena (sinal `banana`), como o enunciado permite.

```
                 /scan, /odom, /tf (odom→base_link, base_link→laser)
CoppeliaSim ──ZeroMQ──▶ coppelia_bridge ─────────────────▶ slam_toolbox ──/map, /tf (map→odom)──┐
     ▲                     │  ▲  /bananas, /ground_truth                                      │
     │ motores             │  │                                                               ▼
     └─────────────────────┘  └──────────────── /cmd_vel ───────────────────────────────── navigator
```

| Executável | Papel |
|---|---|
| `coppelia_bridge` | Só comunicação: publica o laser como `LaserScan`, a odometria (gabarito ou rodas) e a TF; aplica o `cmd_vel` nas rodas; publica as bananas |
| `slam_toolbox` (`async_slam_toolbox_node`) | Monta o `/map` e publica `map → odom` |
| `navigator` | Infla o mapa, soma as forças, faz a parada de emergência e publica `cmd_vel` |

---

## 1. Pré-requisitos

- ROS 2 Jazzy com `slam_toolbox`, `rviz2` e `tf2_ros`:

```bash
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard
```

- No Python do sistema (o `ros2 run` não usa o Anaconda nem o `.venv/`):

```bash
/usr/bin/python3 -m pip install --user --break-system-packages coppeliasim-zmqremoteapi-client
```

- CoppeliaSim aberto com a cena **`coppeliasim/p3_slam_toolbox.ttt`**. Os
  modelos `banana.ttm` e `poop.ttm` precisam estar na mesma pasta da cena: o
  `/buildScene` os carrega de lá no *play*.

### Os scripts da cena

Exportados em [coppeliasim/scripts/](../../../../coppeliasim/scripts):

| Script | Faz | Interfere? |
|---|---|---|
| `/buildScene` | Sorteia 20 bananas e 20 poops em ±2,5 m no *play*; publica os sinais `banana`/`poop` | Não. A ponte lê o sinal `banana` |
| `/myRobot/LaserScanner_2D` | Gira o sensor de 180° em passos de 0,4° e grava 452 distâncias em `signal.distSignal` | Não. É o laser que a ponte publica |
| `/myRobot/odometry`, `encoder` | Odometria das rodas numa propriedade do robô | Não; e **não é usada**: ela supõe a frente do robô em +x, e aqui é +y. A ponte integra as rodas por conta própria |
| `/myRobot/battery` | Nível de bateria num sinal | Não |
| `/myRobot/controler`, `python_controler` | **Escrevem nos motores** a cada passo | Estão **desabilitados** na cena. Se forem habilitados, anulam o `cmd_vel`; a ponte avisa |

A cena **não tem** `/dirt_script`: nada some sozinho. Quando o navegador
alcança uma banana, a ponte a manda para z = 1000, para aparecer no vídeo.
O laser está a 20 cm do chão: ele vê paredes e móveis, mas **não** vê bananas
nem poops (os poops não são obstáculo neste exercício).

---

## 2. Compilar

```bash
cd ros2/ros2_ws
colcon build --packages-select slam_field --symlink-install
source install/setup.bash
source install/setup.zsh
ros2 pkg executables slam_field
```

---

## 3. Executar

Com o CoppeliaSim aberto e a cena carregada. A ponte dá o *play* sozinha e, no
fim (todas as bananas), para a simulação; o SLAM e o rviz continuam abertos
para salvar o mapa.

**Terminal 1 — tudo (gabarito como odometria):**

```bash
ros2 launch slam_field slam_field.launch.py
```

**Com odometria das rodas:**

```bash
ros2 launch slam_field slam_field.launch.py odom:=wheel
```

O rviz abre com o `/map`, o mapa inflado por cima (cores de *costmap*), o
scan em vermelho, as bananas em amarelo, o alvo em verde e o gabarito da pose
em eixos. No terminal:

```
[navigator]: mapa 212x198 @ 0.050 m: 1480 ocupadas, 21034 livres, 19462 desconhecidas; inflação 0.25 m = 5 células
[navigator]: bananas (use banana_ids:=N para testar uma):
banana   x       y     folga   situação
   0   -1.21    0.84   1.73   aberta
   1    2.10   -2.31   0.38   parede
...
[navigator]: alvo: banana em (-1.21, 0.84), free
[navigator]: banana 1/20 alcançada em (-1.21, 0.84)
[navigator]: erro da pose no map: 1.3 cm, 0.4°
```

### Argumentos do launch

| Argumento | Padrão | O que faz |
|---|---|---|
| `odom` | `ground_truth` | `ground_truth` (pose da cena) ou `wheel` (integração das rodas) |
| `navigator` | `1` | `0` sobe só ponte + SLAM + rviz, para mapear com teleop |
| `banana_ids` | vazio | Só estas bananas, ex. `4` ou `4,9,12` (índices da tabela acima) |
| `v_max` | `0.15` | Velocidade de avanço (m/s) |
| `unknown_is_obstacle` | `false` | `true` trata célula desconhecida como obstáculo |
| `stop_on_done` | `true` | Parar a simulação quando acabarem as bananas |
| `rviz` | `1` | `0` não abre o rviz |
| `robot` / `port` | `/myRobot` / `23000` | Robô e porta da Remote API |
| `slam_params_file` | `config/slam_toolbox.yaml` | Parâmetros do SLAM Toolbox |

---

## 4. Roteiro de testes (o do enunciado)

**1. Mapa completo, baixa velocidade.** Primeiro só mapear, dirigindo pelo
teclado; depois soltar o navegador sobre o mapa pronto:

```bash
# Terminal 1
ros2 launch slam_field slam_field.launch.py navigator:=0
# Terminal 2
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Passeie pela casa até o `/map` fechar. Verifique que tudo atualiza:

```bash
ros2 topic hz /scan                 # ~10 Hz
ros2 topic echo /map --once --field info
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames      # map → odom → base_link → laser
```

Com o mapa pronto, no Terminal 2 (sem fechar o Terminal 1, que mantém o mapa):

```bash
ros2 run slam_field navigator --ros-args -p unknown_is_obstacle:=true -p v_max:=0.1
```

**2. Uma banana em espaço aberto** e **3. uma banana perto da parede:**
escolha na tabela que o navegador imprime (`aberta` / `parede` / `inflada`):

```bash
ros2 run slam_field navigator --ros-args -p banana_ids:=0
ros2 run slam_field navigator --ros-args -p banana_ids:=1
```

**4. Várias bananas:** `-p banana_ids:=0,1,5` ou sem o parâmetro (todas).

**5. Mapa sendo construído:** do zero, tudo junto:

```bash
ros2 launch slam_field slam_field.launch.py
```

**Repetir com odometria das rodas:** os mesmos passos com `odom:=wheel`. Para
comparar, o navegador imprime a cada 5 s o erro entre a pose do SLAM (TF
`map → base_link`) e a pose verdadeira (`/ground_truth`). Para ver a deriva
da odometria pura, sem a correção do SLAM:

```bash
ros2 run tf2_ros tf2_echo odom base_link      # compare com /ground_truth
```

### Salvar o mapa

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'mapa_gabarito'}}"
```

Gera `mapa_gabarito.pgm` e `.yaml` na pasta de onde o launch foi chamado — útil
para comparar lado a lado os mapas com gabarito e com rodas.

---

## 5. Tópicos

| Tópico | Tipo | Quem publica | Conteúdo |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | ponte | 452 leituras, −90° a +90,4°, passo 0,4°, até 10 m (sem retorno = `inf`) |
| `/odom` | `nav_msgs/Odometry` | ponte | Pose e velocidade no `odom` (gabarito ou rodas) |
| `/tf` | | ponte, SLAM | `odom → base_link` (ponte), `map → odom` (SLAM) |
| `/tf_static` | | ponte | `base_link → laser` (3 mm à frente, 20 cm de altura) |
| `/ground_truth` | `geometry_msgs/PoseStamped` | ponte | Pose verdadeira, só para comparação |
| `/bananas` | `geometry_msgs/PoseArray` | ponte | Gabarito das bananas, no `map` (transient local) |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM | Mapa de ocupação, 5 cm, atualizado a cada 1 s |
| `/inflated_map` | `nav_msgs/OccupancyGrid` | navegador | 100 = bloqueado (inflado), 0 = livre, −1 = desconhecido |
| `/cmd_vel` | `geometry_msgs/Twist` | navegador | `linear.x` (m/s), `angular.z` (rad/s) |
| `/target`, `/force` | `PointStamped`, `Vector3Stamped` | navegador | Alvo atual e resultante do campo |
| `/collected`, `/collected_banana` | `Int32`, `PointStamped` | navegador | Contagem e banana recém-alcançada (a ponte a tira da cena) |
| `/done` | `std_msgs/Bool` | navegador | `true` quando acabaram as bananas |

---

## 6. Como funciona

### Odometria e SLAM (`coppelia_bridge` + `slam_toolbox`)

A ponte lê, numa thread com conexão própria ao simulador, o laser, a pose e as
juntas das rodas, e publica `/scan` e `odom → base_link` **com o mesmo
carimbo de tempo**: o SLAM Toolbox casa cada scan com a TF do instante dele.

- `odom:=ground_truth`: a pose da cena vira a odometria, sem erro. Serve para
  separar problema de SLAM de problema de odometria.
- `odom:=wheel`: integração das juntas: cada roda avança `Δφ · r` (r = 0,05 m),
  `Δs = (sₑ + s_d)/2`, `Δθ = (s_d − sₑ)/L` (L = 0,2 m medido na cena), e a
  pose é integrada pelo ponto médio do arco.

As duas começam na pose verdadeira do robô. Como o SLAM Toolbox põe o `map`
sobre o `odom` do primeiro scan, **`map` coincide com o mundo da cena** — é o
que permite usar as posições das bananas direto no `map`.

A frente do `/myRobot` é o +y dele; o `base_link` publicado tem a frente em +x
(convenção do ROS). O laser varre da direita (índice 0) para a esquerda, como o
`LaserScan` espera.

### Extração dos obstáculos (`grid.py`)

A `OccupancyGrid` é um vetor em ordem de linhas: a célula (linha, coluna) está
no índice `linha·width + coluna`, e seu centro no `map` é

```
x = origin.x + (coluna + 0,5)·resolution      y = origin.y + (linha + 0,5)·resolution
```

| Valor | Classe | Tratamento |
|---|---|---|
| ≥ 65 (`occupied_threshold`) | ocupada | obstáculo: é inflada e repele |
| 0 … 64 | livre | livre |
| −1 | desconhecida | **livre** por padrão |

**Desconhecido como livre:** no começo quase todo o mapa é desconhecido;
tratá-lo como obstáculo prenderia o robô onde está. Quem protege contra o que
o mapa ainda não mostra é a parada de emergência, que usa só o scan atual. Com
o mapa já completo, `unknown_is_obstacle:=true` impede o robô de sair para
fora da área mapeada.

### Inflação

Raio de inflação = `robot_radius` (0,17 m; o robô mede 0,167 m do centro à
borda) + `safety_margin` (0,08 m) = 0,25 m. Em células:
`ceil(0,25 / 0,05) = 5`. Cada célula ocupada marca como **bloqueadas** todas as
células a até 5 células dela (um disco). É uma dilatação feita por
deslocamentos do vetor inteiro (81 deslocamentos para o disco de raio 5), ~4 ms
num mapa 200×200, então ela é **refeita a cada `/map` novo** (1 s).

Com a inflação, o robô vira um ponto: basta o **centro** ficar fora da região
bloqueada para o corpo ficar a pelo menos 8 cm de qualquer obstáculo.

### Força repulsiva

Só a **vizinhança** do robô é examinada: uma janela de `influence` (0,4 m) em
volta dele no mapa inflado. Dentro dela, as células bloqueadas são separadas em
`sectors` (12) setores angulares de 30°, e de cada setor entra **um único
ponto: o mais próximo**, que é a borda da região inflada voltada para o robô.
Cada um repele com a forma clássica:

```
F = k_rep · (1/d − 1/influence) / d²     (d = folga do centro do robô até a borda inflada)
```

Somar a força de **todas** as células ocupadas faria a repulsão depender da
resolução (o mesmo muro com células de 2,5 cm teria 4× mais células) e do
tamanho do obstáculo (um armário grande empurraria mais que uma parede fina à
mesma distância). Com um ponto por setor, um obstáculo conta pela geometria
— quantos setores ele ocupa —, e não pelo número de células.

Se o centro do robô estiver **dentro** da região inflada (o mapa mudou, a
odometria pulou), a soma perde o sentido. Aí vale uma força de fuga, para
longe da célula ocupada mais próxima, e a atração é desligada até sair.

### Força atrativa e comando

Atração para a banana alvo com módulo constante `k_att` até 0,3 m dela (ver
"travamentos" abaixo). A resultante é saturada em `f_max`, e a direção dela é o
rumo desejado: o erro de rumo vira giro (`k_heading`), e o avanço é
`v_max·cos(erro)` (zero se o erro passar de 70°).

O alvo é a banana pendente de menor custo: distância + `blocked_penalty` × a
fração do segmento robô→banana que atravessa a região inflada.

### Parada de emergência

Usa **só o scan atual** — nada do mapa —, então vale para obstáculos que o
SLAM ainda não registrou:

- **corredor à frente**: pontos do scan com `x > 0` e `|y| ≤ raio + 6 cm`. O mais
  próximo a ≤ 0,30 m do centro **zera o avanço**; entre 0,30 e 0,55 m o avanço
  é reduzido (até 30% de `v_max`);
- **proximidade**: qualquer ponto da metade da frente a ≤ 0,24 m também para
  (obstáculo de lado, quando o robô avança em curva).

Parado, o robô gira no lugar para o lado que tinha mais espaço no scan no
momento da parada — sempre o mesmo lado —, e só volta a andar quando a folga
passar o limiar **mais 10 cm** (histerese). Sem isso ele oscilava na borda do
limiar diante de um obstáculo fora do mapa, sem sair do lugar. O controle
também para se o scan tiver mais de 0,6 s ou se a TF `map → base_link` faltar.

### Fim

Banana alcançada a 12 cm (ou a 45 cm se ela estiver dentro da região inflada:
encostada em parede ou móvel, o centro do robô não pode chegar nela). Quando
não sobra banana pendente, o navegador para o robô, publica `/done` e a ponte
para a simulação.

---

## 7. Onde o campo potencial trava

| Situação | O que acontece | O que o navegador faz |
|---|---|---|
| **Banana atrás de uma parede** (o mínimo local clássico) | Atração e repulsão se anulam diante da parede | Depois de 3 s sem se aproximar 5 cm do alvo, entra uma força **tangente à repulsão** (segue a borda), para o lado que mais aponta para o alvo. Ela fica ligada até o robô ficar mais perto do que estava no empate (critério do algoritmo *bug*) ou por até 60 s. Contornar uma parede de 4 m a 0,15 m/s leva ~2 min |
| **Banana dentro da região inflada** (colada a parede, móvel ou vaso) | A repulsão segura o robô antes de chegar | Conta como alcançada a ≤ 45 cm. Se nem assim, desiste após 4 empates ou 180 s, tenta de novo mais tarde, e após 3 tentativas a dá por **inalcançável** (sai no log final) |
| **Corredor estreito / porta** | Repulsão dos dois lados: com a margem de 8 cm, uma porta de menos de ~0,5 m fica fechada no mapa inflado | Tangente e desistência. Reduzir `safety_margin` abre passagens, à custa de folga |
| **Obstáculo fora do mapa** | A parada de emergência segura o robô, mas o campo continua puxando para ele | Gira para o lado livre até o corredor liberar e a tangente do empate o leva ao redor. Em geral o SLAM registra o obstáculo no `/map` seguinte e o campo passa a contorná-lo |
| **Odometria das rodas** | O mapa sai torto/duplicado quando o casamento de scans não corrige a deriva, e as bananas (no mundo) deixam de coincidir com o `map` | O erro aparece no log (`erro da pose no map`). Andar devagar e girar devagar ajuda o SLAM |

---

## 8. Validação feita sem o simulador

O `navigator.py` (o arquivo real, sem modificação) foi rodado contra um robô
diferencial cinemático, com laser de 180° por *ray casting* e um "SLAM" que
revela as células que o laser alcança, numa casa de 10×10 m com duas paredes
internas e um móvel. Resultados, a 0,15 m/s:

| Cenário | Resultado |
|---|---|
| 6 bananas, mapa sendo construído (desconhecido = livre) | 6/6 em 197 s; centro do robô sempre a ≥ 0,44 m dos obstáculos |
| Banana atrás de parede de 4 m | Alcançada em ~110–130 s, com 2 contornos pela tangente |
| Banana a 0,2 m da parede (dentro da região inflada) | Alcançada a ~0,3 m, sem entrar na região inflada |
| Obstáculo de 30 cm no caminho que **só o laser vê** (nunca entra no mapa) | Parada de emergência a 0,30 m, giro, contorno; banana alcançada; folga mínima 0,26 m (raio do robô 0,167 m) |

A odometria das rodas da ponte foi conferida separadamente (1 m em linha reta
e 90° no lugar, com as juntas passando de π para −π). Os testes **no
CoppeliaSim com o SLAM Toolbox** (seção 4) são os que valem para a entrega.

---

## 9. Arquivos

| Arquivo | Conteúdo |
|---|---|
| `slam_field/coppelia_bridge.py` | Ponte com o CoppeliaSim: laser, odometria, TF, bananas, motores |
| `slam_field/grid.py` | Classificação, inflação e repulsão sobre a `OccupancyGrid` (só numpy) |
| `slam_field/navigator.py` | Campo potencial, parada de emergência, escolha de alvo |
| `config/slam_toolbox.yaml` | SLAM Toolbox online async: frames, 5 cm, 10 m, atualização a 1 s |
| `config/slam_field.rviz` | Visualização |
| `launch/slam_field.launch.py` | Sobe tudo |
