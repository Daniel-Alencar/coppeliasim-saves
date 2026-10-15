# a_star_banana

Planejamento global com **A\*** sobre um mapa de ocupação montado da própria
cena: o robô vai do ponto onde está até uma banana escolhida, desviando dos
poops, e quem o move é o **controlador de posição do exercício anterior** (a
lei ρ-α-β), alimentado com os waypoints que saem do caminho planejado.

É o exercício descrito em [requirements.md](requirements.md), com a cena
[coppeliasim/a_star_banana.ttt](coppeliasim/a_star_banana.ttt) (a mesma de
`projects/8 - A*/`).

Nenhuma coordenada é fixa no código: as posições do robô, dos 49 poops e das 11
bananas vêm da hierarquia da cena pela ZeroMQ Remote API.

```
                      poops, bananas, banana_paths, pose, sim_time
CoppeliaSim ──ZeroMQ──▶ coppelia_bridge ─────────────────────────▶ planner
     ▲                        ▲                                      │ goal
     │ motores                │ cmd_vel                               ▼
     └────────────────────────┴──────────────────────────── position_control
```

| Executável | Papel |
|---|---|
| `coppelia_bridge` | Só comunicação: lê a cena, publica pose, poops, bananas e o relógio da simulação; aplica o `cmd_vel` nas rodas |
| `planner` | Monta a grade de ocupação, infla os poops, roda o A\* e manda um waypoint de cada vez para `goal` |
| `position_control` | A lei de controle do exercício anterior: recebe `goal`, publica `cmd_vel` |

---

## 1. Pré-requisitos

- ROS 2 Jazzy.
- No Python do sistema (o `ros2 run` não usa o Anaconda nem um `.venv/`):

```bash
/usr/bin/python3 -m pip install --user --break-system-packages coppeliasim-zmqremoteapi-client
```

- CoppeliaSim 4.10 aberto com a cena **`a_star_banana.ttt`**.

> A cena tem um `/sim_prep` que, ao **parar** a simulação, salva o chão sujo em
> `result.ttm` e recarrega `dirty_floor.ttm` **da pasta da cena**. Sem esse
> modelo ali, o fim de cada simulação termina em
> `[/sim_prep:error] in sim.loadModel: file does not exist` — erro inofensivo
> (a navegação já acabou), mas feio. Por isso o `dirty_floor.ttm` acompanha as
> duas cópias da cena, aqui e em `projects/8 - A*/`. O `result.ttm` é gerado a
> cada parada e não vai para o git.

### Os scripts da cena

| Script | Faz | Interfere? |
|---|---|---|
| `/dirt_script` | Recolhe o que passa sob o `/myRobot/poopximity`: manda para z = 1000 e soma no placar | Não. É assim que a ponte sabe que a banana foi pega |
| `/room_setup`, `/sim_prep` | Textura do chão suja/limpa e salvamento do resultado | Não |
| `/myRobot/python_controler` | Joystick: escreve nos motores **a cada passo** | **Sim.** A ponte o desabilita enquanto roda e o devolve ao sair (`disable_rival_scripts`) |
| `/myRobot/odometry`, `encoder`, `battery` | Odometria das rodas e bateria | Não; e não são usados: aqui a pose vem do gabarito da cena |

---

## 2. Compilar

```bash
cd ros2/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select a_star_banana
source install/setup.bash      # ou install/setup.zsh
```

> Mesmo com `--symlink-install`, os módulos Python de um pacote `ament_python`
> são **copiados** para `install/`. Depois de editar qualquer `.py`, rode o
> `colcon build` de novo — senão o `ros2 launch` continua rodando o código
> antigo.

---

## 3. Como executar

**Terminal 1** — CoppeliaSim com a cena aberta (não precisa dar *play*: a ponte
dá).

**Terminal 2** — tudo de uma vez:

```bash
cd ros2/ros2_ws
source install/setup.bash
ros2 launch a_star_banana a_star_banana.launch.py
```

O robô planeja até a banana mais barata, segue o caminho e para quando a cena
recolhe a banana. O mapa de ocupação aparece desenhado no terminal e, junto com
o caminho, no rviz.

### Argumentos do launch

| Argumento | Padrão | O que faz |
|---|---|---|
| `goal` | `''` | `''` = banana de menor custo A\*; `'/bananas/Banana[4]'` = aquela banana; `'1.5,-0.8'` = um ponto qualquer |
| `resolution` | `0.2` | Lado da célula do mapa, em metros |
| `robot_radius` | `0.16` | Raio do robô (medido na cena: 0,32 m de diâmetro) |
| `inflate` | `0.09` | Margem de segurança **além** do raio do robô |
| `tolerance` | `0.15` | Raio para dar um waypoint como alcançado |
| `diagonal` | `true` | `true` = vizinhança de 8; `false` = de 4 |
| `collect_all` | `false` | `true` replaneja para a próxima banana depois de cada coleta |
| `v_const` | `0.22` | Velocidade de cruzeiro, em m/s |
| `stop_on_done` | `true` | Parar a simulação ao terminar |
| `port` | `23000` | Porta da ZeroMQ Remote API |
| `rviz` | `1` | `0` não abre o rviz |

Exemplos:

```bash
# uma banana escolhida pelo caminho da API
ros2 launch a_star_banana a_star_banana.launch.py goal:='/bananas/Banana[4]'

# grade mais fina e inflação maior
ros2 launch a_star_banana a_star_banana.launch.py resolution:=0.1 inflate:=0.2

# destino inalcançável (ponto fora da sala)
ros2 launch a_star_banana a_star_banana.launch.py goal:='3.5,3.5'

# coletar todas as bananas, uma de cada vez
ros2 launch a_star_banana a_star_banana.launch.py collect_all:=true
```

Para rodar um nó de cada vez (três terminais, cada um com o `install/setup.bash`):

```bash
ros2 run a_star_banana coppelia_bridge   --ros-args -r __ns:=/myRobot
ros2 run a_star_banana position_control  --ros-args -r __ns:=/myRobot
ros2 run a_star_banana planner           --ros-args -r __ns:=/myRobot -p goal:='/bananas/Banana[0]'
```

### O que aparece no terminal

```
[planner] mapa 27x27 células de 0.20 m, inflação 0.25 m (2 células), 49 poops no mapa
[planner] caminho até /bananas/Banana[4]: 6 células, 1.08 m, 3 waypoints, 6 nós expandidos
[planner] mapa 27x27 | 0.20 m/célula | inflação 0.25 m
          legenda: . livre  # poop  o margem  * caminho  R robô  B alvo
          ...
          o........o...R...o........o
          o.......o#o...****B.......o
          ...
[planner] waypoint 1/3: (+0.02, -0.02)
[position_control] novo alvo: (+0.02, -0.02) rumo -45°
[bridge]  placar da cena: 1/11 bananas
[planner] banana recolhida pela cena (1 no total)
```

---

## 4. Os tópicos

Todos no namespace `myRobot`.

| Tópico | Tipo | Quem publica |
|---|---|---|
| `/myRobot/pose` | `geometry_msgs/PoseStamped` | ponte — pose do robô no mundo, já com a frente em +x |
| `/myRobot/poops` | `geometry_msgs/PoseArray` | ponte — os 49 obstáculos (lidos uma vez) |
| `/myRobot/bananas` | `geometry_msgs/PoseArray` | ponte — as bananas ainda no chão |
| `/myRobot/banana_paths` | `std_msgs/String` (JSON) | ponte — os caminhos da API, na mesma ordem |
| `/myRobot/collected` | `std_msgs/Int32` | ponte — placar da cena |
| `/myRobot/sim_time` | `std_msgs/Float64` | ponte — relógio da simulação |
| `/myRobot/goal` | `geometry_msgs/PoseStamped` | planner — **o waypoint atual** |
| `/myRobot/path` | `nav_msgs/Path` | planner — o caminho planejado |
| `/myRobot/grid` | `nav_msgs/OccupancyGrid` | planner — o mapa já inflado |
| `/myRobot/cmd_vel` | `geometry_msgs/Twist` | position_control |
| `/myRobot/done` | `std_msgs/Bool` | planner — acabou; a ponte para a simulação |

Dá para dirigir o robô sem o planejador, publicando um alvo à mão:

```bash
ros2 topic pub --once /myRobot/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: world}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}'
```

---

## 5. Como funciona

### Tarefa 1 — informação da cena

A ponte acha `/myRobot`, percorre os filhos de `/poops` e `/bananas` com
`sim.getObjectChild` e guarda o *handle* e o caminho de cada um
(`/bananas/Banana[4]`). As posições saem de `sim.getObjectPosition` no
referencial do mundo.

A frente do `/myRobot` é o eixo **+y** dele, e não o +x: a ponte converte
(`yaw = atan2(m[5], m[1])`), e o que sai em `pose` já está na convenção do ROS.

Os poops são lidos uma vez só — eles não se mexem. As bananas são relidas em
rodízio, **uma por ciclo**: ler as onze de uma vez segurava a pose por mais de
um segundo, e o controlador parava o robô achando que a ponte tinha caído.
Quando uma banana aparece em z > 100, a cena a recolheu.

### Tarefa 2 — grade de ocupação e inflação

[`grid.py`](a_star_banana/grid.py). A caixa do mapa é o retângulo que contém
todos os objetos da cena mais `arena_margin`; a borda da matriz fica bloqueada,
que é o papel das paredes da sala.

Cada poop ocupa a célula em que cai. A **inflação** marca como bloqueada toda
célula cujo centro esteja a menos de `robot_radius + safety_margin` do centro de
um poop — a distância vira número de células pela resolução, e o disco é
percorrido uma vez por obstáculo. Como o A\* anda com o **centro** do robô, essa
margem é o que garante que o corpo dele não encoste em nada.

Na saída, poop vira 100 e margem vira 99 (`occupancy_data`), só para aparecerem
em tons diferentes no rviz; para o planejador os dois são bloqueio.

### Tarefa 3 — A\*

[`a_star.py`](a_star_banana/a_star.py). Vizinhança de 8, custo 1 na reta e √2 na
diagonal, heurística octile (admissível para 8 vizinhos, então o caminho é
ótimo). O desempate prefere menor `h`, o que deixa a busca mais direta. Diagonal
só é permitida quando as duas células ortogonais estão livres — senão o caminho
"corta a quina" de um poop por um vão de largura zero.

Se o robô ou a banana caem dentro da margem (banana encostada num poop,
inflação generosa), `nearest_free` procura em anéis a célula livre mais próxima
para servir de partida ou chegada, em vez de desistir.

O caminho é simplificado: ficam só as células onde ele **muda de direção**.
São esses os waypoints — mandar célula por célula faria o robô parar e
realinhar a cada 20 cm.

### Tarefa 4 — navegação

O planner publica um waypoint de cada vez em `/myRobot/goal` e só manda o
seguinte quando o robô está a menos de `waypoint_tolerance` do atual
(`goal_tolerance`, menor, no último, que é a banana de verdade e não o centro da
célula — é sobre ela que o sensor precisa passar). O rumo pedido em cada
waypoint é a direção do trecho seguinte, para o robô já sair alinhado com a
curva.

Quem move o robô é o `position_control`, **sem mudança de lei**:

```
ρ = √(Δx² + Δy²)        α = atan2(Δy, Δx) − θ        β = θ_g − θ − α
v = k_ρ ρ               ω = k_α α + k_β β
```

com os ganhos (3, 8, −1,5) satisfazendo as condições de estabilidade do
enunciado. As duas melhorias da parte 2 estão lá: **marcha a ré** (com o alvo
atrás, |α| > π/2, o erro é medido com o robô de costas e v sai negativo) e
**velocidade constante** (v e ω são escalados juntos, o que mantém a curvatura
v/ω e tira a desaceleração exponencial; só dentro de `approach_radius` a
velocidade volta a cair, para parar no ponto).

Planejador e controlador usam o mesmo referencial: `world`, o da cena.

### Quando dá errado

- **Sem caminho**: o planner diz quantos nós expandiu, desenha o mapa e encerra
  (ou pula para a próxima banana, com `collect_all:=true`).
- **Waypoint que não chega**: passado `replan_timeout` (25 s **de simulação**),
  replaneja da pose atual; depois de `attempts_per_banana` tentativas, desiste
  daquela banana.
- Os prazos são contados no **relógio da simulação** (`/myRobot/sim_time`), e
  não no do computador: numa máquina sem GPU esta cena roda a ~0,2x do tempo
  real, e um prazo de relógio viraria poucos segundos de simulação.

---

## 6. Experimentos pedidos

```bash
# 1. navegação bem-sucedida
ros2 launch a_star_banana a_star_banana.launch.py goal:='/bananas/Banana[4]'

# 2. mesma banana, outra resolução
ros2 launch a_star_banana a_star_banana.launch.py goal:='/bananas/Banana[4]' resolution:=0.1
ros2 launch a_star_banana a_star_banana.launch.py goal:='/bananas/Banana[4]' resolution:=0.4

# 3. destino inalcançável
ros2 launch a_star_banana a_star_banana.launch.py goal:='3.5,3.5'       # fora da sala
ros2 launch a_star_banana a_star_banana.launch.py inflate:=0.6          # fecha as passagens
```

**Resolução.** Célula grande (0,4 m) dá poucas células, busca rápida e caminho
grosseiro: um poop ocupa uma célula inteira e a inflação arredonda para 1
célula, então passagens estreitas somem e o robô dá voltas maiores — ou o alvo
fica inalcançável. Célula pequena (0,1 m) acha passagens que existem de verdade
e dá um caminho mais curto, ao custo de mais nós expandidos (a área cresce com o
quadrado) e de mais waypoints, com mais paradas para realinhar.

**Inflação.** É ela que separa "o caminho existe no mapa" de "o robô cabe no
caminho". Abaixo do raio do robô (0,16 m) o planejador manda o robô raspar nos
poops — e pisar num poop faz a cena espalhá-lo e contar ponto negativo. Muito
acima, o mapa fecha corredores legítimos e o A\* devolve "sem caminho". O padrão
(0,16 + 0,09 = 0,25 m, 2 células a 0,2 m) é o meio-termo que passa entre dois
poops vizinhos sem encostar.

**Tolerância do waypoint.** Grande (≥ 0,3 m) o robô "corta" os cantos e o
caminho real fica mais liso, mas também mais perto dos obstáculos do que o
planejado. Pequena (≤ 0,05 m) ele persegue cada ponto até acertar, o que, com o
controlador desacelerando perto do alvo, vira uma sequência de paradas — e o
último ponto, a banana, exige precisão mesmo (`goal_tolerance` 0,06 m), senão o
sensor não passa por cima dela.

---

## 7. Parâmetros

### `planner`

| Parâmetro | Padrão | O que é |
|---|---|---|
| `resolution` | `0.2` | Lado da célula [m] |
| `robot_radius` | `0.16` | Raio do robô [m] |
| `safety_margin` | `0.09` | Margem além do raio [m] |
| `arena_margin` | `0.4` | Folga da caixa do mapa além dos objetos [m] |
| `diagonal` | `true` | Vizinhança de 8 |
| `goal` | `''` | Alvo: vazio, caminho da API, ou `x,y` |
| `collect_all` | `false` | Replanejar para a próxima banana |
| `waypoint_tolerance` | `0.15` | Raio de "cheguei" num waypoint [m] |
| `goal_tolerance` | `0.06` | O mesmo, no último ponto [m] |
| `replan_timeout` | `25.0` | Tempo de simulação até replanejar [s] |
| `attempts_per_banana` | `3` | Replanejamentos antes de desistir |
| `collect_timeout` | `6.0` | Espera em cima da banana [s] |
| `show_map` | `true` | Desenhar o mapa no log |

### `position_control`

| Parâmetro | Padrão | O que é |
|---|---|---|
| `k_rho`, `k_alpha`, `k_beta` | `3.0`, `8.0`, `-1.5` | Ganhos da lei |
| `v_const` | `0.22` | Velocidade de cruzeiro [m/s] |
| `v_max`, `w_max` | `0.30`, `2.0` | Tetos de avanço e giro |
| `approach_radius` | `0.25` | Onde começa a desacelerar [m] |
| `turn_in_place` | `75°` | Acima disso, gira parado |
| `allow_reverse` | `true` | Marcha a ré |
| `position_tolerance` | `0.05` | Parar a esta distância do alvo [m] |
| `align_final_yaw` | `false` | Alinhar o rumo final (nos waypoints não importa) |
| `pose_timeout` | `2.0` | Sem pose por este tempo, para o robô [s] |

### `coppelia_bridge`

| Parâmetro | Padrão | O que é |
|---|---|---|
| `host`, `port` | `localhost`, `23000` | ZeroMQ Remote API |
| `robot`, `poops_path`, `bananas_path` | `/myRobot`, `/poops`, `/bananas` | Caminhos na cena |
| `stepping` | `true` | A ponte dá os passos da simulação |
| `disable_rival_scripts` | `true` | Desabilita o `python_controler` enquanto roda |
| `wheel_radius`, `wheel_separation` | `0.05`, `0` (mede na cena) | Geometria do robô |
| `motor_sign` | `-1.0` | Junta invertida: negativo anda para a frente |
| `map_rate` | `3.0` | Bananas relidas por segundo (uma por ciclo) |
| `autostart`, `stop_on_done` | `true`, `true` | Dar play ao subir, parar ao terminar |

---

## 8. Problemas comuns

| Sintoma | Causa provável |
|---|---|
| `Não consegui falar com o CoppeliaSim` | Simulador fechado, outra porta, ou um script da cena travado |
| O robô não sai do lugar | Outro script escrevendo nos motores. A ponte avisa quais são; com `disable_rival_scripts:=false` ela só avisa, não desabilita |
| `sem pose há N s` | O simulador está servindo poucas chamadas por segundo (cena pesada, máquina sem GPU). Aumente `pose_timeout` |
| Simulação parece travada depois de um Ctrl+C violento | A ponte ficou em *stepping* sem devolver o controle: pare e dê play de novo no CoppeliaSim |
| `sem caminho: o objetivo é inalcançável` | Inflação grande demais, resolução grossa demais, ou a banana está mesmo cercada |
| O robô chega na banana e ela não some | Ele parou perto, mas não **em cima**: diminua `goal_tolerance` / `position_tolerance` |

---

## 9. Arquivos

| Arquivo | O que tem |
|---|---|
| [a_star_banana/coppelia_bridge.py](a_star_banana/coppelia_bridge.py) | Leitura da cena, pose, motores, relógio |
| [a_star_banana/grid.py](a_star_banana/grid.py) | Grade de ocupação, inflação, conversões |
| [a_star_banana/a_star.py](a_star_banana/a_star.py) | A busca A\* e a simplificação do caminho |
| [a_star_banana/planner.py](a_star_banana/planner.py) | O nó: planeja e executa por waypoints |
| [a_star_banana/position_control.py](a_star_banana/position_control.py) | A lei ρ-α-β do exercício anterior |
| [launch/a_star_banana.launch.py](launch/a_star_banana.launch.py) | Sobe os três nós e o rviz |
| [config/a_star_banana.rviz](config/a_star_banana.rviz) | Mapa, caminho, waypoint e objetos no rviz |
| [coppeliasim/a_star_banana.ttt](coppeliasim/a_star_banana.ttt) | A cena, com o `dirty_floor.ttm` que ela recarrega ao parar |
| [requirements.md](requirements.md) | O enunciado |

Os scripts autônomos do `projects/8 - A*/scripts/` (`scene_objects.py`,
`grid_map.py`, `a_star.py`, `collect_bananas.py`) fazem o mesmo sem ROS, direto
pela Remote API, e continuam valendo como ferramenta de inspeção rápida.
