# youbot_banana

O robô **pega** a banana. Todos os exercícios anteriores pararam em chegar até
ela; aqui um **manipulador móvel** — o KUKA youBot, base de quatro rodas suecas
mais um braço de 5 juntas — planeja com A\*, navega até uma pose de onde o braço
alcança, fecha a garra na banana, levanta e deposita numa bandeja. É o trabalho
descrito em [requirements.md](requirements.md), com a cena
[coppeliasim/youbot_banana.ttt](coppeliasim/youbot_banana.ttt).

A preensão é **física**: a garra fecha, trava contra a banana e segura por
atrito. Nada é grudado na mão por software.

```
                    bananas, poops, depósito, pose, juntas
CoppeliaSim ──ZeroMQ──▶ coppelia_bridge ──────────────────────▶ mission
     ▲                     ▲        ▲                           │   │
     │ rodas, juntas       │cmd_vel │arm_joints          goal ◀──┘   └──▶ arm_goal
     └─────────────────────┴────────┴──── base_control ◀─┘      arm_control ◀┘
```

| Executável | Papel |
|---|---|
| `coppelia_bridge` | Só comunicação: lê a cena, publica pose, bananas, poops, depósito e estado das juntas; aplica `cmd_vel` nas rodas, `arm_joints` no braço e `gripper` na garra |
| `base_control` | Controle de posição **holonômico**: três malhas proporcionais desacopladas → `cmd_vel` |
| `arm_control` | Cinemática inversa, trajetória interpolada e a garra → `arm_joints` |
| `mission` | A máquina de estados: A\* (reaproveitado), pose de preensão, sequência de manipulação, falhas e placar |

E a matemática, em Python puro e testável sem simulador:

| Módulo | Conteúdo |
|---|---|
| `kinematics.py` | Cinemática inversa da base de rodas suecas; direta e inversa do braço; garra |
| `coppelia_scene.py` | O que a ponte e os scripts precisam saber da hierarquia da cena |

---

## 1. O que há de novo em relação aos exercícios anteriores

### 1.1 O planejamento é reaproveitado, não reescrito

```python
from a_star_banana import a_star
from a_star_banana.grid import Grid, bounds_from_points
```

O A\* e o mapa de ocupação vêm do pacote do exercício anterior
([a_star_banana](../a_star_banana/)), por dependência declarada no
`package.xml`. O que muda são os números deste robô — o youBot tem footprint de
**0,42 × 0,57 m**, logo **0,36 m de raio circunscrito**, contra os 0,16 m do
robô diferencial — e um obstáculo novo, a bandeja.

### 1.2 A base é holonômica, e isso simplifica o controle

O robô diferencial comanda duas grandezas (v e ω) para três coordenadas
(x, y, θ): daí a lei ρ-α-β, com a curva que acerta posição e rumo ao mesmo
tempo, a marcha a ré quando o alvo está atrás e o cuidado com a estabilidade
dos ganhos.

As quatro rodas suecas dão as três liberdades. O problema se desfaz:

```
e_robô = R(-θ) · (alvo - pose)      o erro de posição, visto pelo robô
v      = k_p · e_robô               anda DIRETO para o alvo
ω      = k_θ · wrap(θ_alvo - θ)     e gira ao mesmo tempo, sem interferir
```

Três malhas proporcionais desacopladas, em vez de uma lei acoplada. É o
resultado mais limpo do trabalho, e vale a comparação no relatório.

### 1.3 O alvo da base não é a banana

É uma **pose de preensão**: a 0,40 m da banana, virada para ela, de onde o
braço alcança. Há uma por direção de aproximação, e elas não são equivalentes —
uma pode cair dentro da margem de um poop. Então `mission` testa 16 direções em
volta da banana, descarta as que caem em célula bloqueada, roda o A\* para as
que sobram e escolhe a de menor caminho. É o planejamento escolhendo a pose de
preensão.

### 1.4 Cinemática de manipulador

O braço do youBot tem a estrutura que torna a inversa **analítica**:

```
junta 0   eixo VERTICAL        gira o braço inteiro
junta 1   eixo +x  ─┐
junta 2   eixo +x   ├─ as três giram no MESMO plano: cadeia planar de 3 elos
junta 3   eixo +x  ─┘
junta 4   eixo da ferramenta   orienta a garra (não move a ponta)
```

A junta 0 resolve o azimute, as juntas 1-2-3 resolvem um problema **plano** de
três elos, e a junta 4 só gira os dedos. Fixando a direção do último elo
(`approach`), o pulso sai direto do alvo e sobra a cadeia de dois elos do livro,
com solução fechada pela lei dos cossenos. Sem essa estrutura seria preciso um
método numérico.

Toda constante geométrica foi **medida na cena**, não copiada de catálogo — ver
[§4](#4-as-medidas-e-de-onde-elas-vêm).

---

## 2. Como rodar

### 2.1 Pré-requisitos

- ROS 2 Jazzy.
- No Python do sistema (o `ros2 run` não usa o Anaconda nem um `.venv/`):

```bash
/usr/bin/python3 -m pip install --user --break-system-packages coppeliasim-zmqremoteapi-client
```

- CoppeliaSim aberto. A variável `COPPELIASIM_ROOT` aponta a instalação se ela
  não estiver em `~/Softwares/CoppeliaSim/CoppeliaSim_Edu`.

### 2.2 Compilar

```bash
cd ros2/ros2_ws
colcon build --symlink-install --packages-select a_star_banana youbot_banana
source install/setup.zsh     # use setup.bash se a sua shell for bash
```

### 2.3 Montar a cena

A cena **é gerada por script**, com o CoppeliaSim aberto:

```bash
cd src/youbot_banana
python3 scene/build_scene.py                        # a cena padrão
python3 scene/build_scene.py --seed 7 --bananas 8   # outro cenário
python3 scene/build_scene.py --poops 10 --arena 6
```

Ela é salva em `coppeliasim/youbot_banana.ttt` e em
`projects/9 - youbot_banana/`. Gerar em vez de montar à mão tem duas razões:
fica **reproduzível** (mesma semente, mesma cena, e trocar a semente dá um
cenário novo para repetir os experimentos) e fica **explicada** — cada decisão
de cena está comentada junto do código que a faz.

### 2.4 Rodar a missão

```bash
ros2 launch youbot_banana youbot_banana.launch.py
```

Parâmetros que valem mexer:

```bash
ros2 launch youbot_banana youbot_banana.launch.py resolution:=0.15
ros2 launch youbot_banana youbot_banana.launch.py grasp_reach:=0.18
ros2 launch youbot_banana youbot_banana.launch.py cartesian_speed:=0.04
ros2 launch youbot_banana youbot_banana.launch.py robot_radius:=0.30
ros2 launch youbot_banana youbot_banana.launch.py rviz:=0 show_map:=false
```

### 2.5 Conferir a cinemática contra a cena

```bash
python3 scene/measure_arm.py        # geometria do braço + direta + inversa + envelope
python3 scene/calibrate_base.py     # as três liberdades da base, uma a uma
pytest test/                        # a matemática, sem simulador
```

---

## 3. A máquina de estados

```
ESPERA ──> RECOLHE ──> PLANEJA ──> VAI_ATE ──> APROXIMA ──> DESCE ──> FECHA
                           ^                                             │
                           │                                             v
                   PROXIMA <── SOLTA <── POSICIONA <── VAI_DEPOSITO <── LEVANTA
                           │
                           └──> FIM (sem banana alcançável)
```

Cada estado tem como falhar, e cada falha tem um jeito de seguir:

| Falha | O que a missão faz |
|---|---|
| Nenhuma pose de preensão alcançável | Descarta a banana e tenta a próxima |
| Braço não alcança (fora do envelope) | Replaneja: a base pode parar melhor |
| Garra fechou no vazio | Abre, replaneja; na segunda vez, descarta |
| Banana caiu no transporte | Replaneja do zero — ela ainda está no chão |
| Banana ficou fora da bandeja | Conta como não entregue |
| Sem caminho até a bandeja | Encerra, dizendo que o robô está preso |

O `RECOLHE` inicial não é decoração: o braço começa na postura em que o modelo
do youBot foi salvo, dobrado para trás, com a ponta do lado **oposto** ao eixo
da junta 0. Sem recolhê-lo primeiro, o caminho até o primeiro alvo passa por
q0 = 180°, fora do limite de ±169° da junta.

---

## 4. As medidas, e de onde elas vêm

Nada aqui é número de catálogo. Tudo saiu de medir na cena, com os scripts de
`scene/`, e os valores medidos estão nos comentários de `kinematics.py`.

### 4.1 O braço

| Medida | Valor |
|---|---|
| Eixo da junta 0, à frente do centro da base | 0,166 m |
| Elos (ombro → cotovelo → pulso → ponta) | 0,155 / 0,135 / 0,181 m |
| Alcance total | 0,50 m |
| Envelope de preensão por cima, no chão | **0,15 a 0,32 m** do eixo da junta 0 |
| Erro da cinemática direta contra a cena | **0,99 mm** (pior de 25 configurações) |
| Erro da inversa (alvo pedido vs. garra medida) | **0,63 mm** |

O envelope é estreito, e é ele que manda na tarefa: a base tem de parar a
**0,40 m** da banana, e o raio de preensão usado (0,233 m) é o **meio** do
envelope, não a ponta — deixar margem para o erro da base é o que faz a
preensão funcionar apesar do escorregamento das rodas.

Dois achados sobre os limites das juntas, que estão nos testes:

- A inversa de dois elos tem sempre duas soluções (cotovelo para cima e para
  baixo), mas o intervalo **assimétrico** da junta 1 (`[-1,571, +1,309]`) elimina
  uma delas em todo o envelope. A escolha do cotovelo não é livre: os limites
  decidem.
- A preensão **lateral** não existe neste braço nas posições de interesse. A
  0,25 m do eixo e 0,12 m de altura, só de 150° a 180° de inclinação há
  solução — por isso a tarefa é por cima.

### 4.2 A base

Comandando cada liberdade isoladamente (`scene/calibrate_base.py`):

| Ensaio | Pedido | Medido | Ganho |
|---|---|---|---|
| frente | +0,200 m/s | +0,128 | 0,640 |
| ré | −0,200 m/s | −0,127 | 0,634 |
| esquerda | +0,200 m/s | +0,126 | 0,632 |
| direita | −0,200 m/s | −0,126 | 0,632 |
| giro + | +0,600 rad/s | +0,390 | 0,651 |
| giro − | −0,600 rad/s | −0,390 | 0,650 |
| diagonal | (0,2; 0,2) | (0,071; 0,071) | 0,355 |

Acoplamento cruzado — movimento nas liberdades que **não** foram pedidas —
sempre abaixo de **0,6 %**: o modelo está certo no que importa.

O ganho de 0,64 é **escorregamento dos rolos**, não erro de modelo. A ponte o
compensa (`cmd_gain = 1/0,64 = 1,56`), de modo que "0,1 m/s" signifique 0,1 m/s
e os ganhos do controlador queiram dizer o que dizem. Na diagonal o ganho cai
para 0,36 — ali duas rodas ficam paradas enquanto duas giram — e é por isso que
o controle é em malha fechada e não por tempo de marcha.

O motor de cada roda tem **2,5 N·m**: pedindo 8,0 rad/s a roda faz 5,4. Acima
de ~0,25 m/s na roda o comando deixa de virar movimento, e é esse o teto que o
`base_control` usa.

### 4.3 A garra

| Medida | Valor |
|---|---|
| Vão útil entre as faces dos dedos | **26 mm** (fechada) a **76 mm** (aberta) |
| Largura da banana original | 100 mm — **não cabe** |
| Largura da banana a 0,5× | 50 mm |

É por isso que a cena reduz a banana (`--banana-scale`, 0,5 por padrão). É uma
mudança de cena, não de algoritmo, e está registrada porque muda o que se vê
rodando.

A garra detecta a preensão **sem sensor de força**: fechando no vazio os dedos
vão até os 26 mm; com algo entre eles, o motor trava antes. Medido numa banana
de 50 mm, ele trava com 55 mm entre as faces — e é essa parada acima do vão
vazio que diz que pegou.

---

## 5. Os cinco erros que o projeto deu, e o que cada um ensinou

Estão aqui porque todos são reproduzíveis e nenhum é óbvio.

### 5.1 O robô caía pelo chão a −27 m

Forçar `setObjectOrientation(robot, [0, 0, 0])` para alinhar o youBot. O frame
do objeto `/youBot` **nasce girado** em (−90°, 0°, −90°): zerar a orientação põe
o robô de cabeça para baixo, e a física o expele pelo chão. O braço obedecia
perfeitamente aos comandos enquanto despencava, o que fazia o sintoma parecer
coisa da cinemática.

Para girar o robô no plano, compõe-se com a orientação natural
(`sim.rotateAroundAxis` sobre a **pose**, que é um utilitário de matemática e
não aceita handle), com pivô no `youBot_ref`.

### 5.2 O movimento lateral desaparecia

Desabilitar **todos** os scripts da árvore do robô, para que não disputassem as
juntas. Mas o modelo tem um script por roda que a cada passo reposiciona a junta
de escorregamento em −45° — *"important to achieve the desired omni-wheel
effect"*, diz o comentário deles. São eles que **fazem** a base ser
omnidirecional.

O critério certo é o comando de junta: calar só quem chama
`setJointTargetVelocity` ou `setJointTargetPosition` (a demonstração do modelo e
o script da garra). Os das rodas usam `setObjectOrientation` e seguem vivos.
Está em `coppelia_scene.silence_joint_scripts`, com uma definição só, usada pela
ponte e pelos scripts de `scene/`.

### 5.3 A garra expelia a banana a 30 cm

Fechar as duas juntas da garra por **posição** até o fim. A garra fecha até
26 mm sobre uma banana de 50 mm: ela esmaga, a força de contato acumula e o
objeto é cuspido pela cena.

O mecanismo certo é o do próprio modelo: a junta 2 em **velocidade** (ela
**trava** contra o objeto e segura com a força limitada) e a junta 1 seguindo em
posição, com razão −0,5, que é o acoplamento mecânico dos dedos. Com isso os
dedos param em contato estável e a banana fica presa.

### 5.4 A banana escapava no levantamento

Mandar a configuração final da inversa de uma vez. Cada junta corre para o alvo
na velocidade que consegue — e as do modelo vão a 6,28 rad/s —, a garra
descreve uma curva qualquer e o tranco vence o atrito dos dedos.

A correção é a de qualquer manipulador: **interpolar**. A ponta anda em linha
reta, a 0,08 m/s, com a inversa resolvida a cada passo.

### 5.5 Não havia caminho até a bandeja

O mapa infla cada obstáculo pelo raio circunscrito do robô (0,36 m). A base
precisa parar a ~0,43 m do centro da bandeja para o braço alcançar — e essa
célula está bloqueada pela inflação da própria bandeja.

O conflito é de fundo, e vale o parágrafo no relatório: **com inflação
circular, um robô cujo raio (0,36 m) é maior que o alcance do braço (0,32 m)
nunca pode estender o braço sobre coisa alguma**. Duas correções, as duas
geométricas:

1. Soltar a banana na **borda próxima** da bandeja, não no centro. Qualquer
   ponto dentro dela serve, e o mais próximo pede só 0,23 m de alcance.
2. Para a pose de depósito, inflar a bandeja pelo **meio-comprimento** do robô
   (0,293 m) em vez do raio circunscrito. A pose final é, por construção, de
   frente para a bandeja, e a folga que importa nessa direção é a da frente do
   robô. Não é licença: é usar a medida certa para a geometria certa.

---

## 6. Por que a banana tem um corpo de colisão invisível

A malha da banana tem 1015 triângulos e é **não convexa**. Marcada como
respondable assim, o motor de física produz contatos errados: a garra empurrava
a banana em vez de envolvê-la.

A solução é a canônica do CoppeliaSim: um **cuboide convexo** do tamanho da
banana faz o corpo dinâmico, e a malha vira filha **estática** e não
respondable, só para ser vista. (Filha não estática cai sozinha, como corpo
livre — na primeira tentativa a malha despencou a −1589 m enquanto o cuboide
ficava no lugar.)

Está em `scene/build_scene.py:add_banana`.

---

## 7. Tópicos

Todos no namespace `youbot`.

| Tópico | Tipo | De → Para |
|---|---|---|
| `pose` | `geometry_msgs/PoseStamped` | ponte → todos |
| `sim_time` | `std_msgs/Float64` | ponte → todos (os prazos são medidos NELE) |
| `bananas` | `geometry_msgs/PoseArray` | ponte → missão |
| `banana_info` | `std_msgs/String` (JSON) | ponte → missão (yaw e tamanho de cada uma) |
| `poops` | `geometry_msgs/PoseArray` | ponte → missão |
| `deposit` | `std_msgs/String` (JSON) | ponte → missão |
| `arm_state` | `sensor_msgs/JointState` | ponte → braço |
| `gripper_gap` | `std_msgs/Float64` | ponte → braço |
| `goal` | `geometry_msgs/PoseStamped` | missão → base |
| `goal_reached` | `std_msgs/Bool` | base → missão |
| `arm_goal` | `geometry_msgs/PoseStamped` | missão → braço |
| `arm_posture` | `std_msgs/String` | missão → braço |
| `gripper` | `std_msgs/Float64` | missão → braço |
| `arm_result` | `std_msgs/String` (JSON) | braço → missão |
| `grasped` | `std_msgs/Bool` | braço → missão |
| `cmd_vel` | `geometry_msgs/Twist` | base → ponte (as **três** componentes) |
| `arm_joints` | `std_msgs/Float64MultiArray` | braço → ponte |
| `grid`, `path` | `OccupancyGrid`, `Path` | missão → rviz |
| `done` | `std_msgs/Bool` | missão → ponte |

O `arm_goal` merece nota: a **posição** é onde a ponta da garra tem de ficar, no
frame da base; a **orientação** não é a da garra, é o azimute do eixo maior do
objeto a pegar. Quem calcula o giro da garra é `kinematics.grasp_roll` — a
missão sabe como a banana está deitada, mas não tem por que saber como a garra
precisa girar para pegá-la.

---

## 8. Os dois referenciais

A cena e o ROS discordam sobre qual eixo é a frente do robô:

```
youBot_ref (a cena)   frente = +y,  esquerda = -x
ROS (REP 103)         frente = +x,  esquerda = +y
```

A ponte publica a pose já na convenção do ROS, como a do exercício anterior
fazia, e é nela que o planejador e os controladores trabalham. Mas a cinemática
do braço é escrita no frame da cena, porque é lá que as juntas foram medidas.
Todo ponto que atravessa a fronteira passa por `kinematics.ros_to_ref` /
`ref_to_ros` — uma função com nome, usada num só lugar, em vez de um sinal
trocado espalhado pelo código.
