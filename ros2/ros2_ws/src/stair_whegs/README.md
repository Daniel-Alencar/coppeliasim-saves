# stair_whegs

Um rover de **whegs** — rodas de três raios curvos — que acha uma escada, se
alinha com ela, sobe os seis degraus e para no patamar antes de cair do outro
lado. É o trabalho descrito em [requirements.md](requirements.md), com a cena
[coppeliasim/stair_whegs.ttt](coppeliasim/stair_whegs.ttt).

O robô não existe no CoppeliaSim: ele é **construído por script**, e por isso a
geometria dele é variável de experimento.

```
                     probes, attitude, pose, sim_time
CoppeliaSim ──ZeroMQ──▶ coppelia_bridge ────────────────▶ climber
     ▲                        ▲                             │ drive, gait
     │ juntas                 │ wheg_joints                 ▼
     └────────────────────────┴───────────────────────── gait
```

| Executável | Papel |
|---|---|
| `coppelia_bridge` | Só comunicação: publica pose, atitude, estado das juntas e as três sondas; aplica as posições de wheg |
| `gait` | Gerador de marcha: converte velocidade + padrão de fases em posição de cada wheg |
| `climber` | A percepção e a decisão: procura o degrau, alinha, sobe, detecta o topo e para |

E a matemática, em Python puro e testável sem simulador:

| Módulo | Conteúdo |
|---|---|
| `geometry.py` | Geometria do wheg e da escada; fases, padrões de marcha, troca de marcha sem salto |

---

## 1. O que é um wheg, e por que ele sobe

Uma roda de raios: em vez do aro contínuo, três raios curvos saindo do cubo.
No plano ela rola como uma roda poligonal; contra um degrau, cada raio funciona
como perna e **engancha na quina**.

É por isso que ela sobe e a roda não: a roda precisaria de atrito na face
vertical do espelho, que não existe, enquanto o raio se apoia na aresta.

O preço é o balanço. Entre um raio e o seguinte, o centro do wheg passa do
apoio na ponta (altura R) ao apoio no vão entre dois raios (R·cos 60°): o corpo
sobe e desce **25 mm a cada passo** e inclina ±15°. Esse balanço não é um
detalhe estético — é o que domina o projeto dos sensores (§4).

---

## 2. Como rodar

### 2.1 Pré-requisitos

- ROS 2 Jazzy.
- No Python do sistema (o `ros2 run` não usa o Anaconda nem um `.venv/`):

```bash
/usr/bin/python3 -m pip install --user --break-system-packages coppeliasim-zmqremoteapi-client
```

- CoppeliaSim aberto. `COPPELIASIM_ROOT` aponta a instalação se ela não
  estiver em `~/Softwares/CoppeliaSim/CoppeliaSim_Edu`.

### 2.2 Compilar e montar a cena

```bash
cd ros2/ros2_ws
colcon build --symlink-install --packages-select stair_whegs
source install/setup.zsh          # setup.bash se a sua shell for bash

cd src/stair_whegs
python3 scene/build_scene.py                     # rover e escada padrão
python3 scene/build_scene.py --riser 0.06        # degrau difícil
python3 scene/build_scene.py --wheg-radius 0.07 --spokes 4
python3 scene/build_scene.py --no-tail           # para ver o capotamento
```

### 2.3 Rodar

```bash
ros2 launch stair_whegs stair_whegs.launch.py
ros2 launch stair_whegs stair_whegs.launch.py gait:=junta
ros2 launch stair_whegs stair_whegs.launch.py climb_speed:=2.0
```

### 2.4 Medir

```bash
python3 scene/characterize.py espelho     # até que degrau sobe
python3 scene/characterize.py raio        # efeito do raio do wheg
python3 scene/characterize.py marcha      # as três marchas
python3 scene/characterize.py robustez    # taxa de sucesso, variando a fase
python3 scene/characterize.py cauda
python3 scene/characterize.py raios
pytest test/                              # a matemática da marcha
```

---

## 3. A marcha

O que define uma marcha não é a velocidade: é a **fase** de cada perna.

```
θ_i(t) = φ_i + (ω ± Δ/2)·t
```

`φ_i` é a fase da marcha, `ω` a velocidade e `Δ` a diferença entre os lados,
que faz o rover virar por **derrapagem** — um wheg não esterça nem rola de
lado, então derrapar é a única forma de curvar.

As fases são dadas em frações de um **passo** (o ângulo entre dois raios,
120° para três raios), e não de uma volta: assim os padrões não dependem do
número de raios.

| Marcha | Fases | Maior vão sem apoio |
|---|---|---|
| `alternada` | diagonais em oposição | 50 % do passo |
| `dianteira` | frente contra trás | 50 % do passo |
| `junta` | os quatro em fase | 100 % do passo |

### Posição, e não velocidade

As juntas são comandadas em **posição**, com o alvo andando no tempo. Parece um
rodeio — mandar velocidade seria mais direto —, mas foi medido nesta cena:

| Modo | Deriva de fase após 14 s de escalada | Tempo até o topo |
|---|---|---|
| velocidade | **−8°**, e crescendo | ~10 s |
| posição | **0°** | ~9 s |

Cada wheg encontra uma carga diferente — um está enganchando a quina enquanto
o outro gira no ar — e, em malha de velocidade, quem está sob carga fica para
trás e não recupera. Fase coordenada é o ponto inteiro de uma marcha; deixá-la
derivar é desfazer a marcha aos poucos.

---

## 4. Os sensores, e a primeira montagem que não funcionou

A montagem óbvia foi a primeira: raios **horizontais** na frente do chassi,
para "ver o degrau". Ela não funciona neste robô, e a medição mostra por quê —
o corpo sobe e desce 25 mm e inclina ±15° a cada passo, contra um espelho de
40 mm. O raio passa metade do tempo enfiado no chão e a outra metade por cima
do degrau. Medido: ele acusava o **piso** em 50 % a 100 % das leituras, com a
distância governada pelo balanço e não pelo degrau.

Os três sensores da versão final apontam **para baixo e para a frente**, e o
que eles informam não é "detectei", é a **distância**:

| Sensor | Inclinação | O que resolve |
|---|---|---|
| `probe_l`, `probe_r` | 30° | Degrau à frente (a leitura **encurta**) e desalinhamento (a **diferença** entre os dois lados) |
| `edge` | 55° | A beirada do patamar: o raio mergulha no vazio e a leitura estoura |

E toda decisão é tomada sobre a **mediana de uma janela de ~um passo**. Mediana
e não média, porque as leituras perdidas aparecem como o alcance máximo do
sensor e puxariam uma média para longe.

### O limiar é medido, não escrito

O nó `climber` começa num estado `CALIBRA`: parado no plano, mede quanto as
sondas leem. A geometria prevê 138 mm; o valor medido é **207 mm**, e a
diferença é o balanço do corpo. O limiar de "há degrau" é uma **fração** dessa
referência medida, de modo que o mesmo código serve a um rover de outra altura
ou com os sensores em outro ângulo.

---

## 5. A máquina de estados

```
CALIBRA ──> PROCURA ──> ALINHA ──> SOBE ──> TOPO ──> FIM
               ^                     │
               └──── RECUA ◀─────────┘   (empacou no degrau)
```

| Estado | O que faz |
|---|---|
| `CALIBRA` | Parado, mede a leitura das sondas sobre piso plano |
| `PROCURA` | Anda até as sondas encurtarem |
| `ALINHA` | Gira no lugar até ficar de frente para a face do degrau |
| `SOBE` | Sobe, vigiando ganho de altura e o fim da escada |
| `RECUA` | Empacou: afasta e entrega o degrau a outra fase da marcha |
| `TOPO` | Anda no patamar até o `edge` ver o vazio, e para |

Uma corrida completa, do log:

```
piso plano: sondas a 207 mm (geometria diria 138 mm; a diferença é o balanço)
-> procura
degrau à frente: sondas a 145 mm contra 207 mm do piso plano
-> alinha
alinhado a +2.2° da face do degrau
-> sobe
topo: subiu 25.3 cm, pitch de volta a +6.5°
-> topo
beirada à frente (350 mm, alcance 350 mm): parando
SUCESSO: subiu 24.4 cm, parou em x=+1.47 m, z=+0.27 m
```

De olhos fechados — girando os whegs e mais nada — o rover sobe os mesmos seis
degraus, **atravessa o patamar e cai do outro lado**. Foi o que os primeiros
ensaios registraram, com o corpo indo a z = −250 m. É isso que a camada de
percepção acrescenta.

---

## 6. O que os ensaios mediram

### 6.1 Altura máxima de degrau

Com whegs de 50 mm (`scene/characterize.py espelho`):

| Espelho | Razão com o raio | Chegou ao topo |
|---|---|---|
| 20 a 60 mm | 0,4 a 1,2 | sim |
| 62 mm | 1,24 | sim |
| **65 mm** | **1,30** | **sim** |
| 68 mm | 1,36 | não |
| 70 mm | 1,40 | não (não sai do lugar) |

A estimativa de livro é **um raio**: a ponta tem de alcançar acima da quina.
O rover faz **1,3 raio**, e o que compra os 30 % a mais é a **curvatura** do
raio — a garra alcança a quina de um ponto em que um raio reto ainda estaria
batendo na face do espelho. A previsão errou para baixo, e o projeto ficou com
o número medido (`geometry.CLIMB_RATIO`).

### 6.2 Marcha: a resposta depende da dificuldade

No degrau fácil de 40 mm (0,8 raio), **as três marchas sobem** — e a `junta`,
que é a "pior" pelo critério geométrico, é a que menos inclina o corpo
(28° contra 42° da `alternada`).

No degrau difícil de 60 mm (1,2 raio), a ordem se separa — e se **inverte** em
relação ao que a intuição de marcha sugere:

| Marcha | Degraus subidos | Chegou ao topo |
|---|---|---|
| `alternada` | 0,8 | não |
| `dianteira` | 6,3 | sim |
| `junta` | 6,2 | sim |

A leitura: a marcha alternada é feita para **andar** — ela mantém apoio
contínuo e o corpo estável. Subir um degrau alto não é andar, é um **evento de
força**: com os quatro whegs em fase, as quatro garras atacam a quina ao mesmo
tempo e somam empuxo, enquanto a alternada ataca com metade das pernas.

É o resultado mais interessante do trabalho, e contraria a premissa com que ele
começou — a de que a marcha alternada seria a marcha de escalada. Vale a
discussão no relatório: a marcha certa depende do terreno, e é por isso que
trocar de marcha em movimento (sem salto nas juntas) é uma função do
`gait`, e não um detalhe.

### 6.3 Robustez

Um ensaio isolado aqui é **determinístico**: mesma cena, mesmas condições
iniciais, mesmo resultado. Repeti-lo não mede robustez nenhuma. O que muda o
resultado é a **fase em que o rover encontra o primeiro degrau**, e essa fase
depende de onde ele partiu — daí o experimento `robustez`, que espalha a
posição de partida por um passo de marcha e devolve uma taxa.

---

## 7. Decisões de cena que custaram depuração

### 7.1 O IMU é um dummy, não o chassi

O chassi é montado com `sim.groupShapes` (corpo + cauda), e o CoppeliaSim
realinha o frame do shape agrupado com os eixos principais de inércia: com a
cauda, ele nasce **20° inclinado** em relação ao mundo. Ler a orientação dele
daria um rover permanentemente empinado.

O dummy `imu` é criado depois, alinhado ao mundo, e preso ao chassi — é o que
um IMU de verdade seria. Parado no plano ele lê (0, 0, 1); o chassi lê
(0,34; 0; 0,94).

### 7.2 O rover não pode ser visto pelos próprios sensores

Os raios dos whegs varrem até 0,125 m à frente do centro — **à frente dos
sensores**. Sem marcar o rover como não-detectável, as sondas acusam obstáculo
a 0,000 m o tempo todo, que foi o que os primeiros ensaios mediram.

### 7.3 O patamar não é decoração

Sem ele, "chegou ao topo" e "empinou na beirada" dão a mesma altura. A primeira
rodada de ensaios contou 8 degraus numa escada de 6 — era o rover de pé na
quina. O critério de sucesso passou a ser **chegar ao patamar nivelado**, e não
ganhar altura.

### 7.4 Cada raio é feito de peças convexas

Quatro cuboides ao longo de um arco, agrupados. Uma malha curva única seria
côncava, e o motor de física produz contatos errados com shapes côncavos —
o mesmo problema que a banana do trabalho anterior deu, e pela mesma razão.

---

## 8. Tópicos

Todos no namespace `whegs`.

| Tópico | Tipo | De → Para |
|---|---|---|
| `pose` | `geometry_msgs/PoseStamped` | ponte → climber |
| `attitude` | `geometry_msgs/Vector3Stamped` | ponte → climber (roll, pitch, yaw do IMU) |
| `wheg_state` | `sensor_msgs/JointState` | ponte → gait |
| `probes` | `std_msgs/Float64MultiArray` | ponte → climber (probe_l, probe_r, edge) |
| `sim_time` | `std_msgs/Float64` | ponte → todos |
| `drive` | `geometry_msgs/Twist` | climber → gait |
| `gait` | `std_msgs/String` | climber → gait |
| `wheg_joints` | `std_msgs/Float64MultiArray` | gait → ponte |
| `status` | `std_msgs/String` | climber → quem quiser |
| `done` | `std_msgs/Bool` | climber → ponte |

Duas convenções que valem nota:

- **`drive.linear.x` é positivo para a frente.** As juntas da cena estão
  montadas de modo que girar no sentido negativo leva o rover adiante; essa
  inversão fica contida no nó `gait`, num lugar só.
- **`probes` publica o alcance máximo quando nada é detectado.** É a leitura
  fisicamente correta ("mais longe do que eu enxergo") e evita que o consumidor
  tenha de tratar um valor especial: na beirada do patamar, "nada" e "longe"
  querem dizer a mesma coisa.
