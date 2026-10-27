# stair_whegs

Um rover de **rodas transformáveis** que acha uma escada, se alinha com ela,
sobe os seis degraus e para no patamar antes de cair do outro lado. É o
trabalho descrito em [requirements.md](requirements.md), com a cena
[coppeliasim/stair_whegs.ttt](coppeliasim/stair_whegs.ttt).

O robô não existe no CoppeliaSim: ele é **construído por script**, e por isso a
geometria dele é variável de experimento.

```
                     probes, attitude, pose, sim_time
CoppeliaSim ──ZeroMQ──▶ coppelia_bridge ────────────────▶ climber
     ▲                     ▲      ▲                         │ drive, gait
     │ juntas              │      └──── wheel_mode ─────────┤
     │                     │ wheg_joints                    ▼
     └─────────────────────┴───────────────────────────── gait
```

| Executável | Papel |
|---|---|
| `coppelia_bridge` | Só comunicação: publica pose, atitude, estado das juntas e as três sondas; aplica as posições de tração e o estado das rodas |
| `gait` | Gerador de marcha: converte velocidade + padrão de fases em posição de cada roda |
| `climber` | A percepção e a decisão: procura o degrau, alinha, **abre a garra**, sobe, detecta o topo e para |

E a matemática, em Python puro e testável sem simulador:

| Módulo | Conteúdo |
|---|---|
| `geometry.py` | Geometria da roda transformável e da escada; fases, padrões de marcha, troca de marcha sem salto |

---

## 1. A roda transformável

O rover usa a roda do **Wheel Transformer** (Kim, Jung, Kim, Cho e Chu, *IEEE
Transactions on Robotics* 30(6), 2014; patente US 9 073 587). Ela tem dois
estados e nenhum meio-termo:

```
   FECHADA ("roda")              ABERTA ("garra")

      ___                           \     /
    /     \      três dorsos          \__/        as mesmas três pernas,
   |       |     emendam e            /    \       giradas 30° sobre o aro
    \ ___ /      fecham o aro       _/      \_
```

| | fechada | aberta |
|---|---|---|
| alcance do contato | 50 mm = 1 raio | **65 mm = 1,30 raios** |
| balanço do corpo por passo | **0 mm** | 32 mm (metade do alcance) |
| tombamento lateral | 61° | 54° |
| cauda pega aos | 4° de pitch | 10° de pitch |
| serve para | andar rápido e liso | **enganchar a quina** |

O eixo de cada perna fica **sobre o aro**, a 0,86 do raio, e não no cubo — é o
que distingue esta roda de um wheg de raios fixos. Fechada, o dorso de cada
perna é o arco de 120° que vai do próprio eixo até o eixo da perna seguinte:
os três emendam e fecham a circunferência. Abrindo, cada perna gira em torno de
um ponto que **não é o centro da roda**, e por isso a ponta se afasta muito
mais depressa do que o ângulo de abertura sugere (`geometry.claw_tip`).

### 1.1 A abertura não é "o máximo possível"

`claw_reach` é monótona: cada grau a mais de abertura alcança mais longe. Pelo
alcance, então, a perna deveria abrir o máximo — e foi assim que esta roda
nasceu, com 80° de abertura e 98 mm de garra. **Não sobe.** O rover empina até
90° no primeiro degrau e capota, e não há bitola que o salve: 150, 180 e
220 mm capotam igual, nas duas marchas.

O que o ensaio mostrou é que o alcance da garra não é só alcance — é também **a
altura em que o rover anda** quando está aberto, e três coisas pioram com ela
ao mesmo tempo:

| Cresce com a abertura | Consequência | Onde está no código |
|---|---|---|
| o braço que empina o rover na quina | o *wheelie*, contra um entre-eixos que não mudou | `WHEELBASE` |
| a altura do centro de massa | o ângulo de tombamento lateral cai | `TRACK` |
| a distância entre as duas alturas de marcha | a cauda rígida pega o chão tarde demais | `tail_catch_angle` |

A abertura final — 30°, que dá 65 mm de garra — não sai de maximizar nada. Sai
de manter o rover **aberto** dentro da mesma faixa de proporções em que o wheg
de raios fixos já funcionava: razão garra/entre-eixos de 0,32 (era 0,65 com a
garra de 98 mm) e cauda pegando aos 10° de pitch (eram 44°). 65 mm ainda são
1,6 vez o degrau da cena, com folga de sobra.

O que isso compra é não ter de escolher. Um wheg de raios fixos sobe degrau,
porque se apoia na **quina** em vez de atritar na face vertical do espelho —
mas balança o tempo todo, inclusive no plano, onde não precisaria. Uma roda
lisa anda liso e para antes, porque o aro só alcança a quina até mais ou menos
um raio de altura. A roda transformável é roda quando convém e garra quando
precisa, e o §6.1 mostra onde as duas se separam.

### 1.2 No robô original a troca é passiva

No Wheel Transformer não há motor de transformação. Uma das três pernas é a
**perna gatilho**: quando a ponta dela esfrega num obstáculo, o atrito a faz
girar, e um **disco transmissor** acoplado leva as outras duas junto — as três
abrem em bloco. Uma mola de torção fecha tudo de volta quando o atrito some.

Aqui a transformação é **comandada**, e as duas simplificações estão escritas
no código:

| No mecanismo | Aqui |
|---|---|
| disco transmissor acopla as três pernas | a ponte escreve o mesmo alvo nas três juntas (`coppelia_bridge.actuate_legs`) |
| atrito da perna gatilho dispara | a sonda do `climber` dispara (`climber.WHEEL_FOR`) |
| pernas fechadas se apoiam umas nas outras | **batente de junta** em zero — este continua mecânico |

Simular o disco e o atrito do gatilho daria um mecanismo instável no motor de
física e não acrescentaria nada ao que este trabalho mede. O que **não** foi
simplificado é o batente: com a roda fechada o peso do rover vai para o limite
da junta, e não para o motor. Isso não é escolha de implementação, é
geometria — o contato no chão fica sempre adiante do eixo da perna, no sentido
em que ela fecha, e por isso a roda fechada **se tranca sozinha**.

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
python3 scene/build_scene.py --leg-open 50       # garra que abre menos
python3 scene/build_scene.py --no-tail           # para ver o capotamento
```

`--leg-open` é o ângulo de abertura da perna, em graus, e é o parâmetro que
governa o compromisso inteiro da roda: mais abertura alcança degrau mais alto
e balança mais (o balanço é metade do alcance). O `build_scene` imprime os dois
números para a combinação pedida.

### 2.3 Ver os dois estados sem subir escada nenhuma

```bash
ros2 topic pub -1 /whegs/wheel_mode std_msgs/String '{data: garra}'
ros2 topic pub -1 /whegs/wheel_mode std_msgs/String '{data: roda}'
```

### 2.4 Rodar

```bash
ros2 launch stair_whegs stair_whegs.launch.py
ros2 launch stair_whegs stair_whegs.launch.py climb_gait:=alternada  # capota
ros2 launch stair_whegs stair_whegs.launch.py climb_speed:=2.0
```

### 2.5 Medir

```bash
python3 scene/characterize.py roda        # fechada x aberta, a mesma escada
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

As fases são dadas em frações de um **passo** (o ângulo entre duas pernas,
120° para três pernas), e não de uma volta: assim os padrões não dependem do
número de pernas.

| Marcha | Fases | Maior vão sem apoio |
|---|---|---|
| `alternada` | diagonais em oposição | 50 % do passo |
| `dianteira` | frente contra trás | 50 % do passo |
| `junta` | os quatro em fase | 100 % do passo |

### A marcha só existe com a garra aberta

Com a roda **fechada** as fases não querem dizer nada: um aro é contínuo, não
tem passo, e girar duas rodas defasadas de 60° dá exatamente o mesmo andar que
girá-las em fase. A coordenação de fases é um conceito de **pernas**, e as
pernas só aparecem quando a roda abre. Vale a pena dizer porque é fácil olhar
`gait.py` e achar que ele governa o rover inteiro: no plano ele está girando
rodas, e o padrão que ele mantém só passa a ter efeito quando o `climber` abre
a garra.

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
com a garra aberta o corpo sobe e desce 32 mm e inclina ±14° em rolagem a cada
passo, contra um espelho de 40 mm. O raio passa metade do tempo enfiado no chão e a outra metade por cima
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
sondas leem. O limiar de "há degrau" é uma **fração** dessa referência medida,
de modo que o mesmo código serve a um rover de outra altura ou com os sensores
em outro ângulo.

E aqui a roda transformável pagou um dividendo que não estava no plano. A
calibração acontece com a roda **fechada**, que não balança:

| | geometria prevê | medido | erro |
|---|---|---|---|
| wheg de raios fixos | 138 mm | 207 mm | +69 mm |
| roda transformável, fechada | 138 mm | **144 mm** | **+6 mm** |

Os 69 mm da versão antiga eram o balanço do corpo contaminando a leitura: a
sonda media o sobe-e-desce da marcha, não o terreno. Com o aro fechado não há
sobe-e-desce, e a referência medida cai praticamente em cima da geométrica. A
calibração continua valendo a pena — ela é que absorve a altura do chassi e o
ângulo do sensor — mas deixou de ter de separar dois sinais misturados.

---

## 5. A máquina de estados

```
CALIBRA ──> PROCURA ──> ALINHA ──> SOBE ──> TOPO ──> FIM
 roda        roda        roda      GARRA    roda
               ^                     │
               └──── RECUA ◀─────────┘   (empacou no degrau)
                     garra
```

A linha de baixo é o estado da roda, e **este nó é quem o escolhe**: é ele que
faz o papel da perna gatilho do mecanismo original. O mesmo sinal de sonda que
já dizia "há degrau à frente" passa a dizer também "abre a garra".

| Estado | Roda | O que faz |
|---|---|---|
| `CALIBRA` | fechada | Parado, mede a leitura das sondas sobre piso plano |
| `PROCURA` | fechada | Anda até as sondas encurtarem |
| `ALINHA` | fechada | Gira no lugar até ficar de frente para a face do degrau |
| `SOBE` | **garra** | Abre, espera `transform_time`, sobe vigiando ganho de altura |
| `RECUA` | garra | Empacou: afasta e entrega o degrau a outra fase da marcha |
| `TOPO` | fechada | Fecha, e anda no patamar até o `edge` ver o vazio |

Os três primeiros estados acontecem com a roda fechada de propósito, e isso
melhora a percepção de graça: **fechada o corpo não balança**, e a calibração
das sondas (§4) deixa de ter de separar o degrau do sobe-e-desce da marcha.

A transformação tem um tempo próprio (`transform_time`, 0,8 s de simulação) em
que o rover fica **parado**. Abrir levanta o corpo e o empurra para trás (§7.7);
fazer isso girando, encostado na face do espelho, prende a ponta de uma perna
debaixo da quina.

Uma corrida completa, do log do launch:

```
piso plano: sondas a 144 mm (geometria diria 138 mm; a diferença é o balanço
            do corpo). Degrau será acusado abaixo de 101 mm
-> procura
degrau à frente: sondas a 101 mm contra 144 mm do piso plano
-> alinha
alinhado a -2.1° da face do degrau
-> sobe  (roda garra: a garra alcança 65 mm contra 50 mm do aro)
marcha -> junta
roda -> garra (+30° por perna)
topo: subiu 25.0 cm, pitch de volta a +5.9°
-> topo  (roda roda)
roda -> roda (+0° por perna)
beirada à frente (350 mm, alcance 350 mm): parando
SUCESSO: subiu 24.4 cm, parou em x=+1.43 m, z=+0.28 m
```

Repare nas três linhas do meio. Elas acontecem no mesmo instante, e são a
cadeia inteira deste trabalho: a **sonda** diz que há degrau, o `climber`
troca a **marcha** para a de escalada e manda **abrir a garra**. No robô
original as duas últimas não existem — a perna gatilho esbarra no obstáculo e
o atrito faz o resto.

De olhos fechados — girando as rodas e mais nada — o rover sobe os mesmos seis
degraus, **atravessa o patamar e cai do outro lado**. Foi o que os primeiros
ensaios registraram, com o corpo indo a z = −250 m. É isso que a camada de
percepção acrescenta.

---

## 6. O que os ensaios mediram

Todos com `scene/characterize.py`, que monta a cena de novo para cada linha e
solta o rover em malha aberta. "Topo" é chegar ao patamar **nivelado** (pitch
< 12°), e não ganhar altura — um rover empinado na beirada também ganha altura.
"Capotou" é rolagem acima de 60°, um veredito que esta versão precisou
acrescentar e a de raios fixos não tinha.

### 6.1 Fechada contra aberta: onde o mecanismo começa a servir

A primeira versão deste ensaio usou um degrau só, o de 40 mm da cena, e deu a
resposta errada: **os dois estados sobem**. Foi uma surpresa útil. A intuição
de que "roda lisa não sobe degrau" vale para uma roda só puxando contra a face
vertical do espelho; este rover tem tração nas quatro, uma cauda, e o aro
fechado se apoia na **quina** como qualquer roda sobe um meio-fio.

O mecanismo não serve para o degrau fácil. Serve para o degrau em que o aro
para — e é por isso que o ensaio varre a altura (`characterize.py roda`,
marcha de escalada nos dois estados):

| Espelho | / aro | Fechada | Aberta |
|---|---|---|---|
| 40 mm | 0,8 | 6,0 degraus, **topo** (11,5 s) | 6,0 degraus, **topo** (7,9 s) |
| 55 mm | 1,1 | 3,6 degraus, não | 6,0 degraus, **topo** |
| 70 mm | 1,4 | −0,3 degrau, não sai do lugar | 6,0 degraus, **topo** |
| 75 mm | 1,5 | — | 0,0 degrau, **capotou** |
| 85 mm | 1,7 | −0,2 degrau, não sai do lugar | 0,4 degrau, não |

Três leituras:

- **O aro fechado para em algum ponto entre 0,8 e 1,1 raio**, que é o que a
  geometria prevê: acima de um raio o aro não alcança mais a quina, e não há
  atrito na face vertical que o segure.
- **A garra vence 70 mm e capota em 75 mm** — 1,4 vez o aro fechado e 1,08 vez
  o próprio alcance (`geometry.CLIMB_RATIO`). É o que a transformação compra, e
  é um número, não uma opinião. Em degrau absoluto ela vai mais longe que o
  wheg de raios fixos do mesmo aro, que parava em 65 mm.
- **No degrau fácil a garra não é melhor, é mais RÁPIDA**: 7,9 s contra 11,5 s,
  porque o raio efetivo é maior e a mesma velocidade angular anda mais. Subir
  com a garra num degrau que o aro vence é gastar balanço à toa — e é
  exatamente por isso que o `climber` só abre quando a sonda acusa o degrau.

### 6.2 A marcha decide o capotamento, não o empuxo

A versão de raios fixos já tinha achado que a marcha `junta` sobe degrau
difícil melhor que a `alternada`, e explicava com **empuxo**: os quatro
atacando a quina ao mesmo tempo somariam força. Com a roda transformável dá
para ver que a explicação era outra, porque agora o modo de falha é visível.

Mesmo rover, mesma escada, só trocando a marcha (`characterize.py bitola`, e o
par A/D dos ensaios de configuração):

| Marcha | Rolagem máxima | Resultado |
|---|---|---|
| `alternada` | **180°** | capotou |
| `junta` | **1°** | subiu os seis degraus |

`alternada` põe os dois lados do rover em fases opostas: enquanto a roda
dianteira esquerda está no alto de uma perna, a direita está no vão entre
duas. Com o aro fechado isso não existe (o aro não tem vão). Com a garra
aberta, medido em piso plano, o corpo balança **±14° em rolagem** o tempo
todo — e no plano não faz diferença nenhuma, porque sobra margem.

Na quina do degrau não sobra. O rover empina, a margem lateral que ele tinha
de pé vai embora com o pitch, e os ±14° terminam o serviço. Com os quatro em
fase os dois lados sobem e descem **juntos**: o balanço vai todo para o pitch,
que a cauda e o entre-eixos aguentam, e some da rolagem, que não tem quem
aguente.

Daí o rover ter duas marchas e trocar entre elas em movimento: `alternada`
para andar no plano, `junta` para atacar o degrau (`geometry.CLIMB_GAIT`,
`climber.do_alinha`). A troca sem salto nas juntas (`geometry.rephase`) deixou
de ser um exercício e virou requisito.

### 6.3 A abertura da perna, e o limite que não é o alcance

Está no §1.1, e é o resultado de projeto mais caro deste trabalho: `claw_reach`
é monótona, então pelo alcance a perna deveria abrir o máximo — e com 80° de
abertura **o rover não sobe degrau nenhum**. Ele empina a 90° e capota, em
qualquer bitola (150, 180 e 220 mm, nas duas marchas) e em qualquer abertura
grande (30°, 45° e 80° capotaram todas, com a bitola e o entre-eixos antigos).

O que a varredura de configuração mostrou é que três quantidades tinham de
voltar à faixa em que o wheg de raios fixos já funcionava, e nenhuma delas é o
alcance:

| | com garra de 98 mm | com garra de 65 mm |
|---|---|---|
| garra / entre-eixos | 0,65 | **0,32** |
| cauda pega aos | 44° de pitch | **10°** |
| tombamento lateral | 51° | **54°** |
| resultado | capota no 1.º degrau | sobe os seis |

E o entre-eixos e a cauda cresceram junto com a garra: 200 mm e 120 mm, contra
150 mm e 90 mm da versão de raios fixos.

### 6.4 Robustez

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

### 7.4 Cada perna é feita de peças convexas

Oito cuboides ao longo de um arco, agrupados. Uma malha curva única seria
côncava, e o motor de física produz contatos errados com shapes côncavos —
o mesmo problema que a banana do trabalho anterior deu, e pela mesma razão.

O número de cuboides é um compromisso que a roda transformável tornou visível:
são `4 rodas x 3 pernas x segmentos` convexos, e cada um é testado a cada passo
de física. Com 8 por perna o rover tem 96 convexos, a flecha de cada corda vale
0,1 mm — o aro fechado é redondo para o que interessa — e a simulação roda em
tempo útil. Com 12 por perna o aro fica 0,04 mm mais redondo e a simulação
fica perceptivelmente mais lenta, sem nada em troca.

### 7.5 O rover não pode colidir consigo mesmo

As três pernas de uma roda emendam quando fechadas. Com a máscara de resposta
a colisão cheia, o motor de física passa a vida resolvendo o contato entre
elas — e o resultado não é "um pouco de ruído": a roda **trava** em vez de
fechar. Todos os shapes do rover nascem com a máscara `0xff00`, só o byte alto,
que é o que o CoppeliaSim usa para dizer "responda a colisões com o mundo, não
com as outras peças da minha própria árvore".

### 7.6 A roda pesa mais do que pareceria

120 g por roda, contra 60 g na versão de raios fixos. Não é estética: o contato
com o chão passa por **uma perna**, e com 60 g de roda essa perna pesava 12 g
contra um corpo de 850 g. Motor de física nenhum trata bem uma razão de 1:70 no
ponto de contato — a peça leve afunda. Com 120 g a perna vai a 24 g e a razão
cai para 1:35. E o número não é implausível no robô real, que carrega ali um
mecanismo inteiro — quadro, três eixos, três pernas — e não um disco maciço.

### 7.7 Abrir a garra empurra o rover para trás

As pernas varrem o chão ao girar para abrir, e o corpo paga a reação. Medido
com a garra de 98 mm da primeira versão, o deslocamento era de **26 cm** — o
suficiente para o rover perder o degrau que acabara de encontrar. Com a garra
final de 65 mm o efeito é da mesma natureza e bem menor, porque as pernas
varrem muito menos chão.

Não é defeito: é a razão de `transform_time` existir e de a transformação
acontecer com o rover **parado**, antes de atacar o degrau. Transformar
girando, encostado na face do espelho, é o jeito de prender a ponta de uma
perna debaixo da quina.

---

## 8. Tópicos

Todos no namespace `whegs`.

| Tópico | Tipo | De → Para |
|---|---|---|
| `pose` | `geometry_msgs/PoseStamped` | ponte → climber |
| `attitude` | `geometry_msgs/Vector3Stamped` | ponte → climber (roll, pitch, yaw do IMU) |
| `wheg_state` | `sensor_msgs/JointState` | ponte → gait (4 juntas de tração + o ângulo de perna de cada roda) |
| `probes` | `std_msgs/Float64MultiArray` | ponte → climber (probe_l, probe_r, edge) |
| `sim_time` | `std_msgs/Float64` | ponte → todos |
| `drive` | `geometry_msgs/Twist` | climber → gait |
| `gait` | `std_msgs/String` | climber → gait |
| `wheg_joints` | `std_msgs/Float64MultiArray` | gait → ponte |
| `wheel_mode` | `std_msgs/String` | climber → ponte (`roda` ou `garra`) |
| `status` | `std_msgs/String` | climber → quem quiser |
| `done` | `std_msgs/Bool` | climber → ponte |

Três convenções que valem nota:

- **`drive.linear.x` é positivo para a frente.** As juntas da cena estão
  montadas de modo que girar no sentido negativo leva o rover adiante; essa
  inversão fica contida no nó `gait`, num lugar só.
- **`probes` publica o alcance máximo quando nada é detectado.** É a leitura
  fisicamente correta ("mais longe do que eu enxergo") e evita que o consumidor
  tenha de tratar um valor especial: na beirada do patamar, "nada" e "longe"
  querem dizer a mesma coisa.
- **`wheel_mode` fala em nomes, não em radianos.** O `climber` sabe que está no
  plano ou atacando um degrau; quanto isso vale em ângulo de perna é assunto da
  geometria da roda, e mora em `geometry.WHEEL_MODES`. O efeito colateral é que
  nenhum nó consegue pedir uma roda **meio aberta** — que no mecanismo real
  também não existe, porque o disco transmissor move as três pernas juntas ou
  não move nenhuma.
