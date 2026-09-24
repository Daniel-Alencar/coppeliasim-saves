# Como ROS 2 e CoppeliaSim conversam neste projeto

Guia do caminho que cada dado percorre no pacote `robot_docking`, do sinal
dentro da simulação até o tópico ROS 2 — e de volta. Escrito para quem vai
implementar o controlador de docking da próxima atividade e precisa saber
exatamente em que ponto encostar.

---

## 1. Três processos, não um

A primeira confusão comum é achar que "o robô" é uma coisa só. São três
programas separados, rodando ao mesmo tempo:

```
┌──────────────────────────┐   ┌──────────────────────┐   ┌────────────────────┐
│      CoppeliaSim         │   │   coppelia_bridge    │   │  SEU controlador   │
│  (processo do simulador) │   │   (processo ROS 2)   │   │  (processo ROS 2)  │
│                          │   │                      │   │                    │
│  scripts da cena:        │   │  o único que fala    │   │  só fala ROS 2.    │
│   battery, beacon,       │◀─▶│  as duas línguas     │◀─▶│  Não sabe que      │
│   odometry, encoder,     │   │                      │   │  existe CoppeliaSim│
│   dockingSensor          │   │                      │   │                    │
└──────────────────────────┘   └──────────────────────┘   └────────────────────┘
        ▲                              ▲                          ▲
        │      ZeroMQ Remote API       │       tópicos ROS 2      │
        └──────── TCP :23000 ──────────┘◀───── (DDS) ────────────┘
```

O ponto central: **o seu controlador da próxima atividade é a caixa da
direita.** Ele não importa `coppeliasim_zmqremoteapi_client`, não conhece
handles, não sabe o que é um "sinal" do CoppeliaSim. Ele assina e publica
tópicos, e pronto. Toda a tradução é trabalho da ponte, que já está feita.

---

## 2. Dois canais diferentes, que não se parecem

| | Tópico ROS 2 | Sinal do CoppeliaSim |
|---|---|---|
| Onde vive | na rede, entre processos ROS | numa tabela global **dentro** do simulador |
| Como funciona | publicador → assinantes, com fila | variável nomeada: alguém escreve, alguém lê |
| Histórico | cada mensagem chega uma vez | só existe o **último** valor escrito |
| Quem avisa | o assinante é chamado quando chega | ninguém: você tem que ir lá perguntar |
| Tipo | `geometry_msgs/Twist`, `std_msgs/Float32`… | float, int ou string |

Sinal do CoppeliaSim **não é** tópico. É mais parecido com uma variável global
compartilhada entre todos os scripts da cena. Por isso a ponte tem um *timer*
que fica perguntando "e agora, quanto está a bateria?" a cada ciclo — não existe
"callback de sinal".

### O prefixo com o handle

Os sinais deste robô têm o **handle** dele colado na frente do nome:

```
84Battery        84Charging        84StrengthSignal
84Docking        84leftVel         84RelativeAngle
```

`84` é o número que o CoppeliaSim deu ao objeto `/myRobot` nesta cena. A
convenção existe para que dois robôs na mesma cena não briguem pelos mesmos
nomes. Os scripts da cena montam esse nome com `str(self.robotHandle)+"Battery"`
(Python) ou `robotHandle..'Battery'` (Lua); a ponte faz o mesmo em
[coppelia_bridge.py:129](robot_docking/coppelia_bridge.py#L129), guardando o
prefixo assim que descobre o handle.

Não decore o `84`: ele muda se a cena mudar. A ponte descobre sozinha e mostra
no log de abertura.

---

## 3. Os seis caminhos, um a um

### 3.1 Bateria — o caminho mais simples

```
/myRobot/battery (script Lua)          a cada 1 s de simulação
   │  sim.setFloatSignal('84Battery', 93.0)
   ▼
[ sinal 84Battery = 93.0 ]             fica lá até alguém sobrescrever
   │
   │  a ponte pergunta a cada ciclo:  sim.getFloatSignal('84Battery')
   ▼
coppelia_bridge.read_battery()         coppelia_bridge.py:301
   │  converte 0–100 % para 0–1 e preenche a BatteryState
   ▼
/myRobot/battery   (sensor_msgs/BatteryState)
```

Esse sinal é **persistente**: ninguém o apaga, então a ponte pode ler quantas
vezes quiser. É o caso fácil — e é a exceção.

### 3.2 Estado de carga — o caso do sinal que some

Aqui a coisa fica esquisita, e é bom entender por quê, porque o código da ponte
parece complicado sem motivo até você ver isto:

```
/chargingBase/beacon                    a cada 1 s, se o robô está na base
   │  sim.setInt32Signal('84Charging', 1)
   ▼
[ sinal 84Charging = 1 ]
   │
   ├──▶ /myRobot/battery LÊ e **APAGA** o sinal, e carrega a bateria
   │
   └──▶ a ponte também quer ler... e muitas vezes chega tarde
```

Dois leitores disputando um sinal que o primeiro apaga. A ponte não tem como
garantir que vai ver o `1`. A saída, em
[read_charging()](robot_docking/coppelia_bridge.py#L305): guardar o último valor
que conseguiu ver **e** conferir com a bateria, que é persistente — se o nível
subiu desde a última leitura, está carregando; se caiu, não está.

> Lição geral: sempre que um script da cena **apaga** um sinal depois de ler,
> esse sinal vira um evento de uso único, e quem chega em segundo lugar não vê
> nada. Vale para `Charging`, `Docking` e `leftVel`/`rightVel`.

### 3.3 Beacon — o sinal que você precisa pedir

O beacon não fica publicando para todo mundo. Ele só calcula a leitura para o
robô cujo handle estiver no sinal global `Beacon`:

```
a ponte, a cada ciclo:  sim.setInt32Signal('Beacon', 84)     ← o pedido
   │
   ▼
/chargingBase/beacon lê 'Beacon', acha o /myRobot/dockingSensor,
mede o feixe do ir_beam e, SE o sensor estiver dentro do feixe:
   │  sim.setFloatSignal('84StrengthSignal', 0.81)
   │  sim.setFloatSignal('84RelativeAngle',  3.13)
   ▼
[ sinais 84StrengthSignal e 84RelativeAngle ]
   │
   │  a ponte lê E APAGA os dois
   ▼
/myRobot/charging_base/strengthSignal   (std_msgs/Float32)
/myRobot/charging_base/relativeAngle    (std_msgs/Float32)
```

Duas sutilezas que a ponte resolve por você, em
[read_beacon()](robot_docking/coppelia_bridge.py#L327):

- **O beacon nunca apaga o que escreveu.** Se a ponte não apagasse depois de
  ler, um robô que saísse do feixe continuaria "vendo" a última leitura para
  sempre. Por isso ela apaga: se passarem `beacon_timeout` segundos sem valor
  novo, ela publica força `0.0` e ângulo `nan` — que é como o seu controlador
  descobre que perdeu o beacon.
- **O nome dos sinais não é o do enunciado.** O enunciado diz
  `signalStrength`/`relativeAngle`; esta cena escreve
  `StrengthSignal`/`RelativeAngle`. A ponte tenta os dois pares.

O `relativeAngle` vem em **radianos**, já normalizado para −π…π: `0` com a base
exatamente à frente, positivo de um lado, negativo do outro. É o número que o
seu controlador vai usar para girar em direção à base.

### 3.4 Modo de docking — ROS 2 escrevendo na cena

Primeiro caminho no sentido inverso:

```
você:  ros2 topic pub /myRobot/docking std_msgs/msg/Bool '{data: true}'
   │
   ▼
coppelia_bridge.docking_callback()      coppelia_bridge.py:284
   │  sim.setInt32Signal('84Docking', 1)
   ▼
[ sinal 84Docking = 1 ]
   │
   ▼
/myRobot/python_controler lê, APAGA o sinal e marca o checkbox "docking"
```

Seja honesto sobre o que isso faz hoje: **nada, além de marcar um checkbox.**
Nenhum script da cena implementa comportamento de docking — procurei em todos os
13. O `python_controler` chega a fazer `sim.broadcastMsg({'id':'dockingMode'})`,
mas ninguém escuta essa mensagem. O comportamento autônomo é justamente o que
*você* vai escrever na próxima atividade, do lado do ROS 2.

Então o checkbox serve para uma coisa só, mas importante: é a **confirmação
visível** de que o comando saiu do ROS 2 e chegou na cena. Para vê-lo, o
`python_controler` precisa estar habilitado (`motor_mode:=signal`).

### 3.5 Velocidade — o caminho com duas variantes

```
seu controlador:  publica Twist em /myRobot/cmd_vel
   │   linear.x = 0.2 m/s      angular.z = 0.5 rad/s
   ▼
coppelia_bridge.cmd_vel_callback()      coppelia_bridge.py:262
   │   cinemática inversa do diferencial:
   │     esquerda = v − w·L/2          direita = v + w·L/2      (m/s na roda)
   │   satura mantendo a proporção e guarda com um prazo de validade
   ▼
coppelia_bridge.apply_motors()          20 Hz, coppelia_bridge.py:368
   │
   ├── motor_mode='joint'   (padrão)
   │      divide pelo raio da roda, aplica motor_sign e chama
   │      sim.setJointTargetVelocity(...)   → a junta, direto
   │
   └── motor_mode='signal'
          sim.setFloatSignal('84leftVel', 0.15)   → o python_controler lê,
          sim.setFloatSignal('84rightVel', 0.25)     divide pelo raio e aplica
```

Três coisas para guardar:

- **O `Twist` é do robô, não das rodas.** `linear.x` em m/s para a frente,
  `angular.z` em rad/s (positivo = virar à esquerda). Quem transforma isso em
  velocidade de roda é a ponte. Seu controlador nunca pensa em rodas.
- **Velocidade negativa anda para a frente nesta cena.** É o `motor_sign=-1.0`.
  Já está tratado; você publica `linear.x` positivo para ir para a frente.
- **O comando expira.** Se passar `cmd_timeout` (0,5 s) sem `cmd_vel` novo, a
  ponte zera os motores. Isso é proposital: se o seu nó travar ou você matar o
  processo, o robô para em vez de sair em linha reta. Na prática, **publique
  continuamente**, não uma vez só.

### 3.6 Odometria — existe, mas não passa pela ponte

O `/myRobot/odometry` calcula `(x, y, theta)` por *dead reckoning* a partir dos
encoders e guarda em `sim.setBufferProperty(robotHandle, "customData.Odometry")`
— uma propriedade do objeto, não um sinal. A ponte **não** publica isso, porque
o enunciado desta atividade não pede. Se o seu controlador precisar, esse é o
lugar de onde tirar.

---

## 4. A tabela que resolve as dúvidas

Quem escreve, quem lê, quem apaga:

| Sinal | Escreve | Lê | Apaga | Persistente? |
|---|---|---|---|---|
| `84Battery` | `/myRobot/battery` | ponte, `python_controler` | ninguém | **sim** |
| `84Charging` | `/chargingBase/beacon` | `/myRobot/battery`, ponte | `/myRobot/battery` | não |
| `84StrengthSignal` | `/chargingBase/beacon` | ponte | **a ponte** | não |
| `84RelativeAngle` | `/chargingBase/beacon` | ponte | **a ponte** | não |
| `Beacon` (global) | **a ponte** | `/chargingBase/beacon` | ninguém | sim |
| `84Docking` | **a ponte** | `python_controler` | `python_controler` | não |
| `84leftVel` / `84rightVel` | **a ponte** (modo signal) | `python_controler` | `python_controler` | não |

---

## 5. Ritmo: por que tudo é mais devagar do que parece

Três relógios diferentes, e vale conhecer os três:

| O quê | Frequência | Onde se ajusta |
|---|---|---|
| Passo da simulação | 20 Hz (50 ms) | propriedade da cena |
| Leitura dos sensores pela ponte | 10 Hz | parâmetro `rate` |
| Escrita nos motores pela ponte | 20 Hz | parâmetro `motor_rate` |
| Bateria e beacon do lado da cena | 1 Hz | fixo nos scripts |

E o limite duro: **com a simulação rodando, cada chamada da Remote API custa
~12 ms** (medido nesta cena). Ela não é atendida na hora — espera a vez entre
passos de simulação. Isso dá um teto de ~80 chamadas por segundo para a ponte
inteira, e é por isso que `rate` e `motor_rate` são baixos. Aumentá-los não
acelera nada: só forma fila.

Consequência prática para o seu controlador: **não adianta publicar `cmd_vel` a
100 Hz.** A ponte só vai conversar com o simulador 20 vezes por segundo. Entre
10 e 20 Hz é o ponto certo.

Uma nota sobre a bateria: ela gasta **1 % por segundo simulado** e começa em
100 %. Você tem cerca de 100 s até o robô parar sozinho. Isso não é bug, é o
problema da atividade — é por isso que existe docking.

---

## 6. O que você vai escrever na próxima atividade

Só ROS 2. Este é o esqueleto, e repare que não há nenhum `import` do
CoppeliaSim:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32


class DockingController(Node):
    def __init__(self):
        super().__init__('docking_controller')

        # O que eu observo
        self.create_subscription(BatteryState, 'battery', self.on_battery, 10)
        self.create_subscription(Float32, 'charging_base/strengthSignal',
                                 self.on_strength, 10)
        self.create_subscription(Float32, 'charging_base/relativeAngle',
                                 self.on_angle, 10)
        self.create_subscription(Bool, 'charging', self.on_charging, 10)

        # O que eu comando
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.dock_pub = self.create_publisher(Bool, 'docking', 10)

        # Estado observado — os callbacks só guardam, quem decide é o timer
        self.battery = None
        self.strength = 0.0
        self.angle = float('nan')
        self.charging = False

        self.create_timer(0.1, self.step)   # 10 Hz: decide e comanda

    def on_battery(self, msg):   self.battery = msg.percentage * 100.0
    def on_strength(self, msg):  self.strength = msg.data
    def on_angle(self, msg):     self.angle = msg.data
    def on_charging(self, msg):  self.charging = msg.data

    def step(self):
        cmd = Twist()
        # ... sua lógica de docking aqui ...
        # if math.isnan(self.angle):  perdi o beacon, procurar girando
        # else:                       alinhar com o ângulo e avançar
        self.cmd_pub.publish(cmd)   # publique SEMPRE: o comando expira em 0,5 s
```

Rode-o no mesmo namespace da ponte, para que os nomes relativos batam:

```bash
ros2 run seu_pacote docking_controller --ros-args -r __ns:=/myRobot
```

Três hábitos que evitam a maioria dos problemas:

1. **Callback guarda, timer decide.** Não tome decisões dentro do callback de
   mensagem: guarde o valor e deixe um timer periódico olhar o conjunto. Senão a
   sua lógica passa a depender de qual mensagem chegou primeiro.
2. **Publique `cmd_vel` em todo ciclo**, inclusive o `Twist` zerado para parar.
   Silêncio não significa "mantenha"; significa "pare em 0,5 s".
3. **Trate `nan` no ângulo.** É o sinal de que o robô está fora do feixe, e é o
   estado inicial. `if math.isnan(angle)` é a primeira linha da sua máquina de
   estados.

---

## 7. Depurando: teste um elo de cada vez

A cadeia é longa, então nunca pergunte "por que não funciona?" — pergunte "até
onde funciona?". De trás para frente:

```bash
# 1. A ponte está viva e os tópicos existem?
ros2 node list                    # /myRobot/remoteAPI_ROS2_bridge
ros2 topic list | grep myRobot

# 2. A cena está produzindo dados? (elo CoppeliaSim → ponte)
ros2 topic echo /myRobot/battery --field percentage
ros2 topic echo /myRobot/charging_base/relativeAngle

# 3. Os comandos saem do ROS? (elo seu nó → ponte)
ros2 topic echo /myRobot/cmd_vel          # num terminal
ros2 topic pub -r 10 /myRobot/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}}'                    # noutro: o robô anda?

# 4. Quem está publicando e quem está ouvindo?
ros2 topic info /myRobot/cmd_vel --verbose
```

Se o passo 2 não mostra nada, o problema está na cena (script desabilitado,
simulação parada, robô fora do feixe). Se o 2 funciona e o 3 não move o robô,
olhe a bateria e o `python_controler`. Se os dois funcionam e o seu controlador
não, o problema é seu — e aí já é a parte divertida.

---

## 8. Resumo em cinco frases

O CoppeliaSim guarda tudo em **sinais**, que são variáveis globais dentro do
simulador, nomeadas com o handle do robô na frente. A ponte é o único processo
que fala as duas línguas: ela *pergunta* os sinais pela Remote API e os
*publica* como tópicos, e faz o inverso com os comandos. Alguns sinais são
apagados por quem os lê, e é daí que vem quase toda a complicação do código da
ponte. O seu controlador de docking vive inteiramente do lado ROS 2, assinando
quatro tópicos e publicando dois. E ele precisa publicar `cmd_vel`
continuamente, porque a ponte, de propósito, esquece o comando em meio segundo.

---

Ver também: [README.md](README.md) (como compilar e rodar),
[Especifications.md](Especifications.md) (o enunciado) e
[coppeliasim/](coppeliasim/) (cópia dos scripts que já estão dentro da cena).
