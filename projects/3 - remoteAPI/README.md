# 3 - remoteAPI

Teleoperação do robô diferencial `/myRobot` pela **ZeroMQ Remote API** do
CoppeliaSim. São duas versões do mesmo controle, para comparar as duas formas
de comandar um robô simulado:

| Script | Como você dirige | Precisa de ROS 2? |
|---|---|---|
| [`controllers/teleoperation.py`](controllers/teleoperation.py) | teclado, lido direto no terminal com `curses` | **não** |
| [`controllers/teleoperation_with_ros2.py`](controllers/teleoperation_with_ros2.py) | tópico ROS 2, alimentado pelo `turtle_teleop_key` | sim |

```
teleoperation.py ──────────────(ZeroMQ :23000)──────────────▶ CoppeliaSim

turtle_teleop_key ──(/turtle1/cmd_vel)──▶ teleoperation_with_ros2.py ──(ZeroMQ)──▶ CoppeliaSim
                                                    │
                                                    └──▶ /joint_states, /proximity
```

Nos dois casos, **nenhum plugin é necessário dentro do CoppeliaSim**: o
simulador escuta na porta 23000 de fábrica, e é o processo Python externo que
chama as funções `sim.*` remotamente.

> Não rode os dois ao mesmo tempo. Ambos escrevem nos mesmos motores e um anula
> o outro.

---

## 1. Pré-requisitos

- **CoppeliaSim aberto**, com uma das cenas desta pasta carregada
- **Python do sistema** com o cliente da Remote API:

  ```bash
  /usr/bin/python3 -c "import coppeliasim_zmqremoteapi_client; print('ok')"
  ```

  Se faltar:

  ```bash
  /usr/bin/python3 -m pip install --user --break-system-packages coppeliasim-zmqremoteapi-client
  ```

- Só para o segundo script: **ROS 2 Jazzy** e o `turtlesim`
  (`sudo apt install ros-jazzy-turtlesim`)

### Use `/usr/bin/python3`, com caminho completo

Se você tiver Anaconda ou um `.venv` ativo, o `python3` puro aponta para ele —
e nenhum dos dois enxerga o `rclpy`. O sintoma clássico é
`GLIBCXX_3.4.30' not found` ao importar `rclpy`. O `.venv` na raiz deste
repositório tem o cliente da Remote API, então serve para o
`teleoperation.py`, mas não para a versão com ROS.

### As cenas

Há duas nesta pasta, e ambas trazem o mesmo robô:

- [`myRobot.ttt`](myRobot.ttt)
- [`myRobot_HandsOn.ttt`](myRobot_HandsOn.ttt)

Os scripts procuram estes três objetos, com estes nomes exatos:

```
/myRobot
├── leftMotor          (revolute joint, modo de velocidade)
├── rightMotor         (revolute joint, modo de velocidade)
└── proximitySensor
```

Se algum não existir, o script para com uma mensagem que **lista todos os
objetos da cena aberta** — é a forma mais rápida de descobrir o nome certo.

**Desabilite child scripts do robô que escrevam nos motores.** Eles rodam a
cada passo de simulação e sobrescrevem o que vem de fora. O
`teleoperation_with_ros2.py` avisa no terminal quando encontra um script
habilitado no robô.

---

## 2. Rodando sem ROS 2

Com a cena aberta:

```bash
cd "projects/3 - remoteAPI/controllers"
/usr/bin/python3 teleoperation.py
```

O script dá o *play* sozinho se a simulação estiver parada, e a para ao sair —
mas só se foi ele quem iniciou, para não interromper uma simulação que você
mesmo começou.

| Tecla | Ação |
|---|---|
| `W` / `↑` | frente |
| `S` / `↓` | ré |
| `A` / `←` | girar à esquerda |
| `D` / `→` | girar à direita |
| `Espaço` | parar |
| `Q`, `Esc`, `Ctrl+C` | sair |

O terminal ocupa a tela inteira e mostra, ao vivo, as velocidades das rodas e a
leitura do sensor de proximidade.

### Por que o robô para sozinho quando você solta a tecla

O terminal não avisa quando uma tecla é **solta** — ele só entrega os toques.
Então o script usa um prazo: passados `INPUT_TIMEOUT` (0,6 s) sem receber tecla
de movimento, ele zera os motores. Segurar a tecla funciona porque o
*auto-repeat* do teclado gera toques repetidos que renovam o prazo.

Esses 0,6 s são maiores que o atraso de repetição do teclado (500 ms por
padrão) de propósito: com um valor menor, o robô daria um tranco entre o
primeiro toque e o começo da repetição.

---

## 3. Rodando com ROS 2

Três terminais. Em **todos** eles, primeiro:

```bash
source /opt/ros/jazzy/setup.bash
```

**Terminal 1 — a ponte:**

```bash
cd "projects/3 - remoteAPI/controllers"
/usr/bin/python3 teleoperation_with_ros2.py
```

Espere a linha de log com o raio da roda e a distância entre rodas medidas na
cena: é a confirmação de que ele achou o robô.

**Terminal 2 — o teclado:**

```bash
ros2 run turtlesim turtle_teleop_key
```

Dirija com as setas, **mantendo o foco na janela deste terminal**. Se você
clicar no CoppeliaSim para ver o robô, as teclas param de chegar.

**Terminal 3 — observando (opcional):**

```bash
ros2 topic echo /proximity
ros2 topic echo /joint_states
ros2 node list          # /coppelia_bridge
ros2 run rqt_graph rqt_graph
```

### Alternativa ao turtlesim

O `teleop_twist_keyboard` publica velocidades de verdade, em m/s e rad/s, em
vez dos `2.0` fixos do turtlesim:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/turtle1/cmd_vel
```

Se usar este, zere as escalas, porque elas existem só para converter os valores
fixos do turtlesim:

```bash
/usr/bin/python3 teleoperation_with_ros2.py --ros-args \
  -p linear_scale:=1.0 -p angular_scale:=1.0
```

### Tópicos

| Tópico | Tipo | Direção |
|---|---|---|
| `/turtle1/cmd_vel` | `geometry_msgs/Twist` | entrada — `linear.x` (m/s) e `angular.z` (rad/s) |
| `/joint_states` | `sensor_msgs/JointState` | saída — posição e velocidade dos dois motores |
| `/proximity` | `sensor_msgs/Range` | saída — distância; `inf` quando nada é detectado |

Os nomes são **absolutos** neste script (começam com `/`), então não mudam se
você aplicar um namespace ao nó.

### Parâmetros

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `robot` | `/myRobot` | Caminho do robô na cena |
| `wheel_radius` | `0.05` | Raio da roda (m) |
| `wheel_separation` | `0.0` | Distância entre rodas (m); `0.0` = medir na própria cena |
| `max_wheel_speed` | `10.0` | Limite por roda (rad/s); satura mantendo a curva |
| `linear_scale` | `0.15` | Multiplica `linear.x` |
| `angular_scale` | `0.75` | Multiplica `angular.z` |
| `sensor_range` | `1.0` | Alcance declarado do sensor (m) |
| `cmd_timeout` | `0.5` | Segundos sem comando até parar |
| `rate` | `20.0` | Frequência do laço que fala com o simulador (Hz) |
| `autostart` | `true` | Dar *play* se a simulação estiver parada |

As escalas convertem o comando do turtlesim, que manda `2.0` fixo nos dois
eixos: com os padrões acima, isso vira 0,3 m/s e 1,5 rad/s.

```bash
/usr/bin/python3 teleoperation_with_ros2.py --ros-args \
  -p robot:=/meuRobo -p max_wheel_speed:=5.0
```

---

## 4. As duas abordagens, lado a lado

O interesse de ter os dois scripts é justamente a comparação:

| | `teleoperation.py` | `teleoperation_with_ros2.py` |
|---|---|---|
| Processos | 1 | 3 (ponte + teleop + simulador) |
| Entrada | `curses`, teclado do terminal | tópico ROS 2 |
| Saída de dados | texto na tela | tópicos `/joint_states` e `/proximity` |
| Trocar o controlador | reescrever o script | publicar no mesmo tópico, de qualquer nó |
| Gravar uma sessão | não dá | `ros2 bag record` |
| Cinemática | teclas já viram velocidade de roda | `Twist` do corpo → cinemática inversa |

O segundo é mais peça-por-peça, e é esse desacoplamento que justifica o ROS: o
robô simulado vira um participante do grafo, e qualquer nó — um teleop, um
controlador autônomo, um gravador — passa a poder falar com ele sem saber que
do outro lado existe um CoppeliaSim.

---

## 5. Problemas comuns

| Sintoma | O que fazer |
|---|---|
| `Não consegui falar com o CoppeliaSim` / conexão recusada | O simulador não está aberto, ou um child script da cena travou a thread principal (típico de controlador em Python com `curses`, ou que abre um `RemoteAPIClient` para o próprio simulador). Pare a simulação e desabilite esse script |
| `Objeto "/myRobot/leftMotor" não existe na cena` | O script lista os objetos disponíveis; ajuste `-p robot:=` ou renomeie na cena |
| `ModuleNotFoundError: coppeliasim_zmqremoteapi_client` | Instale no Python do sistema (seção 1) |
| `GLIBCXX_3.4.30' not found` ao importar `rclpy` | Você usou o `python3` do Anaconda; chame `/usr/bin/python3` com caminho completo |
| O teleop responde, mas o robô não anda | Foco está na janela errada; simulação pausada; ou um child script do robô sobrescrevendo os motores |
| O robô anda e para sozinho | Comportamento esperado — ver a explicação do prazo na seção 2 |
| A tela do `teleoperation.py` fica truncada | Aumente o terminal; o script corta o que não cabe para o `curses` não lançar erro |
