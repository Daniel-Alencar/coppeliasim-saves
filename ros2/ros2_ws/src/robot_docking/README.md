# robot_docking

Ponte ROS 2 ↔ CoppeliaSim que expõe, por tópicos, tudo o que um controlador de
docking externo precisa: bateria, estado de carga, beacon da base de recarga,
modo de docking e comando de velocidade. O nó conversa com o simulador pela
**ZeroMQ Remote API** (porta 23000): não é preciso plugin `simROS2`.

```
controlador de docking ──(cmd_vel, docking)──▶ coppelia_bridge ──(ZeroMQ :23000)──▶ CoppeliaSim
                       ◀──(battery, charging, strengthSignal, relativeAngle)──
```

O docking autônomo **não** é implementado aqui: este pacote é só a comunicação,
e será usado pelo controlador da próxima atividade.

| Executável | Papel |
|---|---|
| `coppelia_bridge` | Lê os sinais do robô na cena e publica no ROS; recebe `cmd_vel` e `docking` e escreve nos motores e no sinal de docking |

---

## 1. Pré-requisitos

- ROS 2 Jazzy (`/opt/ros/jazzy`)
- CoppeliaSim aberto, com a cena de docking
  (`projects/5 - robot_docking/Evaluation scene3.2_students.ttt`)
- Cliente Python da Remote API disponível para o `python3` do sistema:
  `pip install --user coppeliasim-zmqremoteapi-client`

### O cliente da Remote API precisa estar no Python do sistema

O `ros2 run` executa com o Python do sistema, não com o `.venv/` da raiz do
repositório. Confira se o pacote está visível lá:

```bash
python3 -c "import coppeliasim_zmqremoteapi_client; print('ok')"
```

### A cena precisa ter

Com estes nomes exatos:

```
/myRobot
├── leftMotor          (revolute joint, modo de velocidade)
├── rightMotor         (revolute joint, modo de velocidade)
├── battery            (script: escreve o sinal <handle>Battery)
├── dockingSensor      (script: pede a leitura ao beacon)
├── odometry           (script)
└── python_controler   (script: DESABILITE, ver abaixo)
/chargingBase
├── beacon             (script: escreve carga e beacon)
└── ir_beam            (proximity sensor: o feixe, alcance de 2 m)
```

**Desabilite o `/myRobot/python_controler`.** Ele escreve nos motores a cada
passo de simulação e anula o `cmd_vel`. A ponte avisa no terminal quando
encontra um script do robô que faz isso. Os demais scripts (`battery`,
`odometry`, `dockingSensor`, `encoder`) precisam continuar **habilitados**:
são eles que produzem os valores publicados.

O `python_controler` oferece uma via de convivência — dois sinais float,
`<handle>leftVel` e `<handle>rightVel`, que ele lê e aplica no lugar dos
sliders. O `motor_mode:=signal` usa essa via e mantém o joystick, os rótulos de
bateria e odometria e o checkbox *docking* funcionando. Só que ele **apaga o
sinal assim que o lê**, e cada chamada da Remote API custa ~12 ms com a
simulação rodando, então boa parte dos passos fica sem comando. Medido nesta
cena, pedindo 0,2 m/s por 4 s no ritmo máximo de escrita:

| `motor_mode` | Distância | Velocidade | Do pedido |
|---|---|---|---|
| `joint` (padrão) | 0,689 m | 0,172 m/s | 86 % |
| `signal` | 0,194 m | 0,047 m/s | 24 % |

Por isso o padrão é `joint`, com o `python_controler` desabilitado.

Como o `python_controler` também parava o robô com a bateria vazia, a ponte
passou a fazer isso (parâmetro `stop_when_battery_empty`).

### A bateria dura 100 s de simulação

O script `/myRobot/battery` começa em 100 % e gasta **1 % por segundo
simulado**. Passados ~100 s sem carregar, ela chega a 0 e o robô para — tanto
pela ponte quanto pelo `python_controler`, que fazem o mesmo corte. Se o robô
parar de responder ao `cmd_vel` no meio de um teste, confira a bateria antes de
procurar bug:

```bash
ros2 topic echo /myRobot/battery --field percentage --once
```

Parar e dar *play* de novo no CoppeliaSim reinicia a bateria em 100 %.

---

## 2. Compilar

```bash
cd ros2/ros2_ws
colcon build --packages-select robot_docking --symlink-install
source install/setup.bash
source install/setup.zsh
```

No zsh, sourcear `setup.bash` não funciona: o script não descobre a própria
pasta e o pacote continua invisível (`Package 'robot_docking' not found`).

Se o pacote já tiver sido compilado **sem** `--symlink-install`, o colcon falha
com `error: [Errno 2] No such file or directory: .../launch/robot_docking.launch.py`.
Misturar os dois modos no mesmo `build/` não funciona; apague e recompile:

```bash
rm -rf build/robot_docking install/robot_docking
colcon build --packages-select robot_docking --symlink-install
```

Confira:

```bash
ros2 pkg executables robot_docking    # robot_docking coppelia_bridge
```

---

## 3. Executar

Com o CoppeliaSim aberto e a cena carregada. A ponte dá o *play* sozinha se a
simulação estiver parada (parâmetro `autostart`), e a para ao sair se foi ela
quem iniciou.

**Terminal 1 — a ponte:**

```bash
ros2 launch robot_docking robot_docking.launch.py
```

Espere as duas linhas de log: a que mostra o handle do robô e a geometria das
rodas, e a que diz `ponte pronta`.

**Terminal 2 — os comandos e as leituras:**

```bash
source /opt/ros/jazzy/setup.bash
source /opt/ros/jazzy/setup.zsh

ros2 topic echo /myRobot/battery --field percentage
ros2 topic echo /myRobot/charging
ros2 topic echo /myRobot/charging_base/strengthSignal
ros2 topic echo /myRobot/charging_base/relativeAngle

ros2 topic pub --once /myRobot/docking std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /myRobot/docking std_msgs/msg/Bool '{data: false}'

ros2 topic pub -r 10 /myRobot/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.0}}'
```

O `-r 10` repete a mensagem: a ponte zera os motores se ficar `cmd_timeout`
segundos sem comando novo, que é a proteção contra um controlador que trave ou
caia.

### Dirigindo pelo teclado

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/myRobot/cmd_vel
```

### Argumentos do launch

| Argumento | Padrão | O que faz |
|---|---|---|
| `robot` | `/myRobot` | Caminho do robô na cena |
| `port` | `23000` | Porta da ZeroMQ Remote API |
| `motor_mode` | `joint` | Como o `cmd_vel` chega aos motores |

```bash
ros2 launch robot_docking robot_docking.launch.py robot:=/meuRobo port:=23010
ros2 launch robot_docking robot_docking.launch.py motor_mode:=signal
```

### Sem o launch

O launch aplica o namespace `myRobot`. Sem ele, os tópicos ficam na raiz
(`/cmd_vel`, `/battery`, …), e o remapeamento explícito resolve:

```bash
ros2 run robot_docking coppelia_bridge --ros-args -r __ns:=/myRobot
```

---

## 4. Tópicos

| Tópico | Tipo | Direção | Sinal na cena |
|---|---|---|---|
| `/myRobot/cmd_vel` | `geometry_msgs/Twist` | entrada (`linear.x` m/s, `angular.z` rad/s) | juntas, ou `<handle>leftVel`/`<handle>rightVel` (ver `motor_mode`) |
| `/myRobot/docking` | `std_msgs/Bool` | entrada | `<handle>Docking` (1 ou 0) |
| `/myRobot/battery` | `sensor_msgs/BatteryState` | saída | `<handle>Battery` |
| `/myRobot/charging` | `std_msgs/Bool` | saída | `<handle>Charging` |
| `/myRobot/charging_base/strengthSignal` | `std_msgs/Float32` | saída | `<handle>StrengthSignal` |
| `/myRobot/charging_base/relativeAngle` | `std_msgs/Float32` | saída | `<handle>RelativeAngle` |

> O enunciado chama os sinais do beacon de `signalStrength` e `relativeAngle`,
> mas o script `/chargingBase/beacon` desta cena escreve `StrengthSignal` e
> `RelativeAngle`. A ponte procura os dois pares, nessa ordem.

`<handle>` é o handle inteiro do robô na cena, que a ponte descobre sozinha e
mostra no log de abertura. Na cena de avaliação ele é `84`, então os sinais se
chamam `84Battery`, `84Docking` e assim por diante.

Conferindo:

```bash
ros2 topic list
ros2 topic hz /myRobot/battery      # deve bater o parâmetro rate (10 Hz)
ros2 node list                      # /myRobot/remoteAPI_ROS2_bridge
```

### Como cada valor é obtido

- **Bateria.** O `percentage` da mensagem vai de 0 a 1, e o sinal da cena de 0 a
  100 %. O `power_supply_status` acompanha a carga: `CHARGING`, `DISCHARGING` ou
  `FULL`. Os campos que a cena não simula (tensão, corrente, temperatura) ficam
  `NaN`, como a mensagem pede.
- **Carga.** O script do beacon escreve `Charging` a cada segundo enquanto o robô
  está na base, e o script da bateria **apaga** esse sinal ao lê-lo. Por isso a
  ponte guarda o último valor visto e o confere com a bateria, que sobe
  carregando e desce descarregando.
- **Beacon.** O script `/chargingBase/beacon` só calcula a leitura para o robô
  cujo handle está no sinal global `Beacon`, e a ponte faz esse pedido a cada
  ciclo. Ela aceita os nomes do enunciado (`signalStrength`, `relativeAngle`) e
  também os que essa cena usa (`StrengthSignal`, `RelativeAngle`).
- **Fora do feixe.** O beacon nunca apaga os sinais, então a ponte os apaga
  depois de ler. Passados `beacon_timeout` segundos sem leitura nova, ela
  publica força `0.0` e ângulo `nan`, em vez de repetir um valor velho.
- **Ângulo.** Em radianos: `0` com a base à frente, positivo de um lado e
  negativo do outro.

---

## 5. Parâmetros

### `coppelia_bridge`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `host` | `localhost` | Onde está o CoppeliaSim |
| `port` | `23000` | Porta da ZeroMQ Remote API |
| `robot` | `/myRobot` | Caminho do robô na cena |
| `wheel_radius` | `0.05` | Raio da roda (m) |
| `wheel_separation` | `0.0` | Distância entre rodas (m); `0.0` = medir na própria cena |
| `motor_sign` | `-1.0` | Sinal aplicado às juntas (só em `motor_mode=joint`) |
| `motor_mode` | `joint` | `joint` = escreve nas juntas; `signal` = escreve `leftVel`/`rightVel` e convive com o `python_controler` |
| `max_wheel_speed` | `10.0` | Limite por roda (rad/s); satura mantendo a curva |
| `cmd_timeout` | `0.5` | Segundos sem `cmd_vel` até parar |
| `beacon_timeout` | `0.5` | Segundos sem leitura do beacon até publicar "sem sinal" |
| `stop_when_battery_empty` | `true` | Com a bateria em 0 %, os motores ficam parados |
| `rate` | `10.0` | Frequência de leitura dos sensores (Hz) |
| `motor_rate` | `20.0` | Frequência de escrita nos motores (Hz) |
| `autostart` | `true` | Dar *play* se a simulação estiver parada |

O `motor_sign` existe porque, nesta cena, velocidade **negativa** faz o robô
andar para a frente. É a mesma convenção do `python_controler`, que usa
`-vel/raio`. Se o robô andar ao contrário, troque para `1.0`.

As duas frequências são baixas de propósito. Com a simulação rodando, cada
chamada da Remote API só é atendida entre passos e custa ~12 ms (medido nesta
cena), o que dá um teto de ~80 chamadas por segundo para a ponte inteira.
Aumentar `rate` ou `motor_rate` não deixa nada mais rápido: só enfileira.

Como o launch só repassa o `robot` e a `port`, ajuste os outros em tempo de
execução:

```bash
ros2 param set /myRobot/remoteAPI_ROS2_bridge max_wheel_speed 5.0
ros2 param list /myRobot/remoteAPI_ROS2_bridge
```

---

## 6. Problemas comuns

| Sintoma | O que fazer |
|---|---|
| `Não consegui falar com o CoppeliaSim em localhost:23000` | Abra o simulador. Se já estiver aberto, algum script da cena está travando a thread principal — pare a simulação e desabilite esse script |
| `Objeto "/myRobot/leftMotor" não existe na cena` | A ponte lista os objetos existentes no terminal; ajuste `robot:=` ou renomeie na cena |
| `motor_mode=joint, mas estes scripts da cena escrevem nos motores` | Desabilite o `/myRobot/python_controler` na cena, ou use `motor_mode:=signal` |
| `motor_mode=signal, mas nenhum script habilitado do robô lê os sinais` | Habilite o `python_controler`, ou volte para `motor_mode:=joint` |
| O robô anda muito mais devagar que o `linear.x` pedido | `motor_mode=signal` entrega ~25 % da velocidade; use `joint` |
| O robô parou de responder ao `cmd_vel` do nada | Bateria zerada (100 s de simulação); pare e dê *play* de novo |
| `a simulação está pausada` | Dê *play* no CoppeliaSim; motores pausados ignoram comandos |
| O robô anda para trás com `linear.x` positivo | Troque `motor_sign` para `1.0` |
| `battery` não publica nada | O script da bateria só escreve o primeiro valor depois de 1 s de simulação; confira se ele está habilitado |
| `strengthSignal` fica sempre em `0.0` e o ângulo em `nan` | O robô está fora do feixe da base (alcance de 2 m), ou o script `/chargingBase/beacon` está desabilitado |
| O robô anda e para sozinho | `cmd_timeout` expirou — publique com `-r 10` ou aumente `cmd_timeout` |
| `bateria em 0 %: motores parados` | Leve o robô à base para carregar, ou use `stop_when_battery_empty:=false` |
| `Package 'robot_docking' not found` | Faltou sourcear o workspace nesta shell — e, no zsh, tem que ser `install/setup.zsh` |
| `error: [Errno 2] No such file or directory: .../robot_docking.launch.py` ao compilar | Build antigo sem `--symlink-install`; apague `build/robot_docking install/robot_docking` e recompile |
