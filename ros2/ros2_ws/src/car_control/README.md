# car_control

Controle do robô diferencial do CoppeliaSim **por tópicos de motor**. Aqui o
ROS não fala com o simulador: ele publica a velocidade de cada roda, e um child
script Lua dentro da cena (com o plugin `simROS2`) assina esses tópicos e aplica
nos motores.

```
turtle_teleop_key ──(turtle1/cmd_vel)──▶ car_node ──(em memória)──▶ motor_publisher_node
                                                                            │
                                            my_robot/left_motor  ◀──────────┤
                                            my_robot/right_motor ◀──────────┘
                                                      │
                                                      ▼
                                   my_robot_ros2.lua (simROS2, dentro da cena)
```

Um único executável (`car_control`) sobe **dois nós** no mesmo processo, girados
por um `MultiThreadedExecutor`:

| Nó | Papel |
|---|---|
| `car_node` | Assina `turtle1/cmd_vel` e aplica a cinemática inversa do robô diferencial |
| `motor_publisher_node` | Publica, a 20 Hz, a velocidade de cada roda em `std_msgs/Float32` (rad/s) |

---

## 1. Pré-requisitos

- ROS 2 Jazzy (`/opt/ros/jazzy`)
- Plugin **`simROS2`** disponível no CoppeliaSim — é o que o pacote
  `sim_ros2_interface`, já presente em [../sim_ros2_interface](../sim_ros2_interface),
  constrói
- `turtlesim` instalado, se for usar o teleop:
  `sudo apt install ros-jazzy-turtlesim`

### O CoppeliaSim precisa ser aberto com o ROS carregado

O plugin só é carregado se o ambiente ROS estiver na shell que abre o
simulador. Abrir pelo ícone do sistema **não** funciona:

```bash
source /opt/ros/jazzy/setup.bash
source /opt/ros/jazzy/setup.zsh
./coppeliaSim.sh
```

No log do CoppeliaSim deve aparecer o carregamento do `simROS2`. Se aparecer
*plugin ... could not be loaded*, o ambiente não estava carregado ou o
`sim_ros2_interface` não foi compilado.

### O script na cena

Copie [coppeliasim/my_robot_ros2.lua](coppeliasim/my_robot_ros2.lua) para um
child script (Lua, **não threaded**) pendurado no robô da cena. Ele espera
encontrar `../leftMotor` e `../rightMotor` como irmãos do script e assina:

- `/car_control/my_robot/left_motor`
- `/car_control/my_robot/right_motor`

Os nomes estão fixos no script: se você rodar o nó **sem** o namespace
`car_control`, os tópicos não batem e o robô não se move.

---

## 2. Compilar

```bash
cd ros2/ros2_ws
colcon build --packages-select car_control --symlink-install
source install/setup.bash
source install/setup.zsh
```

No zsh, sourcear `setup.bash` não funciona: o script não descobre a própria
pasta e o pacote continua invisível (`Package 'car_control' not found`).

Confira:

```bash
ros2 pkg executables car_control    # car_control car_control
```

---

## 3. Executar

Com o CoppeliaSim aberto (do jeito da seção 1), a cena carregada, o script no
lugar e a simulação em **play**:

**Terminal 1 — o controle:**

```bash
ros2 launch car_control car_control.launch.py
```

**Terminal 2 — o teclado:**

```bash
source /opt/ros/jazzy/setup.bash
source /opt/ros/jazzy/setup.zsh
ros2 run turtlesim turtle_teleop_key --ros-args -r __ns:=/car_control
```

O `-r __ns:=/car_control` é essencial: sem ele o teleop publica em
`/turtle1/cmd_vel` e o `car_node`, que está no namespace, escuta
`/car_control/turtle1/cmd_vel`. Use as setas com a janela do teleop em foco.

### Sem teleop

Publicando o `Twist` na mão (os valores são escalados por `linear_scale` e
`angular_scale`):

```bash
ros2 topic pub -r 10 /car_control/turtle1/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 2.0}, angular: {z: 0.0}}'
```

Ou pulando o `car_node` e mandando direto para as rodas — bom para checar se o
script Lua está funcionando, isolado do resto:

```bash
ros2 topic pub -r 10 /car_control/my_robot/left_motor  std_msgs/msg/Float32 '{data: 3.0}'
ros2 topic pub -r 10 /car_control/my_robot/right_motor std_msgs/msg/Float32 '{data: 3.0}'
```

### Sem o launch

```bash
ros2 run car_control car_control --ros-args -r __ns:=/car_control
```

O `ros2 run` puro não aplica namespace, e aí os tópicos publicados não seriam
os que o script Lua assina — daí o remapeamento explícito.

---

## 4. Tópicos

| Tópico | Tipo | Direção |
|---|---|---|
| `/car_control/turtle1/cmd_vel` | `geometry_msgs/Twist` | entrada |
| `/car_control/my_robot/left_motor` | `std_msgs/Float32` | saída — rad/s da roda esquerda |
| `/car_control/my_robot/right_motor` | `std_msgs/Float32` | saída — rad/s da roda direita |

Conferindo:

```bash
ros2 topic echo /car_control/my_robot/left_motor
ros2 topic info /car_control/my_robot/left_motor --verbose   # o script Lua deve aparecer como subscriber
ros2 node list
```

Se o `--verbose` não mostrar nenhum *subscriber*, o problema está do lado do
CoppeliaSim (plugin ou script), não do lado do ROS.

---

## 5. Parâmetros

### `car_node`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `wheel_radius` | `0.05` | Raio da roda (m) |
| `wheel_separation` | `0.2` | Distância entre rodas (m) |
| `max_wheel_speed` | `10.0` | Limite por roda (rad/s); satura mantendo a curva |
| `linear_scale` | `0.15` | Multiplica `linear.x` |
| `angular_scale` | `0.75` | Multiplica `angular.z` |
| `cmd_timeout` | `0.5` | Segundos sem comando até zerar as rodas |

Os padrões das escalas estão calibrados para o `turtle_teleop_key`, que publica
`2.0` fixo nos dois eixos: dá 0,3 m/s e 1,5 rad/s. Se for publicar valores já em
m/s e rad/s, use `1.0` nas duas.

### `motor_publisher_node`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `rate` | `20.0` | Frequência de publicação (Hz) — publica sempre, inclusive zeros |

Como o launch não passa parâmetros, ajuste por linha de comando ou editando o
launch file:

```bash
ros2 run car_control car_control --ros-args -r __ns:=/car_control \
  -p wheel_separation:=0.25 -p linear_scale:=1.0 -p angular_scale:=1.0
```

---

## 6. Problemas comuns

| Sintoma | O que fazer |
|---|---|
| O nó roda, os tópicos existem, mas o robô não se move | Simulação parada (dê *play*); script Lua ausente, desabilitado ou *threaded*; plugin `simROS2` não carregado |
| `[simROS2] plugin could not be loaded` | O CoppeliaSim foi aberto sem o ambiente ROS carregado, ou falta compilar `sim_ros2_interface` |
| Teleop responde, robô não | Namespace: confira com `ros2 topic list` se o teleop publica em `/car_control/turtle1/cmd_vel` |
| `sim.getObject('../leftMotor')` falha no log da cena | O script não está pendurado no lugar certo, ou os motores têm outro nome |
| O robô anda e para sozinho | `cmd_timeout` expirou — o teleop só publica quando uma tecla chega; segure a tecla ou publique com `-r 10` |
| Robô treme ou dois comandos brigam | O `diff_robot` está rodando ao mesmo tempo, ou há outro child script escrevendo nos motores |
| `Package 'car_control' not found` | Faltou sourcear o workspace nesta shell — e, no zsh, tem que ser `install/setup.zsh` |
| `no such file or directory: .../ros2_ws/local_setup.sh` | `setup.bash` sourceado a partir do zsh |
| `not found: ".../install/sim_ros2_interface/.../local_setup.zsh"` | Install do plugin herdado de outro caminho; reconstrua (seção 2) |
