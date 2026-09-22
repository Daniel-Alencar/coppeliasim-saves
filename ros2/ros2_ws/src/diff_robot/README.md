# diff_robot

Ponte ROS 2 ↔ CoppeliaSim para o robô diferencial da cena, usando a **ZeroMQ
Remote API**. O nó ROS conversa direto com o simulador pela porta 23000: não é
preciso plugin `simROS2` nem script na cena.

```
dummy_driver / teleop ──(cmd_vel)──▶ coppelia_bridge ──(ZeroMQ :23000)──▶ CoppeliaSim
                                            ◀──(joint_states, proximity)──
```

O pacote tem **dois executáveis**:

| Executável | Papel |
|---|---|
| `coppelia_bridge` | Assina `cmd_vel`, aplica cinemática inversa, comanda os motores e publica `joint_states` e `proximity` |
| `dummy_driver` | Motorista de teste: publica um roteiro fixo em `cmd_vel` (frente, esquerda, frente, direita, repetindo) |

---

## 1. Pré-requisitos

- ROS 2 Jazzy (`/opt/ros/jazzy`)
- CoppeliaSim aberto, com a cena do robô diferencial
- Cliente Python da Remote API disponível para o `python3` do sistema:
  `pip install --user coppeliasim-zmqremoteapi-client`
- `teleop_twist_keyboard` instalado, se for usar o teleop:
  `sudo apt install ros-jazzy-teleop-twist-keyboard`

### O cliente da Remote API precisa estar no Python do sistema

O `ros2 run` executa com o Python do sistema, não com o `.venv/` da raiz do
repositório. Confira se o pacote está visível lá:

```bash
python3 -c "import coppeliasim_zmqremoteapi_client; print('ok')"
```

Se der `ModuleNotFoundError`, instale com o `pip install --user` acima.

### A cena precisa ter

Sob o objeto do robô (padrão `/myRobot`), com estes nomes exatos:

```
/myRobot
├── leftMotor          (revolute joint, modo de velocidade)
├── rightMotor         (revolute joint, modo de velocidade)
└── proximitySensor
```

**Desabilite os child scripts do robô.** Um script da cena que também escreve
nos motores disputa o controle com a ponte e ela nem sempre ganha. O
`coppelia_bridge` avisa no terminal quando encontra um script habilitado.

---

## 2. Compilar

```bash
cd ros2/ros2_ws
colcon build --packages-select diff_robot --symlink-install
source install/setup.bash
source install/setup.zsh
```

No zsh, sourcear `setup.bash` não funciona: o script não descobre a própria
pasta e o pacote continua invisível (`Package 'diff_robot' not found`).

Confira:

```bash
ros2 pkg executables diff_robot    # diff_robot coppelia_bridge
                                   # diff_robot dummy_driver
```

---

## 3. Executar

Com o CoppeliaSim aberto e a cena carregada. A ponte dá o *play* sozinha se a
simulação estiver parada (parâmetro `autostart`), e a para ao sair se foi ela
quem iniciou.

**Terminal 1 — a ponte:**

```bash
ros2 launch diff_robot diff_robot.launch.py
```

**Terminal 2 — o teclado:**

```bash
source /opt/ros/jazzy/setup.bash
source /opt/ros/jazzy/setup.zsh
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/diff_robot/cmd_vel
```

O `-r cmd_vel:=/diff_robot/cmd_vel` é essencial: sem ele o teleop publica em
`/cmd_vel` e a ponte, que está no namespace, escuta `/diff_robot/cmd_vel`. Use
as teclas com a janela do teleop em foco.

### Sem teleop

Subindo a ponte **e** o `dummy_driver`, que faz o robô percorrer o roteiro
sozinho — a forma mais rápida de validar que tudo está conectado:

```bash
ros2 launch diff_robot diff_robot.launch.py dummy:=1
```

Ou publicando o `Twist` na mão — 0,2 m/s para frente girando à esquerda:

```bash
ros2 topic pub -r 10 /diff_robot/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.5}}'
```

O `-r 10` repete a mensagem: a ponte zera os motores se ficar `cmd_timeout`
segundos sem comando novo, que é a proteção contra teleop que trava ou cai.

### Argumentos do launch

| Argumento | Padrão | O que faz |
|---|---|---|
| `dummy` | `0` | `1` sobe também o `dummy_driver` |
| `robot` | `/myRobot` | Caminho do robô na cena |

```bash
ros2 launch diff_robot diff_robot.launch.py robot:=/meuRobo dummy:=1
```

### Sem o launch

```bash
ros2 run diff_robot coppelia_bridge --ros-args -p robot:=/myRobot   # → /cmd_vel
ros2 run diff_robot dummy_driver                                    # → /cmd_vel
```

O `ros2 run` puro não aplica namespace, e aí os tópicos ficam na raiz
(`/cmd_vel`) — o teleop, nesse caso, não precisa de remapeamento.

---

## 4. Tópicos

| Tópico | Tipo | Direção |
|---|---|---|
| `/diff_robot/cmd_vel` | `geometry_msgs/Twist` | entrada (`linear.x` m/s, `angular.z` rad/s) |
| `/diff_robot/joint_states` | `sensor_msgs/JointState` | saída — posição e velocidade dos dois motores |
| `/diff_robot/proximity` | `sensor_msgs/Range` | saída — distância; `inf` quando nada é detectado |

Conferindo:

```bash
ros2 topic hz /diff_robot/joint_states     # deve bater o parâmetro rate (20 Hz)
ros2 topic echo /diff_robot/proximity
ros2 node list
```

Se o `joint_states` não aparecer, o problema está na conexão com o CoppeliaSim:
a ponte escreve o motivo no terminal ao iniciar.

---

## 5. Parâmetros

### `coppelia_bridge`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `robot` | `/myRobot` | Caminho do robô na cena |
| `wheel_radius` | `0.05` | Raio da roda (m) |
| `wheel_separation` | `0.0` | Distância entre rodas (m); `0.0` = medir na própria cena |
| `max_wheel_speed` | `10.0` | Limite por roda (rad/s); satura mantendo a curva |
| `linear_scale` | `1.0` | Multiplica `linear.x` |
| `angular_scale` | `1.0` | Multiplica `angular.z` |
| `sensor_range` | `1.0` | Alcance declarado do sensor (m) |
| `cmd_timeout` | `0.5` | Segundos sem comando até parar |
| `rate` | `20.0` | Frequência do laço que fala com o simulador (Hz) |
| `autostart` | `true` | Dar *play* se a simulação estiver parada |

As escalas existem para adaptar teleops que mandam valores fixos. O
`turtle_teleop_key`, por exemplo, publica `2.0` nos dois eixos; com
`linear_scale:=0.15` e `angular_scale:=0.75` isso vira 0,3 m/s e 1,5 rad/s.

### `dummy_driver`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `linear_speed` | `0.2` | Velocidade nos trechos retos (m/s) |
| `angular_speed` | `1.0` | Velocidade nos giros (rad/s) |
| `forward_time` | `3.0` | Duração de cada trecho reto (s) |
| `turn_time` | `1.5` | Duração de cada giro (s) |
| `rate` | `10.0` | Frequência de publicação (Hz) |

Como o launch só repassa o `robot`, ajuste os outros em tempo de execução:

```bash
ros2 param set /diff_robot/coppelia_bridge max_wheel_speed 5.0
ros2 param list /diff_robot/coppelia_bridge
```

---

## 6. Problemas comuns

| Sintoma | O que fazer |
|---|---|
| `Não consegui falar com o CoppeliaSim na porta 23000` | Abra o simulador. Se já estiver aberto, algum child script da cena está travando a thread principal (típico de controlador Python com `curses` ou que abre um `RemoteAPIClient` para o próprio simulador) — pare a simulação e desabilite o script |
| `Objeto "/myRobot/leftMotor" não existe na cena` | A ponte lista os objetos existentes no terminal; ajuste `robot:=` ou renomeie na cena |
| `o script X está habilitado e sobrescreve os comandos` | Desabilite o child script do robô na cena |
| `a simulação está pausada` | Dê *play* no CoppeliaSim; motores pausados ignoram comandos |
| Teleop responde, robô não | Namespace: confira com `ros2 topic list` se o teleop publica em `/diff_robot/cmd_vel` |
| `ModuleNotFoundError: coppeliasim_zmqremoteapi_client` | Instale no Python do sistema (seção 1) |
| O robô anda e para sozinho | `cmd_timeout` expirou — publique com `-r 10` ou aumente `cmd_timeout` |
| Robô treme ou dois comandos brigam | O `car_control` está rodando ao mesmo tempo, há outro child script escrevendo nos motores, ou o `dummy_driver` (`dummy:=1`) e um teleop estão publicando juntos |
| `Package 'diff_robot' not found` | Faltou sourcear o workspace nesta shell — e, no zsh, tem que ser `install/setup.zsh` |
| `no such file or directory: .../ros2_ws/local_setup.sh` | `setup.bash` sourceado a partir do zsh |
| `not found: ".../install/sim_ros2_interface/.../local_setup.zsh"` | Install do plugin herdado de outro caminho; não atrapalha o `diff_robot`. Para sumir, apague `build/sim_ros2_interface install/sim_ros2_interface` e recompile |
