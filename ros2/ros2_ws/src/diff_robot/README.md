# diff_robot

Ponte ROS 2 ↔ CoppeliaSim para o robô diferencial da cena, usando a **ZeroMQ
Remote API**. O nó ROS conversa direto com o simulador pela porta 23000: não é
preciso plugin `simROS2` nem script na cena.

```
dummy_driver / teleop ──(cmd_vel)──▶ coppelia_bridge ──(ZeroMQ :23000)──▶ CoppeliaSim
                                            ◀──(joint_states, proximity)──
```

| Executável | Papel |
|---|---|
| `coppelia_bridge` | Assina `cmd_vel`, aplica cinemática inversa, comanda os motores e publica `joint_states` e `proximity` |
| `dummy_driver` | Motorista de teste: publica um roteiro fixo em `cmd_vel` (frente, esquerda, frente, direita, repetindo) |

---

## 1. Pré-requisitos

- ROS 2 Jazzy (`/opt/ros/jazzy`)
- CoppeliaSim aberto, com a cena do robô diferencial
- Cliente Python da Remote API disponível para o `python3` do sistema:

  ```bash
  python3 -c "import coppeliasim_zmqremoteapi_client; print('ok')"
  ```

  Se der `ModuleNotFoundError`:

  ```bash
  pip install --user coppeliasim-zmqremoteapi-client
  ```

  > O `.venv/` na raiz do repositório **não** é usado pelo ROS: o `ros2 run`
  > executa com o Python do sistema. Por isso o pacote precisa estar visível lá.

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
Isso inclui o `my_robot_ros2.lua` do pacote `car_control`: os dois pacotes são
formas alternativas de controlar o mesmo robô, **não use os dois ao mesmo tempo**.

---

## 2. Compilar

```bash
cd ros2/ros2_ws
colcon build --packages-select diff_robot --symlink-install
source install/setup.zsh     # no zsh; use setup.bash se a sua shell for bash
```

Duas coisas que derrubam esse passo:

- **Sourcear a variante errada.** No zsh, `source install/setup.bash` não
  descobre a própria pasta (`$BASH_SOURCE` não existe lá) e procura os arquivos
  no diretório atual: aparece
  `no such file or directory: .../ros2_ws/local_setup.sh` e, logo depois,
  `Package 'diff_robot' not found`. Use `setup.zsh`.
- **`source` vale só para a shell atual** — repita em cada terminal novo.

Confira que deu certo antes de seguir:

```bash
ros2 pkg executables diff_robot
# diff_robot coppelia_bridge
# diff_robot dummy_driver
```

---

## 3. Executar

Abra o CoppeliaSim com a cena antes de rodar. A ponte dá o *play* sozinha se a
simulação estiver parada (parâmetro `autostart`), e a para ao sair se foi ela
quem iniciou.

### Teste rápido, sem teclado

```bash
ros2 launch diff_robot diff_robot.launch.py dummy:=1
```

Sobe a ponte **e** o `dummy_driver`. O robô começa a percorrer o roteiro
imediatamente. É a forma mais rápida de validar que tudo está conectado.

### Só a ponte (para dirigir você mesmo)

```bash
ros2 launch diff_robot diff_robot.launch.py
```

E, em outro terminal (com o `source` feito):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/diff_robot/cmd_vel
```

Ou sem teclado nenhum, publicando na mão — 0,2 m/s para frente girando à esquerda:

```bash
ros2 topic pub -r 10 /diff_robot/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.2}, angular: {z: 0.5}}'
```

> Use `-r 10` (repetição). A ponte zera os motores se ficar `cmd_timeout`
> segundos sem comando novo — é a proteção contra teleop que trava ou cai.

### Argumentos do launch

| Argumento | Padrão | O que faz |
|---|---|---|
| `dummy` | `0` | `1` sobe também o `dummy_driver` |
| `robot` | `/myRobot` | Caminho do robô na cena |

```bash
ros2 launch diff_robot diff_robot.launch.py robot:=/meuRobo dummy:=1
```

### Rodando os nós separadamente

O launch aplica o namespace `diff_robot`. Sem ele, os tópicos ficam na raiz:

```bash
ros2 run diff_robot coppelia_bridge --ros-args -p robot:=/myRobot   # → /cmd_vel
ros2 run diff_robot dummy_driver                                    # → /cmd_vel
```

---

## 4. Tópicos

Com o namespace do launch:

| Tópico | Tipo | Direção |
|---|---|---|
| `/diff_robot/cmd_vel` | `geometry_msgs/Twist` | entrada (`linear.x` m/s, `angular.z` rad/s) |
| `/diff_robot/joint_states` | `sensor_msgs/JointState` | saída — posição e velocidade dos dois motores |
| `/diff_robot/proximity` | `sensor_msgs/Range` | saída — distância; `inf` quando nada é detectado |

Para conferir se está rodando:

```bash
ros2 node list
ros2 topic hz /diff_robot/joint_states     # deve bater o parâmetro rate (20 Hz)
ros2 topic echo /diff_robot/proximity
ros2 run rqt_graph rqt_graph
```

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

```bash
ros2 launch diff_robot diff_robot.launch.py
# em outro terminal, ajustando em tempo de execução:
ros2 param set /diff_robot/coppelia_bridge max_wheel_speed 5.0
ros2 param list /diff_robot/coppelia_bridge
```

### `dummy_driver`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `linear_speed` | `0.2` | Velocidade nos trechos retos (m/s) |
| `angular_speed` | `1.0` | Velocidade nos giros (rad/s) |
| `forward_time` | `3.0` | Duração de cada trecho reto (s) |
| `turn_time` | `1.5` | Duração de cada giro (s) |
| `rate` | `10.0` | Frequência de publicação (Hz) |

---

## 6. Problemas comuns

| Mensagem / sintoma | O que fazer |
|---|---|
| `Não consegui falar com o CoppeliaSim na porta 23000` | Abra o simulador. Se já estiver aberto, algum child script da cena está travando a thread principal (típico de controlador Python com `curses` ou que abre um `RemoteAPIClient` para o próprio simulador) — pare a simulação e desabilite o script |
| `Objeto "/myRobot/leftMotor" não existe na cena` | A ponte lista os objetos existentes no terminal; ajuste `robot:=` ou renomeie na cena |
| `o script X está habilitado e sobrescreve os comandos` | Desabilite o child script do robô na cena |
| `a simulação está pausada` | Dê *play* no CoppeliaSim; motores pausados ignoram comandos |
| `ModuleNotFoundError: coppeliasim_zmqremoteapi_client` | Instale no Python do sistema (seção 1) |
| O robô anda e para sozinho | Normal sem comando contínuo — publique com `-r 10` ou aumente `cmd_timeout` |
| `Package 'diff_robot' not found` | Faltou sourcear o workspace nesta shell — e, no zsh, tem que ser `install/setup.zsh` |
| `no such file or directory: .../ros2_ws/local_setup.sh` | Mesma causa: `setup.bash` sourceado a partir do zsh |
| `not found: ".../install/sim_ros2_interface/share/.../local_setup.zsh"` | Aviso de um install antigo do plugin, herdado de outro caminho. Não atrapalha o `diff_robot`; para sumir, apague `build/sim_ros2_interface install/sim_ros2_interface` e recompile |

---

Ver também: [car_control](../car_control/README.md) (mesma tarefa pela via do
plugin `simROS2`) e [ESTRUTURA_ROS2.md](../../../ESTRUTURA_ROS2.md).
