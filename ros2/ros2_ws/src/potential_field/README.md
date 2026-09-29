# potential_field

Navegação por **campos potenciais** em ROS 2: o robô anda até as bananas
desviando dos poops, na cena `pega_banana_potential_field.ttt`
(`projects/7 - obstacle_avoidance/`). É o exercício daquela pasta, implementado
como pacote ROS 2 em vez de child script.

A trajetória **não** é um caminho traçado nem uma lista de pontos: ela aparece
sozinha da soma das forças.

```
F = F_atrativa(banana alvo) + Σ F_repulsiva(poop)
```

O pacote separa o algoritmo da simulação em dois nós:

```
                    bananas, poops, pose                cmd_vel
CoppeliaSim ──ZeroMQ:23000──▶ coppelia_bridge ──────▶ navigator ──┐
     ▲                              ▲                             │
     └──────── motores ─────────────┴──────── cmd_vel ────────────┘
```

| Executável | Papel |
|---|---|
| `coppelia_bridge` | Só comunicação: lê a pose do robô e os sinais `banana`/`poop` da cena, publica no ROS e aplica o `cmd_vel` nos motores |
| `navigator` | Só algoritmo: recebe mapa e pose por tópicos, calcula o campo potencial e publica `cmd_vel`. Não conhece o CoppeliaSim |
| `fake_world` | Substitui a ponte para testar o navegador sem abrir o simulador |

Essa divisão é o que torna o algoritmo testável: o `navigator` roda igual
contra o simulador ou contra o `fake_world`.

---

## 1. Pré-requisitos

- ROS 2 Jazzy (`/opt/ros/jazzy`)
- CoppeliaSim aberto com `projects/7 - obstacle_avoidance/pega_banana_potential_field.ttt`
- Cliente da Remote API no Python do sistema (o `ros2 run` não usa o `.venv/`):

```bash
pip install --user coppeliasim-zmqremoteapi-client
python3 -c "import coppeliasim_zmqremoteapi_client; print('ok')"
```

### A cena precisa ter

```
/myRobot
├── leftMotor          (revolute joint, modo de velocidade)
├── rightMotor         (revolute joint, modo de velocidade)
└── python_controler   (script: DESABILITE)
/buildScene            (script: cria os sinais banana e poop no play)
```

**Desabilite o `/myRobot/python_controler`.** Ele escreve nos motores a cada
passo de simulação e anula o `cmd_vel`. A ponte avisa no terminal quando acha
um script da cena nessa situação.

### Os sinais do mapa

O enunciado entrega as posições em dois sinais de string, com tabelas Lua
empacotadas no formato `[x1, y1, x2, y2, ...]`:

| Sinal | Conteúdo |
|---|---|
| `banana` | posições das bananas |
| `poop` | posições dos poops |

Eles **só existem depois do play**: quem os cria é o `/buildScene`. Enquanto
não existirem, a ponte avisa uma vez e continua tentando.

---

## 2. Compilar

```bash
cd ros2/ros2_ws
colcon build --packages-select potential_field --symlink-install
source install/setup.bash    # no zsh: source install/setup.zsh
ros2 pkg executables potential_field
```

---

## 3. Executar

Com o CoppeliaSim aberto e a cena carregada. A ponte dá o *play* sozinha se a
simulação estiver parada, e a para ao sair se foi ela quem iniciou.

```bash
ros2 launch potential_field potential_field.launch.py
```

O robô sai atrás da banana pendente mais próxima e para quando todas tiverem
sido alcançadas:

```
[navigator]: 12 bananas no mapa
[navigator]: 8 poops no mapa
[navigator]: banana 1/12 alcançada
...
[navigator]: todas as 12 bananas coletadas, parando
```

### Sem o simulador

Para ver o algoritmo funcionando sem abrir o CoppeliaSim:

```bash
ros2 launch potential_field potential_field.launch.py fake:=1
```

O `fake_world` integra a cinemática do robô e publica os mesmos tópicos da
ponte. O mapa padrão dele tem três bananas e uma parede de poops entre o robô e
a primeira — o caso em que o campo empata e a tangente precisa agir.

### Só a ponte, dirigindo pelo teclado

```bash
ros2 launch potential_field potential_field.launch.py navigator:=0
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/myRobot/cmd_vel
```

### Argumentos do launch

| Argumento | Padrão | O que faz |
|---|---|---|
| `robot` | `/myRobot` | Caminho do robô na cena |
| `port` | `23000` | Porta da ZeroMQ Remote API |
| `navigator` | `1` | `0` sobe só a ponte (para teleop ou outro controlador) |
| `fake` | `0` | `1` troca a ponte pelo `fake_world` |

---

## 4. Tópicos

Todos no namespace `myRobot`, aplicado pelo launch.

| Tópico | Tipo | Quem publica | Conteúdo |
|---|---|---|---|
| `/myRobot/pose` | `geometry_msgs/PoseStamped` | ponte | Pose do robô no mundo |
| `/myRobot/bananas` | `geometry_msgs/PoseArray` | ponte | Mapa do sinal `banana` |
| `/myRobot/poops` | `geometry_msgs/PoseArray` | ponte | Mapa do sinal `poop` |
| `/myRobot/cmd_vel` | `geometry_msgs/Twist` | navegador | `linear.x` (m/s), `angular.z` (rad/s) |
| `/myRobot/target` | `geometry_msgs/PointStamped` | navegador | Banana perseguida agora |
| `/myRobot/force` | `geometry_msgs/Vector3Stamped` | navegador | Resultante do campo |
| `/myRobot/collected` | `std_msgs/Int32` | navegador | Quantas bananas já foram |

```bash
ros2 topic echo /myRobot/target
ros2 topic echo /myRobot/force
ros2 run rqt_graph rqt_graph
```

Os dois tópicos de mapa usam QoS **transient local**: a última mensagem fica
guardada, então o navegador recebe o mapa mesmo subindo depois da ponte.

### Duas convenções que a ponte acerta

- **A frente do robô é o eixo +y dele** nesta cena, e não o +x como é usual no
  ROS. A ponte converte, e o `yaw` publicado já é no sentido do ROS.
- **Velocidade negativa na junta faz o robô andar para a frente** (mesma
  convenção do `python_controler` da cena). É o parâmetro `motor_sign`, em
  `-1.0`. Se o robô andar ao contrário, troque para `1.0`.

---

## 5. O algoritmo

Tudo em `potential_field/navigator.py`, um ciclo por mensagem de `pose`.

1. **Alvo.** A banana pendente mais próxima. Só ela atrai; as outras são
   ignoradas até virarem alvo.
2. **Atração.** Aponta para o alvo, com módulo saturado em `k_att`: longe o
   puxão é constante, perto ele diminui junto com a distância.
3. **Repulsão.** Cada poop dentro de `d0_poop` empurra o robô, na forma
   clássica `k_rep · (1/d − 1/d0) / d²`, apontando do poop para o robô.
4. **Resultante.** Soma das duas, saturada em `f_max`. A direção dela é o rumo
   desejado.
5. **Comando.** O erro entre esse rumo e o do robô vira `angular.z`; o avanço é
   `v_max · cos(erro)`, e com erro acima de `turn_in_place` o robô gira parado
   em vez de andar torto.
6. **Coleta.** Chegando a `collect_radius` do alvo, a banana sai da lista e ele
   escolhe a próxima. Sem bananas pendentes, publica velocidade zero e para.

### Mínimos locais

Campo potencial tem um problema conhecido: a repulsão pode cancelar a atração e
o robô trava sem chegar na banana — típico com vários poops enfileirados entre
ele e o alvo.

O navegador detecta isso pelo **progresso**: se em `stall_time` segundos a
distância até o alvo não cai pelo menos `stall_progress`, ele liga por
`swirl_time` segundos uma componente **tangencial**, perpendicular à repulsão,
ou seja, acompanhando a borda do obstáculo. O lado é escolhido pela geometria:
aquele cuja tangente aponta mais para o alvo.

Continua sendo força, e não caminho traçado: a tangente entra na soma junto com
a atração e a repulsão.

> O lado precisa vir da geometria. Uma versão anterior simplesmente invertia o
> lado a cada ativação, e no teste com parede de poops o robô oscilava na frente
> dela indefinidamente, sem nunca contornar.

---

## 6. Parâmetros

### `navigator`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `k_att` | `1.0` | Ganho da atração |
| `att_range` | `1.0` | Acima desta distância a atração satura (m) |
| `k_rep` | `0.08` | Ganho da repulsão |
| `d0_poop` | `0.55` | Raio de influência de cada poop (m) |
| `d_min` | `0.08` | Distância mínima usada na conta (m) |
| `f_max` | `3.0` | Saturação da resultante |
| `v_max` | `0.25` | Velocidade de avanço (m/s) |
| `w_max` | `1.8` | Velocidade de giro (rad/s) |
| `k_heading` | `3.0` | rad/s por rad de erro de rumo |
| `turn_in_place` | `1.31` (75°) | Acima disso gira parado (rad) |
| `collect_radius` | `0.10` | Distância que conta como banana alcançada (m) |
| `stall_time` | `2.0` | Segundos sem progresso que caracterizam empate |
| `stall_progress` | `0.05` | Aproximação mínima nesse tempo (m) |
| `swirl_time` | `2.5` | Duração da componente tangencial (s) |
| `swirl_gain` | `1.2` | Peso dela em relação à atração |

Os ganhos são lidos a cada ciclo, então dá para ajustar com a simulação rodando:

```bash
ros2 param set /myRobot/navigator k_rep 0.15
ros2 param set /myRobot/navigator d0_poop 0.7
ros2 param list /myRobot/navigator
```

### `coppelia_bridge`

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `host` / `port` | `localhost` / `23000` | Onde está o CoppeliaSim |
| `robot` | `/myRobot` | Caminho do robô na cena |
| `banana_signal` / `poop_signal` | `banana` / `poop` | Nomes dos sinais do mapa |
| `wheel_radius` | `0.05` | Raio da roda (m) |
| `wheel_separation` | `0.0` | Distância entre rodas (m); `0.0` = medir na cena |
| `motor_sign` | `-1.0` | Sinal aplicado às juntas |
| `max_wheel_speed` | `10.0` | Limite por roda (rad/s) |
| `cmd_timeout` | `0.5` | Segundos sem `cmd_vel` até parar |
| `rate` | `20.0` | Frequência da pose (Hz) — é o ritmo do controle |
| `motor_rate` | `20.0` | Frequência de escrita nos motores (Hz) |
| `map_rate` | `0.5` | Frequência de releitura dos sinais do mapa (Hz) |
| `autostart` | `true` | Dar *play* se a simulação estiver parada |

Com a simulação rodando, cada chamada da Remote API só é atendida entre passos
e custa alguns milissegundos. Subir `rate` além de ~20 Hz não deixa o controle
mais rápido: só enfileira chamadas.

---

## 7. Problemas comuns

| Sintoma | O que fazer |
|---|---|
| `Não consegui falar com o CoppeliaSim em localhost:23000` | Abra o simulador; se já estiver aberto, algum script da cena está travando a thread principal |
| `sinais "banana" e "poop" ainda não existem` | Dê *play* na cena: quem os cria é o `/buildScene` |
| `Objeto "/myRobot/leftMotor" não existe na cena` | A ponte lista os objetos no terminal; ajuste `robot:=` |
| `estes scripts da cena escrevem nos motores` | Desabilite o `/myRobot/python_controler` |
| O robô anda para trás | Troque `motor_sign` para `1.0` |
| O robô gira em torno de si mesmo sem sair | Rumo invertido: confira o eixo da frente do robô na cena (aqui é o +y) |
| O robô para no meio do caminho | `cmd_timeout` expirou porque o navegador caiu; veja o terminal dele |
| O robô encosta nos poops | Aumente `k_rep` ou `d0_poop` |
| O robô dá voltas largas demais | Diminua `d0_poop`, ou aumente `k_att` |
| Trava antes da banana e fica indo e voltando | É mínimo local: diminua `stall_time` ou aumente `swirl_gain` |
| `Package 'potential_field' not found` | Faltou sourcear o workspace; no zsh tem que ser `install/setup.zsh` |

---

## 8. Relação com o script da pasta `projects/`

`projects/7 - obstacle_avoidance/scripts/PotentialFields.py` faz o mesmo
algoritmo em um arquivo só, para colar como child script na cena. Este pacote é
a versão ROS 2: mesma física, separada em nós, com os ganhos como parâmetros e
com o mapa, o alvo e a força visíveis como tópicos.
