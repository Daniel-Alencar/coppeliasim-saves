# Estrutura de um projeto ROS 2

Guia da organização de um projeto ROS 2, usando como exemplo o workspace deste
repositório (`ros2/ros2_ws`), que integra o CoppeliaSim com nós em Python.

---

## 1. Os três níveis: workspace → pacote → nó

ROS 2 organiza o código em três níveis encaixados:

| Nível | O que é | Exemplo aqui |
|---|---|---|
| **Workspace** | Pasta que você compila de uma vez com `colcon`. Agrupa pacotes. | `ros2/ros2_ws` |
| **Pacote** | Unidade mínima de build, instalação e distribuição. Tem nome, versão, dependências e licença. | `car_control`, `diff_robot` |
| **Nó** | Processo (ou objeto dentro de um processo) que fala no grafo ROS: publica/assina tópicos, oferece serviços, tem parâmetros. | `car_node`, `motor_publisher_node` |

Um pacote pode conter vários nós, e um processo pode rodar vários nós juntos —
é o que `car_control.py` faz, com um `MultiThreadedExecutor` girando `car_node`
e `motor_publisher_node` no mesmo processo.

---

## 2. Anatomia do workspace

```
ros2_ws/
├── src/        ← o SEU código (a única pasta versionada)
│   ├── car_control/
│   ├── diff_robot/
│   └── sim_ros2_interface/
├── build/      ← artefatos intermediários do colcon (descartável)
├── install/    ← resultado final, o que o ROS realmente carrega (descartável)
└── log/        ← logs de cada invocação do colcon (descartável)
```

Apenas `src/` entra no Git. As outras três são geradas por `colcon build` e por
isso estão no [.gitignore](.gitignore) do repositório:

```
build/
install/
log/
```

Regra prática: se apagar `build/`, `install/` e `log/` e um `colcon build`
reconstruir tudo, o workspace está saudável.

### Fluxo de trabalho

```bash
cd ros2/ros2_ws

# 1. instala dependências declaradas nos package.xml (opcional, mas recomendado)
rosdep install --from-paths src --ignore-src -r -y

# 2. compila
colcon build --symlink-install

# 3. "entra" no workspace: coloca os pacotes no PATH/PYTHONPATH da shell
source install/setup.bash

# 4. roda
ros2 launch car_control car_control.launch.py
```

Dois detalhes que economizam horas:

- **`source install/setup.bash` vale só para aquela shell.** Toda aba nova de
  terminal precisa repetir. O `setup.bash` do workspace já encadeia o do ROS
  (`/opt/ros/<distro>/setup.bash`), chamado de *underlay*.
- **`--symlink-install`** faz `install/` apontar para os arquivos em `src/` por
  link simbólico em vez de copiar. Com isso, editar um `.py` já vale na próxima
  execução, sem recompilar. Mudanças em `setup.py`, `package.xml` ou em
  arquivos novos ainda exigem `colcon build`.

---

## 3. Anatomia de um pacote Python (`ament_python`)

É o tipo usado pelos dois pacotes escritos aqui. Estrutura de `car_control`:

```
car_control/
├── package.xml                    ← metadados e dependências (obrigatório)
├── setup.py                       ← como instalar (obrigatório em ament_python)
├── setup.cfg                      ← onde colocar os executáveis
├── resource/car_control           ← arquivo VAZIO que registra o pacote no índice
├── car_control/                   ← o módulo Python (mesmo nome do pacote)
│   ├── __init__.py
│   └── car_control.py             ← implementação dos nós
├── launch/
│   └── car_control.launch.py      ← como subir tudo de uma vez
└── coppeliasim/
    └── my_robot_ros2.lua          ← recurso extra (child script do simulador)
```

A duplicação de nome (`car_control/car_control/`) é intencional: a pasta
externa é o **pacote ROS**, a interna é o **pacote Python** importável.

### 3.1 `package.xml` — o cartão de identidade

Lido pelo ROS e pelo `rosdep`. É o que torna a pasta um pacote de verdade.

```xml
<package format="3">
  <name>car_control</name>
  <version>0.0.1</version>
  <description>Controle do robô diferencial do CoppeliaSim...</description>
  <maintainer email="...">daniel.carvalho</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <test_depend>ament_flake8</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

Tipos de dependência que importam:

| Tag | Quando usar |
|---|---|
| `<depend>` | precisa em build e em execução (caso comum em C++) |
| `<build_depend>` | só para compilar |
| `<exec_depend>` | só para rodar — típico de pacotes Python |
| `<test_depend>` | só nos testes |

O `<build_type>` dentro de `<export>` decide quem compila o pacote:
`ament_python` (setuptools) ou `ament_cmake` (CMake).

### 3.2 `setup.py` — o que vai parar em `install/`

```python
data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ('share/' + package_name + '/coppeliasim', glob('coppeliasim/*.lua')),
],
entry_points={
    'console_scripts': [
        'car_control = car_control.car_control:main',
    ],
},
```

Duas responsabilidades:

1. **`data_files`** copia arquivos que não são código Python. Esquecer de listar
   a pasta `launch/` aqui é o erro nº 1 de quem começa: `ros2 launch` responde
   *"file not found"* porque o arquivo existe em `src/`, mas nunca chegou a
   `install/share/`. O mesmo vale para `.lua`, `.yaml`, URDF e meshes.
2. **`entry_points`/`console_scripts`** cria os executáveis. A linha
   `'car_control = car_control.car_control:main'` significa:
   `ros2 run car_control car_control` → chama `main()` do módulo
   `car_control/car_control.py`. `diff_robot` declara dois:
   `coppelia_bridge` e `dummy_driver`.

### 3.3 `setup.cfg` e `resource/`

```ini
[develop]
script_dir=$base/lib/car_control
[install]
install_scripts=$base/lib/car_control
```

Isso põe os executáveis em `install/car_control/lib/car_control/`, que é
exatamente onde o `ros2 run` procura. Sem isso, o comando não encontra o nó.

O arquivo `resource/car_control` é vazio de propósito: ao ser copiado para
`share/ament_index/resource_index/packages/`, ele é o que faz o pacote aparecer
em `ros2 pkg list`.

### 3.4 Estrutura de um nó

```python
class CarNode(Node):
    def __init__(self, command):
        super().__init__('car_node')                      # nome no grafo
        self.declare_parameter('wheel_radius', 0.05)      # parâmetro configurável
        self.create_subscription(Twist, 'turtle1/cmd_vel', self.on_cmd_vel, 10)
        self.pub = self.create_publisher(Float32, 'my_robot/left_motor', 10)
```

Os quatro blocos que aparecem em quase todo nó: **nome**, **parâmetros**,
**assinaturas** e **publicações** — mais *timers* quando há trabalho periódico,
como o `motor_publisher_node`, que republica a velocidade em frequência fixa.

---

## 4. Pacote C++ (`ament_cmake`) — para comparação

`sim_ros2_interface`, o plugin do CoppeliaSim clonado dentro de `src/`, segue o
outro padrão:

```
sim_ros2_interface/
├── package.xml            ← <build_type>ament_cmake</build_type>
├── CMakeLists.txt         ← substitui o setup.py
├── src/                   ← fontes .cpp
├── include/               ← cabeçalhos .h
├── meta/ templates/ lua/  ← recursos do plugin
└── examples/ tests/
```

Resumo das diferenças:

| | `ament_python` | `ament_cmake` |
|---|---|---|
| Receita de build | `setup.py` + `setup.cfg` | `CMakeLists.txt` |
| Código em | `<pacote>/<pacote>/*.py` | `src/*.cpp` + `include/` |
| Executável | `console_scripts` | `add_executable()` + `install(TARGETS ...)` |
| Recompilar após editar | não (com `--symlink-install`) | sim, sempre |

Pacotes que **definem mensagens, serviços ou actions** (`msg/`, `srv/`,
`action/`) também usam `ament_cmake`, mesmo que os nós que as consomem sejam em
Python — é a convenção habitual: um pacote `*_interfaces` só com as definições,
e os pacotes de nós dependendo dele.

---

## 5. Launch files — subindo o sistema

Um robô raramente é um nó só. O launch file descreve o conjunto:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='car_control',
            namespace='car_control',
            executable='car_control',   # o nome do console_script
            output='screen'
        ),
    ])
```

O `namespace` prefixa todos os tópicos do nó. Por isso, neste projeto, o que o
código chama de `turtle1/cmd_vel` aparece no grafo como
`/car_control/turtle1/cmd_vel`, e os motores como
`/car_control/my_robot/left_motor` e `.../right_motor`. É o mecanismo que
permite rodar dois robôs iguais sem colisão de nomes.

Também é no launch file que se passam parâmetros (`parameters=[{...}]` ou um
arquivo YAML), remapeamentos (`remappings=[('/entrada', '/outra')]`) e
argumentos de linha de comando (`DeclareLaunchArgument`).

Convenção de nome: `*.launch.py` (também existem as formas XML e YAML).

---

## 6. Onde cada coisa vai parar depois do build

```
install/car_control/
├── lib/car_control/car_control          ← executável (ros2 run acha aqui)
├── lib/python3.X/site-packages/car_control/   ← o módulo Python
└── share/car_control/
    ├── package.xml
    ├── launch/car_control.launch.py     ← ros2 launch acha aqui
    └── coppeliasim/my_robot_ros2.lua
```

Mapa mental: **`lib/` é código executável, `share/` é tudo que não é código.**
Se um arquivo não está em `install/`, para o ROS ele não existe — foi isso que
o `data_files` do `setup.py` resolveu.

---

## 7. Comandos de referência

```bash
# criar um pacote novo já com a estrutura correta
ros2 pkg create --build-type ament_python --license MIT meu_pacote

# compilar só um pacote (e suas dependências)
colcon build --packages-select car_control
colcon build --packages-up-to car_control

# inspecionar o que está rodando
ros2 pkg list                  # pacotes visíveis na shell atual
ros2 node list                 # nós ativos
ros2 topic list                # tópicos
ros2 topic echo /car_control/my_robot/left_motor
ros2 topic info /car_control/turtle1/cmd_vel --verbose
ros2 param list /car_control/car_node
ros2 run rqt_graph rqt_graph   # visualizar o grafo

# executar
ros2 run car_control car_control
ros2 launch car_control car_control.launch.py
```

---

## 8. Erros frequentes e o que checar

| Sintoma | Causa provável |
|---|---|
| `Package 'x' not found` | faltou `source install/setup.bash` nesta shell, ou o build falhou |
| `ros2 launch` não acha o arquivo | `launch/` não está no `data_files` do `setup.py` |
| `No executable found` | falta a entrada em `console_scripts`, ou o `setup.cfg` não aponta para `lib/<pacote>` |
| Editei o `.py` e nada mudou | build sem `--symlink-install`, ou alteração em `setup.py`/`package.xml` (aí recompile) |
| Os nós não se veem | namespaces diferentes, `ROS_DOMAIN_ID` diferente, ou QoS incompatível |
| Build quebrado sem motivo aparente | apague `build/ install/ log/` e recompile do zero |

---

## 9. Resumo em uma frase

`src/` guarda pacotes; cada pacote se descreve em `package.xml` e diz como se
instalar em `setup.py`/`CMakeLists.txt`; `colcon build` transforma isso em
`install/`; `source install/setup.bash` torna esse resultado visível para a
shell; e `ros2 run`/`ros2 launch` sobem os nós que conversam por tópicos.
