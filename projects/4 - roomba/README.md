# 4 - roomba

Robô aspirador numa sala com móveis: o robô percorre o chão e um script da cena
vai recolhendo a sujeira por onde ele passa. Há **duas estratégias de cobertura**
implementadas, para comparar a mais ingênua com a sistemática.

| Controlador | Estratégia | Linhas |
|---|---|---|
| [`controllers/1 - python_controller.py`](controllers/1%20-%20python_controller.py) | reativa: anda reto, bate, recua e gira para um lado aleatório | ~77 |
| [`controllers/2 - python_controller.py`](controllers/2%20-%20python_controller.py) | varredura em faixas paralelas (*boustrophedon*), com contorno de obstáculos | ~650 |

Ambos são **child scripts em Python**: não se executam pelo terminal, são
colados dentro do script do robô na cena.

---

## 1. Como rodar

1. Abra o CoppeliaSim e carregue [`roomba style.ttt`](roomba%20style.ttt).
2. Na *Scene Hierarchy*, abra o script Python pendurado no robô.
3. Apague o conteúdo e cole um dos dois controladores.
4. Dê *play*.

Não é preciso terminal, ROS 2 nem a Remote API: tudo roda dentro do simulador.

> Só um controlador por vez. Os dois escrevem nos mesmos motores.

### O que a cena precisa ter

Os scripts procuram estes objetos por nome:

```
/leftMotor          (revolute joint, modo de velocidade)
/rightMotor         (revolute joint, modo de velocidade)
/proximitySensor    (sensor frontal)
```

O controlador `1` para com uma mensagem no console se não achar algum deles. O
controlador `2` usa `{'noError': True}` na busca e segue adiante, então confira
o console se o comportamento sair estranho.

### Arquivos de apoio

| Arquivo | Para que serve |
|---|---|
| `roomba style.ttt` | a cena |
| `model.ttm`, `dirty_floor.ttm`, `result.ttm` | modelos importáveis (robô, chão sujo, placar) |
| `*.jpg`, `*.png` | texturas do chão, em vários estágios de sujeira |
| `python_controller.py.bak` | backup de uma versão anterior; não é usado |

---

## 2. Controlador 1 — reativo

Uma máquina de três estados, sem memória e sem noção de onde o robô está:

```
FORWARD ──(obstáculo a ≤ 0,45 m)──▶ BACKWARD ──(0,3 s)──▶ TURN ──(0,8 a 1,8 s)──▶ FORWARD
```

O giro é para um lado sorteado (`random.choice([1, -1])`) e dura um tempo
sorteado. A cinemática inversa está em `set_motors(linVel, rotVel)`, que
converte velocidade do corpo em velocidade de roda.

Parâmetros no topo do arquivo:

| Constante | Valor | O que é |
|---|---|---|
| `wheelRadius` | `0.05` | raio da roda (m) |
| `trackWidth` | `0.2` | distância entre rodas (m) |
| `cruiseSpeed` | `0.4` | velocidade de cruzeiro (m/s) |
| `turnSpeed` | `1.2` | velocidade de giro (rad/s) |

**A limitação é o ponto da comparação:** como o robô não sabe onde já passou,
a cobertura é estatística. Ele limpa o quarto inteiro só se rodar por muito
tempo, e passa repetidas vezes pelo mesmo lugar enquanto deixa cantos intocados.

---

## 3. Controlador 2 — varredura em faixas

A estratégia sistemática, e é onde está o trabalho de verdade. O cabeçalho do
arquivo explica o algoritmo em detalhe; em resumo:

1. Anda reto até a primeira parede.
2. Gira até ficar paralelo a ela e a segue lado a lado — a primeira faixa.
3. Ao encontrar um obstáculo, contorna-o **sempre para o lado ainda não
   limpo**.
4. Quando o afastamento perpendicular à faixa chega a um diâmetro do robô,
   começa a faixa seguinte, paralela e em sentido oposto — a curva em "U" do
   cortador de grama.
5. Repete.

### Sensores criados em tempo de execução

A cena só tem o sensor frontal. O script **clona esse sensor** no início da
simulação, criando cópias nas laterais e nos cantos da frente, e as remove no
fim. Não é preciso alterar a cena — mas é por isso que a hierarquia ganha
objetos novos ao dar play.

### Ground truth × odometria

A constante `POSE_SOURCE` escolhe de onde vem a posição do robô:

```python
POSE_SOURCE = "ground_truth"   # pose exata, lida do simulador
POSE_SOURCE = "odometry"       # integrada dos encoders, como num robô real
```

O padrão é `ground_truth` por um motivo medido e registrado no código: nesta
cena as rodas deslizam muito nos giros no lugar. Em 120 s, os encoders
acusaram **44% a mais de rotação do que a real** e o rumo errou **mais de
100°** — o suficiente para entortar as faixas e destruir a varredura. Vale
trocar para `odometry` justamente para ver isso acontecer.

### Os casos difíceis

O cabeçalho do script documenta três situações que a estratégia pura não
resolve e que foram tratadas à parte: **obstáculo estreito** (o afastamento
nunca chega a um diâmetro e o robô giraria em volta dele para sempre), **fim da
sala** (o contorno traz o robô de volta sem que ele consiga se afastar) e
**armadilhas** como corredores curtos, móveis inclinados e cunhas entre móvel e
parede. Nesse último caso o robô gira para o lado mais livre, num ângulo
sorteado, e recomeça do passo 1.

### Principais parâmetros

| Grupo | Constantes |
|---|---|
| Geometria | `WHEEL_RADIUS`, `TRACK_WIDTH`, `MOTOR_SIGN`, `ROBOT_DIAMETER`, `LANE_SPACING` |
| Velocidades | `LANE_SPEED`, `CONTOUR_SPEED`, `TURN_RATE`, `MAX_ROT`, `BACK_SPEED` |
| Controle | `K_HEADING`, `K_WALL`, `K_WALL_D`, `K_WALL_ANGLE`, `WALL_MAX_ANGLE` |
| Limiares | `FRONT_STOP`, `SIDE_TARGET`, `SIDE_LOST`, `SIDE_MIN` |

O `LANE_SPACING` vale 0,30 m porque o script `/Dirt` da cena recolhe sujeira a
até 0,15 m do centro do robô: é a largura efetivamente limpa em cada passada, e
por isso também a distância entre faixas vizinhas.

O `SIDE_TARGET` de 0,22 m tem justificativa geométrica: girando no lugar, a
traseira do robô descreve um círculo de ~0,19 m em torno do eixo das rodas.
Manter-se acima disso garante que ele gire rente à parede sem raspar.

O `MOTOR_SIGN = -1` é a convenção desta cena, a mesma dos outros projetos do
repositório: velocidade negativa na junta faz o robô andar para a frente.

---

## 4. Problemas comuns

| Sintoma | O que fazer |
|---|---|
| `Erro ao carregar objetos` no console | Os nomes na *Scene Hierarchy* não batem com `/leftMotor`, `/rightMotor`, `/proximitySensor` |
| O robô anda para trás | Inverta o sinal em `set_motors` (controlador 1) ou `MOTOR_SIGN` (controlador 2) |
| As faixas saem tortas | `POSE_SOURCE = "odometry"` com as rodas deslizando; volte para `ground_truth` |
| O robô fica girando em volta de um móvel | Caso do obstáculo estreito; veja os limiares `RETURN_MIN_OFFSET` e `RETURN_MIN_ALONG` |
| Sobraram sensores extras na cena | O controlador 2 os remove no `sysCall_cleanup`; se a simulação foi interrompida à força, apague-os na mão |
