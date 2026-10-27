# 10 — Rover de rodas transformáveis que sobe escada

Troca de morfologia — e, desta vez, troca de morfologia **em movimento**. O
rover tem a roda transformável do **Wheel Transformer** (Kim, Jung, Kim, Cho e
Chu, *IEEE Transactions on Robotics*, 2014; patente US 9 073 587), que tem dois
estados:

| | Roda **fechada** | Roda **aberta** (garra) |
|---|---|---|
| forma | as três pernas emendam e fecham um **aro** | as pernas giram sobre o aro e abrem em **catavento** |
| alcance do contato | 50 mm (um raio) | 65 mm (1,30 raios) |
| balanço do corpo por passo | **zero** | 32 mm (metade do alcance) |
| degrau que vence | até ~1 raio | **70 mm**, 1,4 vez o aro |
| serve para | andar rápido e liso no plano | **enganchar a quina** do degrau |

No plano, uma roda de verdade: rola sem balanço. Contra o degrau, as pernas
abrem e cada uma vira uma garra que se apoia na **quina**. É a diferença que dá
sentido ao mecanismo: um wheg de raios fixos sobe degrau mas anda balançando o
tempo todo, inclusive no plano, onde não precisaria; um aro liso anda liso e
para mais cedo, porque só alcança a quina até mais ou menos um raio de altura.
A roda transformável não escolhe.

Onde as duas se separam está medido: num degrau de 55 mm (1,1 aro) a roda
fechada para em 3,6 degraus e a garra sobe os seis; num de 70 mm a fechada não
sai do lugar. Num degrau **fácil** de 40 mm as duas sobem, e a garra só é mais
rápida — por isso o rover só abre quando a sonda acusa o degrau.

No robô original a transformação é **passiva**: uma das três pernas é a perna
gatilho, o atrito dela contra o obstáculo a faz girar, e um disco transmissor
abre as outras duas. Aqui ela é **comandada** — o disco vira um acoplamento de
software e o gatilho de atrito vira o gatilho de **percepção** do `climber`,
que já enxerga o degrau pelas sondas. A simplificação está escrita no código,
em `geometry.py` e em `coppelia_bridge.py`.

- **Cena:** [stair_whegs.ttt](stair_whegs.ttt) — o rover não existe no
  CoppeliaSim, ele é construído por script.
- **Código, documentação e enunciado:**
  [ros2/ros2_ws/src/stair_whegs/](../../ros2/ros2_ws/src/stair_whegs/)

## Montar a cena e rodar

```bash
cd ros2/ros2_ws
colcon build --symlink-install --packages-select stair_whegs
source install/setup.bash

cd src/stair_whegs
python3 scene/build_scene.py
ros2 launch stair_whegs stair_whegs.launch.py
```

## O que este exercício acrescenta

| Conceito | Onde |
|---|---|
| **Morfologia que muda em tempo de execução** | `geometry.claw_tip`, `build_scene.add_wheel` |
| **Proporções do robô como consequência da morfologia** | `WHEELBASE`, `TRACK`, `tail_catch_angle` |
| **Marcha escolhida pelo modo de falha, não pelo empuxo** | `geometry.CLIMB_GAIT` |
| Mecanismo acoplado (três pernas, um disco) virando restrição de software | `coppelia_bridge.actuate_legs` |
| Locomoção com pernas e **coordenação de fases** | `geometry.gait_targets`, `gait.py` |
| Troca de marcha em movimento, sem salto nas juntas | `geometry.rephase` |
| Giro por derrapagem (uma roda de garras não esterça) | `geometry.gait_targets` |
| Percepção de terreno com sensores num corpo que balança | `climber.py` |
| Limiar **medido** em vez de escrito (estado CALIBRA) | `climber.do_calibra` |
| Máquina de estados com recuperação de falha | `climber.py` |
| Construção programática de um robô que não existe no simulador | `scene/build_scene.py` |

Os números, os ensaios e as decisões de projeto estão no
[README do pacote](../../ros2/ros2_ws/src/stair_whegs/README.md).
