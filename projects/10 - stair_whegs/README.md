# 10 — Rover de whegs que sobe escada

Troca de morfologia: em vez de rodas, **whegs** (*wheel-legs*) — rodas de três
raios curvos que rolam como roda no plano e engancham a quina do degrau como
perna na escada. O rover acha a escada, se alinha com ela, sobe os seis degraus
e para no patamar antes de cair do outro lado.

- **Cena:** [stair_whegs.ttt](stair_whegs.ttt) — o rover não existe no
  CoppeliaSim, ele é construído por script.
- **Código, documentação e enunciado:**
  [ros2/ros2_ws/src/stair_whegs/](../../ros2/ros2_ws/src/stair_whegs/)

## Montar a cena e rodar

```bash
cd ros2/ros2_ws
colcon build --symlink-install --packages-select stair_whegs
source install/setup.zsh

cd src/stair_whegs
python3 scene/build_scene.py            # monta e salva a cena
ros2 launch stair_whegs stair_whegs.launch.py
```

## O que este exercício acrescenta

| Conceito | Onde |
|---|---|
| Locomoção com pernas e **coordenação de fases** | `geometry.gait_targets`, `gait.py` |
| Troca de marcha em movimento, sem salto nas juntas | `geometry.rephase` |
| Giro por derrapagem (um wheg não esterça) | `geometry.gait_targets` |
| Percepção de terreno com sensores num corpo que balança | `climber.py` |
| Limiar **medido** em vez de escrito (estado CALIBRA) | `climber.do_calibra` |
| Máquina de estados com recuperação de falha | `climber.py` |
| Construção programática de um robô que não existe no simulador | `scene/build_scene.py` |

## O resultado que contraria a intuição

No degrau fácil as três marchas sobem. No degrau difícil (1,2 × o raio do wheg)
a marcha **alternada** — a boa para andar — **falha**, e a marcha com os quatro
whegs **em fase** sobe. Subir degrau alto não é andar: é um evento de força, e
as quatro garras atacando a quina ao mesmo tempo somam empuxo.

Os números, os outros ensaios e as decisões de projeto estão no
[README do pacote](../../ros2/ros2_ws/src/stair_whegs/README.md).
