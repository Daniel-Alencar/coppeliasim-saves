# 9 — youBot: pegar e depositar a banana

Trabalho final. O robô deixa de apenas **chegar** até a banana e passa a
**pegá-la**: um manipulador móvel (KUKA youBot) planeja com A\*, navega com a
base holonômica, fecha a garra na banana com cinemática inversa do braço de 5
juntas e deposita numa bandeja.

- **Cena:** [youbot_banana.ttt](youbot_banana.ttt) — gerada por script, não
  montada à mão.
- **Código, documentação e enunciado:**
  [ros2/ros2_ws/src/youbot_banana/](../../ros2/ros2_ws/src/youbot_banana/)

## Montar a cena

A cena é reproduzível a partir de uma semente. Com o CoppeliaSim aberto:

```bash
cd ros2/ros2_ws/src/youbot_banana
python3 scene/build_scene.py              # a cena padrão (semente 1)
python3 scene/build_scene.py --seed 7     # outro cenário
```

## Rodar

```bash
cd ros2/ros2_ws
colcon build --symlink-install --packages-select a_star_banana youbot_banana
source install/setup.zsh
ros2 launch youbot_banana youbot_banana.launch.py
```

## O que este exercício acrescenta

| Conceito | Onde |
|---|---|
| Cinemática inversa de base **holonômica** (4 rodas suecas) | `kinematics.wheel_velocities` |
| Controle de posição sem a restrição não-holonômica | `base_control.py` |
| Cinemática **direta e inversa** de manipulador (5 juntas) | `kinematics.arm_fk`, `arm_ik` |
| Envelope de alcance e limites de junta | `scene/measure_arm.py` |
| Trajetória cartesiana interpolada | `arm_control.plan_path` |
| Preensão física, com detecção por travamento da garra | `arm_control.check_grasp` |
| Máquina de estados de uma tarefa completa | `mission.py` |
| A\* e mapa de ocupação | **reaproveitados** do projeto 8 |

A discussão dos resultados, as medidas feitas no simulador e os cinco erros que
o projeto deu (com o que cada um ensinou) estão no
[README do pacote](../../ros2/ros2_ws/src/youbot_banana/README.md).
