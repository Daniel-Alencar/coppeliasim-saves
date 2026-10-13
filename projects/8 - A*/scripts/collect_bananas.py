#!/usr/bin/env python3
# =============================================================================
#  Coleta de todas as bananas com A* + controle de posição (cena a_star_banana)
#
#  Junta as três partes: o mapa de ocupação (grid_map.py), o planejador
#  (a_star.py) e um controlador de posição para o robô diferencial. O ciclo é
#
#      planeja A* até a banana pendente mais barata
#        -> segue os waypoints um a um
#        -> a cena recolhe a banana quando ela passa sob o /myRobot/poopximity
#        -> replaneja para a próxima
#
#  Quem decide que a banana foi coletada é a própria cena: o script /dirt_script
#  teleporta para z=1000 o que passar sob o sensor e soma um ponto no placar.
#  Esse mesmo script espalha poop se o robô pisar num — por isso o caminho é
#  planejado com inflação, e não em cima dos obstáculos.
#
#  O robô anda com a frente no eixo +y do referencial dele e as juntas montadas
#  invertidas (MOTOR_SIGN), igual à cena do campo potencial.
#
#  Como usar (com o CoppeliaSim aberto na cena):
#      /usr/bin/python3 collect_bananas.py
#      /usr/bin/python3 collect_bananas.py --resolution 0.15 --inflate 0.3
#      /usr/bin/python3 collect_bananas.py --tolerance 0.08
#      /usr/bin/python3 collect_bananas.py --no-diagonal --stop-at-end
# =============================================================================

import argparse
import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from a_star import a_star, nearest_free_cell, simplify
from grid_map import (arena_bounds, build_grid, cell_to_world, print_grid,
                      world_to_cell)
from scene_objects import BANANAS_PATH, POOPS_PATH, read_group, read_robot

ROBOT_PATH = '/myRobot'
LEFT_MOTOR_PATH = '/myRobot/leftMotor'
RIGHT_MOTOR_PATH = '/myRobot/rightMotor'

# ----------------------------- geometria -------------------------------------
WHEEL_RADIUS = 0.05          # raio da roda [m]
TRACK_WIDTH = 0.2            # distância entre as rodas [m]
MOTOR_SIGN = -1.0            # juntas montadas invertidas: negativo anda à frente

# ----------------------------- controle --------------------------------------
V_MAX = 0.25                 # velocidade de avanço [m/s]
W_MAX = 1.8                  # velocidade de giro [rad/s]
K_HEADING = 3.0              # rad/s por rad de erro de rumo
TURN_IN_PLACE = math.radians(70.0)   # acima disto, gira parado
SLOW_RADIUS = 0.30           # começa a desacelerar a esta distância do alvo [m]
V_MIN = 0.06                 # velocidade mínima na aproximação final [m/s]

# O sensor poopximity aponta para baixo no centro do robô: a banana só é
# recolhida quando passa por baixo dele, então o alvo final conta como atingido
# só bem em cima, não a 10 cm.
COLLECT_RADIUS = 0.05        # tolerância do último ponto (a banana) [m]
COLLECTED_Z = 100.0          # a cena teleporta a banana para z=1000

# Tempo máximo perseguindo uma banana antes de replanejar / desistir.
REPLAN_TIME = 25.0           # [s] de simulação
GIVE_UP_ATTEMPTS = 3         # replanejamentos antes de pular a banana


def wrap(angle):
    """Normaliza um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class DifferentialController:
    """Controle de posição: leva o robô a um ponto (x, y) do mundo.

    Primeiro alinha o rumo, depois avança; perto do alvo desacelera. É o mesmo
    esquema do controlador de posição usado nos exercícios anteriores, aqui
    alimentado pelos waypoints que saem do A*.
    """

    def __init__(self, sim):
        self.sim = sim
        self.robot = sim.getObject(ROBOT_PATH)
        self.left_motor = sim.getObject(LEFT_MOTOR_PATH)
        self.right_motor = sim.getObject(RIGHT_MOTOR_PATH)

    def pose(self):
        """(x, y, rumo) do robô no mundo; a frente é o eixo +y do robô."""
        m = self.sim.getObjectMatrix(self.robot, self.sim.handle_world)
        return m[3], m[7], math.atan2(m[5], m[1])

    def set_velocity(self, v, w):
        """Cinemática inversa: (m/s, rad/s) -> velocidade de cada roda."""
        half = TRACK_WIDTH / 2.0
        right = (v + w * half) / WHEEL_RADIUS
        left = (v - w * half) / WHEEL_RADIUS
        self.sim.setJointTargetVelocity(self.right_motor, MOTOR_SIGN * right)
        self.sim.setJointTargetVelocity(self.left_motor, MOTOR_SIGN * left)

    def stop(self):
        self.set_velocity(0.0, 0.0)

    def drive_to(self, target):
        """Um passo de controle rumo a (x, y). Devolve a distância que falta."""
        x, y, heading = self.pose()
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)

        error = wrap(math.atan2(dy, dx) - heading)
        w = max(-W_MAX, min(W_MAX, K_HEADING * error))

        if abs(error) > TURN_IN_PLACE:
            v = 0.0                              # erro grande: gira parado
        else:
            v = V_MAX
            if distance < SLOW_RADIUS:           # aproximação final
                v = max(V_MIN, V_MAX * distance / SLOW_RADIUS)
            v *= math.cos(error)                 # tortou, anda menos

        self.set_velocity(v, w)
        return distance


class BananaCollector:
    """Planeja com A* e dirige o robô até coletar todas as bananas."""

    def __init__(self, sim, controller, grid, info, diagonal, tolerance):
        self.sim = sim
        self.controller = controller
        self.grid = grid
        self.info = info
        self.diagonal = diagonal
        self.tolerance = tolerance
        self.collected = []
        self.skipped = []

    # ------------------------------------------------------------- cena

    def banana_position(self, banana):
        """Posição atual da banana, ou None se a cena já a recolheu."""
        position = self.sim.getObjectPosition(banana['handle'],
                                              self.sim.handle_world)
        return None if position[2] > COLLECTED_Z else position

    def goal_cell(self, position):
        """Célula livre que representa a banana no mapa, ou None."""
        cell = world_to_cell(self.info, position)
        if cell is None:
            return None
        return nearest_free_cell(self.grid, cell)

    # ------------------------------------------------------------- plano

    def plan(self, banana):
        """Caminho (lista de pontos do mundo) até a banana, ou None."""
        x, y, _ = self.controller.pose()
        start = world_to_cell(self.info, (x, y))
        if start is None:
            return None
        start = nearest_free_cell(self.grid, start)

        position = self.banana_position(banana)
        if position is None or start is None:
            return None
        goal = self.goal_cell(position)
        if goal is None:
            return None

        cells, stats = a_star(self.grid, start, goal, self.diagonal)
        if cells is None:
            return None

        points = [cell_to_world(self.info, r, c) for r, c in simplify(cells)]
        # O último ponto é a banana de verdade, não o centro da célula: é sobre
        # ela que o sensor precisa passar.
        points[-1] = (position[0], position[1])
        return points, cells, stats

    def choose_next(self, pending):
        """Banana pendente de menor custo A* a partir de onde o robô está."""
        best, best_plan, best_cost = None, None, math.inf
        for banana in pending:
            plan = self.plan(banana)
            if plan is None:
                continue
            _, _, stats = plan
            if stats['cost_cells'] < best_cost:
                best, best_plan, best_cost = banana, plan, stats['cost_cells']
        return best, best_plan

    # ------------------------------------------------------------- execução

    def follow(self, banana, points):
        """Segue os waypoints. Devolve True se a cena recolheu a banana."""
        index = 0
        started = self.sim.getSimulationTime()

        while index < len(points):
            if self.banana_position(banana) is None:
                return True                      # recolhida no meio do caminho

            last = index == len(points) - 1
            distance = self.controller.drive_to(points[index])
            if distance < (COLLECT_RADIUS if last else self.tolerance):
                index += 1

            if self.sim.getSimulationTime() - started > REPLAN_TIME:
                return False                     # demorou demais: replaneja

            self.sim.step()

        # Chegou ao fim da lista: pode ser que o sensor ainda não tenha pego a
        # banana por alguns milímetros. Insiste um pouco em cima dela.
        while self.sim.getSimulationTime() - started < REPLAN_TIME:
            if self.banana_position(banana) is None:
                return True
            self.controller.drive_to(points[-1])
            self.sim.step()
        return False

    def run(self, bananas):
        # Uma banana já recolhida (z=1000) antes de começar não entra na conta.
        pending = [b for b in bananas if self.banana_position(b) is not None]
        already = len(bananas) - len(pending)
        if already:
            print(f'{already} banana(s) já estavam recolhidas antes do início')
        attempts = {banana['path']: 0 for banana in bananas}

        while pending:
            banana, plan = self.choose_next(pending)
            if banana is None:
                print('\n[aviso] nenhuma das bananas restantes é alcançável: '
                      + ', '.join(b['path'] for b in pending))
                self.skipped.extend(pending)
                break

            points, cells, stats = plan
            distance = stats['cost_cells'] * self.info['resolution']
            print(f"\n-> {banana['path']}: {len(points)} waypoints, "
                  f"{distance:.2f} m, {stats['expanded']} nós expandidos")

            if self.follow(banana, points):
                self.collected.append(banana)
                pending.remove(banana)
                print(f"   coletada ({len(self.collected)}/{len(bananas)})")
                continue

            attempts[banana['path']] += 1
            if attempts[banana['path']] >= GIVE_UP_ATTEMPTS:
                print(f"   desistindo de {banana['path']} após "
                      f"{GIVE_UP_ATTEMPTS} tentativas")
                self.skipped.append(banana)
                pending.remove(banana)
            else:
                print('   não chegou a tempo, replanejando')

        self.controller.stop()


def main():
    parser = argparse.ArgumentParser(
        description='Faz o robô diferencial coletar todas as bananas usando '
                    'A* sobre o mapa de ocupação da cena')
    parser.add_argument('--resolution', type=float, default=0.2,
                        help='lado da célula em metros (padrão: 0.2)')
    parser.add_argument('--inflate', type=float, default=0.25,
                        help='margem de segurança ao redor dos poops [m]')
    parser.add_argument('--tolerance', type=float, default=0.12,
                        help='raio para considerar um waypoint alcançado [m]')
    parser.add_argument('--no-diagonal', action='store_true',
                        help='usa vizinhança de 4 em vez de 8')
    parser.add_argument('--show-map', action='store_true',
                        help='imprime o mapa de ocupação antes de começar')
    parser.add_argument('--stop-at-end', action='store_true',
                        help='para a simulação ao terminar')
    args = parser.parse_args()

    client = RemoteAPIClient()
    sim = client.require('sim')

    # A cena só recolhe bananas com a simulação rodando.
    if sim.getSimulationState() == sim.simulation_stopped:
        print('iniciando a simulação...')
        sim.startSimulation()

    # Passo a passo: cada iteração do laço é um passo de simulação, o que deixa
    # o controle determinístico e independente da velocidade da máquina.
    client.setStepping(True)

    poops = read_group(sim, POOPS_PATH)
    bananas = read_group(sim, BANANAS_PATH)
    robot = read_robot(sim)
    if robot is None:
        raise SystemExit('robô não encontrado na cena')

    # Os handles são guardados para acompanhar cada banana durante a corrida.
    for banana in bananas:
        banana['handle'] = sim.getObject(banana['path'])

    points = [o['position'] for o in poops + bananas] + [robot['position']]
    bounds = arena_bounds(sim, points)
    grid, info = build_grid(poops, bananas, robot, bounds,
                            args.resolution, args.inflate)

    print(f"{len(poops)} poops, {len(bananas)} bananas  |  mapa "
          f"{info['rows']}x{info['cols']} a {args.resolution} m/célula, "
          f"inflação {args.inflate} m")
    if args.show_map:
        print_grid(grid, info)

    controller = DifferentialController(sim)
    collector = BananaCollector(sim, controller, grid, info,
                                not args.no_diagonal, args.tolerance)
    try:
        collector.run(bananas)
    except KeyboardInterrupt:
        print('\ninterrompido')
    finally:
        controller.stop()
        sim.step()
        client.setStepping(False)

    print(f'\nColetadas {len(collector.collected)}/{len(bananas)} bananas '
          f'em {sim.getSimulationTime():.1f} s de simulação')
    if collector.skipped:
        print('Não coletadas: '
              + ', '.join(b['path'] for b in collector.skipped))

    if args.stop_at_end:
        sim.stopSimulation()


if __name__ == '__main__':
    main()
