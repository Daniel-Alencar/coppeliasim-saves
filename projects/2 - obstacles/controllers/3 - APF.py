import math
import numpy as np

deg2rad = math.pi / 180.0
rad2deg = 180.0 / math.pi

# --- Parâmetros do APF ------------------------------------------------------
Kp = 1.2
K_ATTRACTIVE = 1.0
K_REPULSIVE = 3.5
# raio de influência de cada obstáculo
D_SAFE = 1.0                  

DEBUG = True

# Duas detecções a menos desta distância (m) são tratadas como o mesmo obstáculo
OBSTACLE_MERGE_DIST = 0.25

# --- Geração aleatória do cenário -------------------------------------------
# Objetivo e obstáculos caem em posições novas a cada execução da simulação.
# troque por um inteiro para repetir um cenário
RANDOM_SEED = None            
N_OBSTACLES = 15

# limites (x, y) do sorteio
ARENA_MIN = np.array([-2.0, -2.0])  
ARENA_MAX = np.array([2.0, 2.0])

OBSTACLE_RADIUS = 0.15
GOAL_RADIUS = 0.1

MIN_GOAL_DIST = 1.5           # objetivo x robô: garante um percurso a percorrer
MIN_ROBOT_CLEARANCE = 0.6     # obstáculo x robô: o robô não nasce dentro de um
MIN_OBSTACLE_GAP = 0.6        # obstáculo x obstáculo: evita cilindros sobrepostos
MIN_GOAL_GAP = D_SAFE + 0.1   # obstáculo x objetivo: mantém o objetivo alcançável
MAX_SAMPLING_ATTEMPTS = 500

def create_cylinder(sim, pos, radius=0.15, height=0.3, color=[1.0, 0.0, 0.0], alias="Cylinder",
                    detectable=True):
    """ Cria cilindros 3D dinamicamente na cena durante a simulação """
    try:
        sizes = [radius * 2, radius * 2, height]
        res = sim.createPrimitiveShape(2, sizes, 0)
        shape_handle = res[0] if isinstance(res, (list, tuple)) else res

        sim.setObjectAlias(shape_handle, alias)
        sim.setObjectPosition(shape_handle, -1, [pos[0], pos[1], height / 2.0])
    except Exception as e:
        print(f"Erro ao criar cilindro {alias}: {e}")
        return None

    try:
        sim.setShapeColor(shape_handle, None, sim.colorcomponent_ambient_diffuse, color)
    except Exception:
        try:
            sim.setObjectColor(shape_handle, 0, sim.colorcomponent_ambient_diffuse, color)
        except Exception:
            pass

    try:
        sim.setObjectInt32Param(shape_handle, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(shape_handle, sim.shapeintparam_respondable, 1)
    except Exception:
        pass

    # Define se o cilindro existe ou não para o sensor de proximidade
    try:
        props = (sim.objectspecialproperty_collidable
                 | sim.objectspecialproperty_measurable
                 | sim.objectspecialproperty_renderable)
        if detectable:
            props |= sim.objectspecialproperty_detectable
        sim.setObjectSpecialProperty(shape_handle, props)
    except Exception as e:
        print(f"Aviso: não foi possível ajustar a detectabilidade de {alias}: {e}")

    return shape_handle

def z_axis(matrix):
    """ Eixo Z (3a coluna) de uma matriz 3x4 do CoppeliaSim, em coordenadas do mundo """
    return np.array([matrix[2], matrix[6], matrix[10]])

def to_world(matrix, p):
    """ Aplica a matriz 3x4 completa (rotação + translação) a um ponto local """
    return np.array([
        matrix[0] * p[0] + matrix[1] * p[1] + matrix[2] * p[2] + matrix[3],
        matrix[4] * p[0] + matrix[5] * p[1] + matrix[6] * p[2] + matrix[7],
        matrix[8] * p[0] + matrix[9] * p[1] + matrix[10] * p[2] + matrix[11],
    ])

def travel_dir(sim, motor_handle):
    """
    Direção (xy, mundo) em que o corpo avança quando este motor recebe velocidade
    positiva. Para uma roda de eixo 'a' tocando o solo: v_corpo = w * R * (a x up).
    """
    axis = z_axis(sim.getObjectMatrix(motor_handle, -1))
    return np.cross(axis, [0.0, 0.0, 1.0])[:2]

def sample_point(rng, is_valid):
    """ Sorteia um ponto na arena que satisfaça is_valid; None se não conseguir """
    for _ in range(MAX_SAMPLING_ATTEMPTS):
        p = rng.uniform(ARENA_MIN, ARENA_MAX)
        if is_valid(p):
            return p
    return None

def sample_scene(rng, robot_xy):
    """
    Sorteia o objetivo e os obstáculos por amostragem com rejeição.

    Nenhum obstáculo pode cair a menos de MIN_GOAL_GAP do objetivo: a atração
    tende a zero ao chegar no destino, enquanto a repulsão continua finita, então
    um obstáculo dentro do raio de influência D_SAFE deixaria de existir um mínimo
    do potencial sobre o objetivo e o robô nunca conseguiria pousar nele.
    """
    goal = sample_point(rng, lambda p: np.linalg.norm(p - robot_xy) >= MIN_GOAL_DIST)
    if goal is None:
        raise RuntimeError("Arena pequena demais para MIN_GOAL_DIST: nenhum objetivo válido.")

    obstacles = []
    for _ in range(N_OBSTACLES):
        def is_valid(p):
            return (np.linalg.norm(p - robot_xy) >= MIN_ROBOT_CLEARANCE
                    and np.linalg.norm(p - goal) >= MIN_GOAL_GAP
                    and all(np.linalg.norm(p - o) >= MIN_OBSTACLE_GAP for o in obstacles))

        p = sample_point(rng, is_valid)
        if p is None:
            print(f"Aviso: só couberam {len(obstacles)} obstáculos com as folgas atuais.")
            break
        obstacles.append(p)

    return goal, obstacles

def compute_forces(q, q_goal, q_obstacles):
    # 1. Força Atrativa (do robô para o objetivo)
    f_attractive = K_ATTRACTIVE * (q_goal - q)

    # 2. Força Repulsiva (do obstáculo para o robô)
    f_repulsive = []
    for q_obs in q_obstacles:
        delta = q - q_obs
        dist = np.linalg.norm(delta)
        if dist <= D_SAFE and dist > 0.01:
            f_rep = K_REPULSIVE * (1.0 / dist - 1.0 / D_SAFE) * (delta / (dist ** 2))
            f_repulsive.append(f_rep)

    if len(f_repulsive) > 0:
        f_rep_total = np.sum(f_repulsive, axis=0)
    else:
        f_rep_total = np.array([0.0, 0.0])

    return f_attractive + f_rep_total

def sysCall_init():
    sim = require('sim')

    # Handles dos componentes do robô
    self.robotHandle = sim.getObject(".")
    self.rightMotorHandle = sim.getObject("/rightMotor")
    self.leftMotorHandle = sim.getObject("/leftMotor")
    self.proximitySensorHandle = sim.getObject("/proximitySensor")

    # Constantes físicas do robô
    global L, max_linVel, max_rotVel, wheelradius
    wheelradius = 0.05
    L = 0.2
    max_linVel = 0.4
    max_rotVel = 90 * deg2rad

    # --- Calibração do referencial de navegação --------------------------------
    # A "frente" do robô é a direção de detecção do sensor de proximidade
    # (eixo +Z local dele), e não o eixo +X do corpo.
    front = z_axis(sim.getObjectMatrix(self.proximitySensorHandle, -1))[:2]
    norm = np.linalg.norm(front)
    if norm < 1e-6:
        raise RuntimeError("O sensor de proximidade aponta para cima/baixo: verifique a montagem.")
    front = front / norm

    # Sinal de cada roda: +1 se velocidade positiva empurra o robô no sentido da frente.
    self.sign_right = 1.0 if np.dot(travel_dir(sim, self.rightMotorHandle), front) > 0 else -1.0
    self.sign_left = 1.0 if np.dot(travel_dir(sim, self.leftMotorHandle), front) > 0 else -1.0

    # Confere se o motor chamado "rightMotor" está mesmo à direita da frente.
    left_dir = np.array([-front[1], front[0]])  # up x front
    rob_xy = np.array(sim.getObjectPosition(self.robotHandle, -1))[:2]
    right_xy = np.array(sim.getObjectPosition(self.rightMotorHandle, -1))[:2]
    if np.dot(right_xy - rob_xy, left_dir) > 0:
        self.rightMotorHandle, self.leftMotorHandle = self.leftMotorHandle, self.rightMotorHandle
        self.sign_right, self.sign_left = self.sign_left, self.sign_right
        print("Calibração: papéis dos motores esquerdo/direito trocados.")

    print(f"Calibração: frente={front.round(3)}, sinal_dir={self.sign_right}, sinal_esq={self.sign_left}")

    self.spawned_objects = []

    # Mapa construído em tempo de execução: só entra aqui o que o sensor já viu
    self.discovered_obstacles = []

    # --- Sorteio do cenário ----------------------------------------------------
    # A seed é impressa para que um cenário problemático possa ser reproduzido.
    seed = RANDOM_SEED if RANDOM_SEED is not None else int(np.random.SeedSequence().entropy % (2 ** 32))
    rng = np.random.default_rng(seed)
    self.q_goal, obstacles = sample_scene(rng, rob_xy)

    print(f"Cenário sorteado (seed={seed}; fixe RANDOM_SEED para repeti-lo)")
    print(f"  Objetivo: {self.q_goal.round(2)}")

    # Objetivo (Verde): marcador visual, invisível para o sensor de proximidade
    self.goal_handle = create_cylinder(sim, self.q_goal, radius=GOAL_RADIUS, height=0.4,
                                       color=[0.0, 1.0, 0.0], alias="Goal_Cylinder",
                                       detectable=False)
    if self.goal_handle:
        self.spawned_objects.append(self.goal_handle)

    # Obstáculos (Vermelhos): detectáveis, é o que o robô vai descobrir
    for i, obs_pos in enumerate(obstacles):
        obs_handle = create_cylinder(sim, obs_pos, radius=OBSTACLE_RADIUS, height=0.3,
                                     color=[1.0, 0.0, 0.0], alias=f"Obstacle_Cylinder_{i}")
        if obs_handle:
            self.spawned_objects.append(obs_handle)
        print(f"  Obstáculo {i}: {obs_pos.round(2)}")

    print("Simulação iniciada. Navegação autônoma por APF ativa!")

def sysCall_sensing():
    global L, max_linVel, max_rotVel, wheelradius

    # 1. Posição do robô e direção para onde a frente (sensor) aponta
    robot_pos_3d = sim.getObjectPosition(self.robotHandle, -1)
    q_robot = np.array([robot_pos_3d[0], robot_pos_3d[1]])

    sensor_matrix = sim.getObjectMatrix(self.proximitySensorHandle, -1)
    front = z_axis(sensor_matrix)[:2]
    theta_front = math.atan2(front[1], front[0])

    # 2. Mapeamento incremental: nenhum obstáculo é conhecido a priori
    current_obstacles = list()

    state, dist, detectedPoint, detectedHandle, _ = sim.readProximitySensor(self.proximitySensorHandle)
    # O objetivo já é criado como não detectável; o teste de handle só protege o
    # caso de setObjectSpecialProperty falhar nesta versão do CoppeliaSim.
    if state == 1 and detectedHandle != self.goal_handle:
        obs_world = to_world(sensor_matrix, detectedPoint)[:2]
        # Só registra se for um ponto novo, senão o mapa cresce a cada passo
        # e a repulsão do mesmo obstáculo seria somada centenas de vezes.
        if all(np.linalg.norm(obs_world - known) > OBSTACLE_MERGE_DIST
               for known in self.discovered_obstacles):
            self.discovered_obstacles.append(obs_world)
            print(f"Obstáculo descoberto em {obs_world.round(2)} "
                  f"(total conhecido: {len(self.discovered_obstacles)})")

    # Tudo que já foi descoberto continua repelindo, mesmo fora do campo de visão
    current_obstacles.extend(self.discovered_obstacles)

    # 3. Cálculo da distância ao Objetivo
    dist_to_goal = np.linalg.norm(q_robot - self.q_goal)

    if dist_to_goal > 0.15:  # Raio de parada no destino
        f = compute_forces(q_robot, self.q_goal, current_obstacles)

        # A frente do robô deve apontar para a resultante das forças
        theta_des = math.atan2(f[1], f[0])

        # Normalização do erro no intervalo [-pi, pi]
        e_theta = math.atan2(math.sin(theta_des - theta_front), math.cos(theta_des - theta_front))
        f_norm = np.linalg.norm(f)

        # Controlador P para velocidade angular e ajuste de velocidade linear
        rotVel = max(-max_rotVel, min(max_rotVel, Kp * e_theta))
        linVel = max_linVel * max(0.0, math.cos(e_theta)) * math.tanh(f_norm)

        if DEBUG:
            print(f"theta_front={theta_front * rad2deg:6.1f}° theta_des={theta_des * rad2deg:6.1f}° "
                  f"erro={e_theta * rad2deg:6.1f}° v={linVel:.3f} w={rotVel:.3f} "
                  f"obstaculos={len(current_obstacles)}")
    else:
        linVel = 0.0
        rotVel = 0.0

    # 4. Cinemática Inversa e Aplicação nas Rodas
    rightVel = linVel + (L / 2.0) * rotVel
    leftVel = linVel - (L / 2.0) * rotVel

    sim.setJointTargetVelocity(self.rightMotorHandle, self.sign_right * rightVel / wheelradius)
    sim.setJointTargetVelocity(self.leftMotorHandle, self.sign_left * leftVel / wheelradius)

def sysCall_actuation():
    pass

def sysCall_cleanup():
    """ Limpeza dos objetos da cena ao encerrar a simulação """
    try:
        for handle in self.spawned_objects:
            sim.removeObject(handle)
    except Exception:
        pass
