import math
import numpy as np

deg2rad = math.pi / 180.0
rad2deg = 180.0 / math.pi

# Parâmetros Globais do APF
Kp = 1.2
q_goal = np.array([1.0, -2.0])
q_obstacles = [np.array([0.4, -0.8]), np.array([0.7, -1.4])]

DEBUG = True

def create_cylinder(sim, pos, radius=0.15, height=0.3, color=[1.0, 0.0, 0.0], alias="Cylinder"):
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

    # Torna o cilindro visível para o sensor de proximidade
    try:
        sim.setObjectSpecialProperty(
            shape_handle,
            sim.objectspecialproperty_collidable
            | sim.objectspecialproperty_measurable
            | sim.objectspecialproperty_detectable
            | sim.objectspecialproperty_renderable,
        )
    except Exception as e:
        print(f"Aviso: não foi possível marcar {alias} como detectável: {e}")

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

def compute_forces(q, q_goal, q_obstacles):
    k_attractive = 1.0
    k_repulsive = 1.5
    d_safe = 0.8

    # 1. Força Atrativa (do robô para o objetivo)
    f_attractive = k_attractive * (q_goal - q)

    # 2. Força Repulsiva (do obstáculo para o robô)
    f_repulsive = []
    for q_obs in q_obstacles:
        delta = q - q_obs
        dist = np.linalg.norm(delta)
        if dist <= d_safe and dist > 0.01:
            f_rep = k_repulsive * (1.0 / dist - 1.0 / d_safe) * (delta / (dist ** 2))
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

    # Criar Objetivo (Verde) e Obstáculos (Vermelhos) na cena
    goal_handle = create_cylinder(sim, q_goal, radius=0.1, height=0.4, color=[0.0, 1.0, 0.0], alias="Goal_Cylinder")
    if goal_handle:
        self.spawned_objects.append(goal_handle)

    for i, obs_pos in enumerate(q_obstacles):
        obs_handle = create_cylinder(sim, obs_pos, radius=0.15, height=0.3, color=[1.0, 0.0, 0.0], alias=f"Obstacle_Cylinder_{i}")
        if obs_handle:
            self.spawned_objects.append(obs_handle)

    print("Simulação iniciada. Navegação autônoma por APF ativa!")

def sysCall_sensing():
    global L, max_linVel, max_rotVel, wheelradius

    # 1. Posição do robô e direção para onde a frente (sensor) aponta
    robot_pos_3d = sim.getObjectPosition(self.robotHandle, -1)
    q_robot = np.array([robot_pos_3d[0], robot_pos_3d[1]])

    sensor_matrix = sim.getObjectMatrix(self.proximitySensorHandle, -1)
    front = z_axis(sensor_matrix)[:2]
    theta_front = math.atan2(front[1], front[0])

    # 2. Obstáculos conhecidos + Leitura dinâmica pelo Sensor de Proximidade
    current_obstacles = list(q_obstacles)
    state, dist, detectedPoint, _, _ = sim.readProximitySensor(self.proximitySensorHandle)
    if state == 1:
        obs_world = to_world(sensor_matrix, detectedPoint)
        current_obstacles.append(obs_world[:2])

    # 3. Cálculo da distância ao Objetivo
    dist_to_goal = np.linalg.norm(q_robot - q_goal)

    if dist_to_goal > 0.15:  # Raio de parada no destino
        f = compute_forces(q_robot, q_goal, current_obstacles)

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
                  f"erro={e_theta * rad2deg:6.1f}° v={linVel:.3f} w={rotVel:.3f} sensor={state}")
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
