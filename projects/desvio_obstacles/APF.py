import math
import numpy as np

deg2rad = math.pi / 180.0
rad2deg = 180.0 / math.pi

# Parâmetros Globais do APF
Kp = 1.2
q_goal = np.array([1.0, -2.0])
q_obstacles = [np.array([0.4, -0.8]), np.array([0.7, -1.4])]

def create_cylinder(sim, pos, radius=0.15, height=0.3, color=[1.0, 0.0, 0.0], alias="Cylinder"):
    """ Cria cilindros 3D dinamicamente na cena durante a simulação """
    try:
        sizes = [radius * 2, radius * 2, height]
        res = sim.createPrimitiveShape(2, sizes, 0)
        shape_handle = res[0] if isinstance(res, (list, tuple)) else res

        sim.setObjectAlias(shape_handle, alias)
        sim.setObjectPosition(shape_handle, -1, [pos[0], pos[1], height / 2.0])

        try:
            sim.setShapeColor(shape_handle, None, sim.colorcomponent_ambient_diffuse, color)
        except:
            try:
                sim.setObjectColor(shape_handle, 0, sim.colorcomponent_ambient_diffuse, color)
            except:
                pass

        sim.setObjectInt32Param(shape_handle, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(shape_handle, sim.shapeintparam_respondable, 1)

        return shape_handle
    except Exception as e:
        print(f"Erro ao criar cilindro {alias}: {e}")
        return None

def compute_forces(q, q_goal, q_obstacles):
    k_attractive = 1.0
    k_repulsive = 1.5
    d_safe = 0.8

    # 1. Força Atrativa (do robô para o objetivo)
    f_attractive = k_attractive * (q_goal - q)

    # 2. Força Repulsiva (do obstáculo para o robô)
    f_repulsive = []
    for q_obs in q_obstacles:
        dist = np.linalg.norm(q - q_obs)
        if dist <= d_safe and dist > 0.01:
            f_rep = k_repulsive * (1.0 / dist - 1.0 / d_safe) * ((q - q_obs) / (dist ** 2))
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

    # 1. Leitura da Posição e Orientação do Robô no espaço 2D
    robot_pos_3d = sim.getObjectPosition(self.robotHandle, -1)
    robot_ori_3d = sim.getObjectOrientation(self.robotHandle, -1)
    
    q_robot = np.array([robot_pos_3d[0], robot_pos_3d[1]])
    theta_robot = robot_ori_3d[2] # Gamma (-pi a +pi)

    # 2. Obstáculos conhecidos + Leitura dinâmica pelo Sensor de Proximidade
    current_obstacles = list(q_obstacles)
    state, dist, detectedPoint, _, _ = sim.readProximitySensor(self.proximitySensorHandle)
    if state == 1:
        matrix = sim.getObjectMatrix(self.proximitySensorHandle, -1)
        sensor_obs_x = matrix[3] + detectedPoint[0]
        sensor_obs_y = matrix[7] + detectedPoint[1]
        current_obstacles.append(np.array([sensor_obs_x, sensor_obs_y]))

    # 3. Cálculo da distância ao Objetivo
    dist_to_goal = np.linalg.norm(q_robot - q_goal)
    
    if dist_to_goal > 0.15: # Raio de parada no destino
        f = compute_forces(q_robot, q_goal, current_obstacles)

        # Compensação de 180° no ângulo caso o referencial do robô esteja invertido
        theta_des = math.atan2(f[1], f[0]) + math.pi

        # Normalização do erro no intervalo [-pi, pi] para lidar com o limite [-180°, 180°]
        e_theta = math.atan2(math.sin(theta_des - theta_robot), math.cos(theta_des - theta_robot))
        f_norm = np.linalg.norm(f)

        # Controlador P para velocidade angular e ajuste de velocidade linear
        rotVel = Kp * e_theta
        linVel = max_linVel * max(0.0, math.cos(e_theta)) * math.tanh(f_norm)
    else:
        linVel = 0.0
        rotVel = 0.0

    # 4. Cinemática Inversa e Aplicação nas Rodas
    rightVel = linVel + (L / 2.0) * rotVel
    leftVel  = linVel - (L / 2.0) * rotVel

    sim.setJointTargetVelocity(self.rightMotorHandle, rightVel / wheelradius)
    sim.setJointTargetVelocity(self.leftMotorHandle, leftVel / wheelradius)

def sysCall_actuation():
    pass

def sysCall_cleanup():
    """ Limpeza dos objetos da cena ao encerrar a simulação """
    try:
        for handle in self.spawned_objects:
            sim.removeObject(handle)
    except:
        pass