import math
import random

deg2rad = math.pi / 180.0

def sysCall_init():
    global sim
    sim = require('sim')

    # Variáveis globais de estado e handles
    global rightMotor, leftMotor, sensor
    global wheelRadius, trackWidth, cruiseSpeed, turnSpeed
    global state, stateTimer, turnDuration, turnDirection

    try:
        rightMotor = sim.getObject("/rightMotor")
        leftMotor = sim.getObject("/leftMotor")
        sensor = sim.getObject("/proximitySensor")
    except Exception as e:
        print(f"Erro ao carregar objetos: {e}. Verifique os nomes na Scene Hierarchy!")

    wheelRadius = 0.05
    trackWidth = 0.2
    cruiseSpeed = 0.4
    turnSpeed = 1.2

    state = "FORWARD"
    stateTimer = 0.0
    turnDuration = 0.0
    turnDirection = 1

def set_motors(linVel, rotVel):
    v_right = linVel + (trackWidth / 2.0) * rotVel
    v_left  = linVel - (trackWidth / 2.0) * rotVel

    omega_right = v_right / wheelRadius
    omega_left  = v_left  / wheelRadius

    # Se o robô andar para trás, inverta o sinal aqui (tire ou coloque o '-')
    sim.setJointTargetVelocity(rightMotor, -omega_right)
    sim.setJointTargetVelocity(leftMotor,  -omega_left)

def sysCall_actuation():
    global state, stateTimer, turnDuration, turnDirection
    
    dt = sim.getSimulationTimeStep()
    stateTimer += dt

    res, dist, *_ = sim.readProximitySensor(sensor)
    obstacle_detected = (res > 0 and dist <= 0.45)

    if state == "FORWARD":
        if obstacle_detected:
            state = "BACKWARD"
            stateTimer = 0.0
        else:
            set_motors(cruiseSpeed, 0.0)

    elif state == "BACKWARD":
        if stateTimer < 0.3:
            set_motors(-cruiseSpeed * 0.5, 0.0)
        else:
            state = "TURN"
            stateTimer = 0.0
            turnDuration = random.uniform(0.8, 1.8)
            turnDirection = random.choice([1, -1])

    elif state == "TURN":
        if stateTimer < turnDuration:
            set_motors(0.0, turnSpeed * turnDirection)
        else:
            state = "FORWARD"
            stateTimer = 0.0

def sysCall_cleanup():
    sim.setJointTargetVelocity(rightMotor, 0.0)
    sim.setJointTargetVelocity(leftMotor, 0.0)
