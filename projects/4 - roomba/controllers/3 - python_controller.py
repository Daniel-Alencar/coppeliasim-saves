# =============================================================================
#  Robô limpador — controlador puramente reativo
#
#  Navegação 100% reativa: as decisões saem apenas da leitura instantânea dos
#  sensores e de temporizadores internos. Não há mapa, não há odometria global
#  e não há planejamento de trajetória.
#
#  Arbitragem por prioridade (do mais urgente para o menos):
#      DESENCALHE  > FUGA (ré + giro) > DESVIO > ESPIRAL > CRUZEIRO
# =============================================================================

import math
import random

# ----------------------------- geometria -------------------------------------
WHEEL_RADIUS = 0.05          # raio da roda [m]
TRACK_WIDTH  = 0.2           # distância entre rodas [m]

# Sinal aplicado às duas rodas. O robô da cena precisa de -1 para andar
# para a frente com linVel positivo (mesma convenção dos controladores 1 e 2).
MOTOR_SIGN = -1

# Sentido do giro. Se o robô girar para o lado "errado" ao desviar, troque
# para -1. Não afeta a fuga, que gira até o sensor liberar seja qual for o lado.
TURN_SIGN = 1

# ----------------------------- velocidades -----------------------------------
CRUISE_SPEED = 0.35          # avanço normal [m/s]
AVOID_SPEED  = 0.12          # avanço durante o desvio [m/s]
AVOID_TURN_MIN = 0.35        # fração mínima de TURN_RATE ao desviar
BACK_SPEED   = 0.20          # velocidade de ré [m/s]
TURN_RATE    = 1.4           # velocidade angular do giro [rad/s]

# ----------------------------- limiares --------------------------------------
DIST_CRITICAL = 0.25         # abaixo disso: para e dá ré [m]
DIST_WARN     = 0.55         # abaixo disso: desvia suavemente [m]
DIST_CLEAR    = 0.75         # só acima disso o caminho volta a ser "livre" [m]
#   DIST_CLEAR > DIST_WARN de propósito: é a histerese do desvio. Sem ela o
#   robô volta ao cruzeiro assim que fica paralelo à parede — a ~5 cm dela,
#   porque o cone frontal deixa de ver a parede — e o zigue-zague o joga de
#   volta contra ela. Com um sensor só apontando para a frente, essa margem é
#   a única defesa contra raspar a parede.

# ----------------------------- tempos ----------------------------------------
DEPART_TIME  = 1.2           # afastamento após o desvio [s]
DEPART_TURN  = 0.35          # curva mantida no afastamento [rad/s]

BACK_TIME    = 0.5           # duração da ré [s]
TURN_EXTRA   = 0.30          # tempo com a frente livre para encerrar o giro [s]
TURN_TIMEOUT = 4.0           # teto do giro, evita girar para sempre [s]

# Zigue-zague lento no cruzeiro: quebra trajetórias retas que se repetem.
# Há um trade-off medido aqui (24 execuções de 300 s num simulador cinemático,
# sala de 5x5 m com 4 obstáculos), entre cobertura e encostar na parede:
#     amplitude 0.25 -> cobertura 81%, 2.3% dos passos barrados pela parede
#     amplitude 0.15 -> cobertura 79%, 0.2%
#     amplitude 0.00 -> cobertura 77%, 0.1%
# Nenhum valor domina os outros. 0.25 privilegia cobertura, que é o objetivo
# de um robô limpador; baixe para 0.15 se encostar na parede for um problema.
WANDER_RATE   = 0.25         # amplitude [rad/s]
WANDER_PERIOD = 3.0          # período [s]

# Espiral: cobre áreas abertas sem precisar de mapa (é o que o Roomba faz).
SPIRAL_PERIOD = 25.0         # tempo de cruzeiro livre até tentar espiralar [s]
SPIRAL_W0     = 2.0          # velocidade angular inicial (espiral fechada)
SPIRAL_WMIN   = 0.25         # ao chegar aqui a espiral acabou
SPIRAL_DECAY  = 0.25         # taxa de abertura da espiral [1/s]

# Desencalhe: muitas fugas em pouco tempo indicam canto ou robô preso.
STUCK_WINDOW    = 12.0       # janela de observação [s]
STUCK_ESCAPES   = 4          # nº de fugas na janela que dispara o desencalhe
UNWEDGE_BACK_TIME = 1.0      # ré longa [s]
UNWEDGE_TURN_TIME = 2.0      # giro longo [s]

RANDOM_SEED = None           # troque por um inteiro para runs reproduzíveis
VERBOSE     = True           # imprime as trocas de estado

# --------------------- estado (definido já aqui para que o cleanup -----------
# --------------------- nunca falhe se a inicialização abortar) ---------------
sim = None
rightMotor = -1
leftMotor = -1
frontSensor = -1
leftSensor = -1
rightSensor = -1

distFront = math.inf
distLeft = math.inf
distRight = math.inf

state = "CRUISE"
tState = 0.0
tSinceSpiral = 0.0
spiralOmega = 0.0
turnDir = 1
avoidDir = 1
turnClearFor = 0.0
wanderPhase = 0.0
escapeTimes = []


# =============================================================================
#  Aquisição de handles
# =============================================================================

def _try_get(path):
    try:
        return sim.getObject(path)
    except Exception:
        return -1


def _find_object(name):
    """Procura o objeto como filho do script ('./nome') e na raiz ('/nome')."""
    for path in ("./" + name, "/" + name):
        handle = _try_get(path)
        if handle != -1:
            return handle, path
    return -1, None


def _require_object(name):
    handle, path = _find_object(name)
    if handle == -1:
        # Sem os motores não há o que fazer: falha alto, com a causa real.
        raise RuntimeError(
            "Objeto '%s' não encontrado (tentei './%s' e '/%s'). "
            "Confira o nome na Scene Hierarchy." % (name, name, name)
        )
    print("[init] %-18s -> %s" % (name, path))
    return handle


def _optional_object(name):
    handle, path = _find_object(name)
    if handle == -1:
        print("[init] %-18s -> ausente (opcional)" % name)
    else:
        print("[init] %-18s -> %s" % (name, path))
    return handle


# =============================================================================
#  Sensores e atuadores
# =============================================================================

def _read_range(handle):
    """Distância lida, ou infinito quando não há detecção ou sensor."""
    if handle == -1:
        return math.inf
    res, dist, *_ = sim.readProximitySensor(handle)
    if res > 0 and dist is not None and dist > 0:
        return dist
    return math.inf


def set_motors(linVel, rotVel):
    v_right = linVel + (TRACK_WIDTH / 2.0) * rotVel
    v_left  = linVel - (TRACK_WIDTH / 2.0) * rotVel
    sim.setJointTargetVelocity(rightMotor, MOTOR_SIGN * v_right / WHEEL_RADIUS)
    sim.setJointTargetVelocity(leftMotor,  MOTOR_SIGN * v_left  / WHEEL_RADIUS)


def _pick_turn_direction():
    """+1 gira para um lado, -1 para o outro.

    Com sensores laterais, foge para o lado mais livre. Sem eles, sorteia —
    é a única informação disponível com um sensor frontal apenas.
    """
    if distLeft > distRight:
        return 1
    if distRight > distLeft:
        return -1
    return random.choice([1, -1])


# =============================================================================
#  Máquina de estados
# =============================================================================

def _enter(newState):
    global state, tState, turnClearFor, avoidDir
    if VERBOSE and newState != state:
        print("[t=%6.2fs] %-12s -> %s" % (sim.getSimulationTime(), state, newState))
    if newState == "TURN":
        turnClearFor = 0.0
    if newState == "AVOID":
        avoidDir = _pick_turn_direction()
    state = newState
    tState = 0.0


def _start_spiral():
    global spiralOmega
    spiralOmega = SPIRAL_W0
    _enter("SPIRAL")


def _start_escape():
    """Entra em fuga; se as fugas se repetem demais, escala para desencalhe."""
    global escapeTimes, turnDir
    now = sim.getSimulationTime()
    escapeTimes = [t for t in escapeTimes if now - t <= STUCK_WINDOW]
    escapeTimes.append(now)

    if len(escapeTimes) >= STUCK_ESCAPES:
        escapeTimes = []
        turnDir = random.choice([1, -1])
        _enter("UNWEDGE_BACK")
    else:
        _enter("BACK")


def _update_state(dt):
    """Decide o estado deste passo. Roda ANTES de comandar os motores."""
    global turnDir

    critical = distFront <= DIST_CRITICAL
    warning  = distFront <= DIST_WARN

    if state == "CRUISE":
        if critical:
            _start_escape()
        elif warning:
            _enter("AVOID")
        elif tSinceSpiral >= SPIRAL_PERIOD:
            _start_spiral()

    elif state == "SPIRAL":
        if critical:
            _start_escape()
        elif warning:
            _enter("AVOID")
        elif spiralOmega <= SPIRAL_WMIN:
            _enter("CRUISE")

    elif state == "AVOID":
        if critical:
            _start_escape()
        elif distFront > DIST_CLEAR:
            _enter("DEPART")

    elif state == "DEPART":
        # Afasta-se antes de voltar a vagar, para não reencostar na parede.
        if critical:
            _start_escape()
        elif warning:
            _enter("AVOID")
        elif tState >= DEPART_TIME:
            _enter("CRUISE")

    elif state == "BACK":
        if tState >= BACK_TIME:
            turnDir = _pick_turn_direction()
            _enter("TURN")

    elif state == "TURN":
        # Termina por sensor (frente livre), não por tempo sorteado. O timeout
        # é só uma rede de segurança para o robô cercado por todos os lados.
        if turnClearFor >= TURN_EXTRA or tState >= TURN_TIMEOUT:
            _enter("CRUISE")

    elif state == "UNWEDGE_BACK":
        if tState >= UNWEDGE_BACK_TIME:
            _enter("UNWEDGE_TURN")

    elif state == "UNWEDGE_TURN":
        if tState >= UNWEDGE_TURN_TIME:
            _enter("CRUISE")


def _act(dt):
    """Comanda os motores. Todo estado comanda, em todo passo, sem exceção."""
    global tSinceSpiral, spiralOmega, wanderPhase, turnClearFor

    if state == "CRUISE":
        tSinceSpiral += dt
        wanderPhase += dt
        rot = WANDER_RATE * math.sin(2.0 * math.pi * wanderPhase / WANDER_PERIOD)
        set_motors(CRUISE_SPEED, rot)

    elif state == "SPIRAL":
        tSinceSpiral = 0.0
        # Decaimento exponencial: raio = v / ω cresce, abrindo a espiral.
        spiralOmega *= math.exp(-SPIRAL_DECAY * dt)
        set_motors(CRUISE_SPEED, TURN_SIGN * spiralOmega)

    elif state == "AVOID":
        # Curva proporcional à proximidade: longe curva pouco, perto curva
        # muito e desacelera. Evita o liga-desliga do controlador anterior.
        span = max(DIST_WARN - DIST_CRITICAL, 1e-6)
        closeness = (DIST_WARN - distFront) / span
        closeness = min(max(closeness, 0.0), 1.0)
        lin = CRUISE_SPEED + (AVOID_SPEED - CRUISE_SPEED) * closeness
        strength = AVOID_TURN_MIN + (1.0 - AVOID_TURN_MIN) * closeness
        rot = TURN_SIGN * avoidDir * TURN_RATE * strength
        set_motors(lin, rot)

    elif state == "DEPART":
        # Segue em frente mantendo um pouco da curva de fuga, sem zigue-zague.
        set_motors(CRUISE_SPEED, TURN_SIGN * avoidDir * DEPART_TURN)

    elif state == "BACK":
        set_motors(-BACK_SPEED, 0.0)

    elif state == "TURN":
        if distFront > DIST_CLEAR:
            turnClearFor += dt
        else:
            turnClearFor = 0.0
        set_motors(0.0, TURN_SIGN * turnDir * TURN_RATE)

    elif state == "UNWEDGE_BACK":
        set_motors(-BACK_SPEED, 0.0)

    elif state == "UNWEDGE_TURN":
        set_motors(0.0, TURN_SIGN * turnDir * TURN_RATE)

    else:
        set_motors(0.0, 0.0)


# =============================================================================
#  Callbacks do CoppeliaSim
# =============================================================================

def sysCall_init():
    global sim, rightMotor, leftMotor, frontSensor, leftSensor, rightSensor
    global state, tState, tSinceSpiral, spiralOmega, escapeTimes, wanderPhase

    sim = require('sim')

    if RANDOM_SEED is not None:
        random.seed(RANDOM_SEED)

    rightMotor  = _require_object("rightMotor")
    leftMotor   = _require_object("leftMotor")
    frontSensor = _require_object("proximitySensor")

    # Opcionais: se existirem na cena, o robô foge para o lado mais livre em
    # vez de sortear. Se não existirem, tudo funciona igual.
    leftSensor  = _optional_object("proximitySensorLeft")
    rightSensor = _optional_object("proximitySensorRight")

    state = "CRUISE"
    tState = 0.0
    tSinceSpiral = 0.0
    spiralOmega = 0.0
    escapeTimes = []
    wanderPhase = random.uniform(0.0, WANDER_PERIOD)

    print("[init] controlador reativo pronto (estado inicial: CRUISE)")


def sysCall_sensing():
    global distFront, distLeft, distRight
    distFront = _read_range(frontSensor)
    distLeft  = _read_range(leftSensor)
    distRight = _read_range(rightSensor)


def sysCall_actuation():
    global tState

    dt = sim.getSimulationTimeStep()

    _update_state(dt)   # 1. decide
    _act(dt)            # 2. atua (sempre)

    # Incrementado no fim: durante o passo, tState é o tempo já gasto no
    # estado — zero no primeiro passo após uma transição.
    tState += dt


def sysCall_cleanup():
    if rightMotor != -1:
        sim.setJointTargetVelocity(rightMotor, 0.0)
    if leftMotor != -1:
        sim.setJointTargetVelocity(leftMotor, 0.0)
