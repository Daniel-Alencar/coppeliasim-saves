# =============================================================================
#  Robô limpador — varredura em faixas paralelas (boustrophedon)
#
#  Estratégia:
#    1. Anda reto até encontrar o primeiro obstáculo (em princípio, uma parede).
#    2. Gira até ficar paralelo a essa parede e a segue lado a lado: é a
#       primeira faixa. O ângulo da parede vem dos três sensores da frente,
#       então o giro é de 90° só quando o robô chega perpendicular a ela.
#    3. Ao encontrar um obstáculo, contorna-o sempre para o lado da varredura,
#       isto é, para o lado ainda não limpo.
#    4. Durante o contorno mede o afastamento perpendicular à linha da faixa.
#       Quando o afastamento chega a um diâmetro do robô, gira e começa a
#       faixa seguinte: paralela à anterior e no sentido oposto. Numa parede
#       isso produz a curva em "U" do cortador de grama; num obstáculo largo
#       no meio da sala, o mesmo critério vale.
#    5. Repete até o fim da simulação.
#
#  Duas situações que a estratégia pura não resolve e que foram tratadas:
#    - Obstáculo estreito (menos de um diâmetro na direção da varredura): o
#      afastamento nunca chega ao diâmetro e o robô ficaria rodando em volta
#      dele. Quando o contorno o traz de volta à linha da faixa, já depois do
#      obstáculo, ele retoma a mesma faixa no mesmo sentido.
#    - Fim da sala: no último canto o contorno leva o robô de volta, no sentido
#      contrário ao da faixa, sem conseguir se afastar. A varredura então troca
#      de lado e recomeça em direção à região já varrida, indefinidamente.
#    - Armadilhas: num corredor curto a troca de lado acontece nas duas pontas
#      e o robô fica indo e voltando; em volta de um móvel inclinado o contorno
#      pode dar voltas; numa cunha entre móvel e parede ele encalha seguidas
#      vezes. Nesses casos ele gira para o lado mais livre, num ângulo sorteado,
#      e recomeça a estratégia a partir do passo 1.
#
#  Sensores: a cena só tem o sensor frontal. Este script cria, no início da
#  simulação, cópias dele nas laterais e nos cantos da frente, e as remove no
#  fim. Não é preciso alterar a cena.
#
#  Posição: a distância até a linha da faixa exige saber onde o robô está.
#  Por padrão ela vem do simulador; veja POSE_SOURCE para usar os encoders.
# =============================================================================

import math
import random

# ----------------------------- geometria -------------------------------------
WHEEL_RADIUS = 0.05          # raio da roda [m]
TRACK_WIDTH  = 0.2           # distância entre rodas [m]

# O robô da cena precisa de -1 para andar para a frente com linVel positivo
# (mesma convenção dos controladores 1, 2 e 3).
MOTOR_SIGN = -1

# Largura limpa a cada passada. O script /Dirt da cena recolhe a sujeira a até
# 0,15 m do centro do robô, então o "diâmetro" útil é 0,30 m. É também a
# distância entre faixas vizinhas.
ROBOT_DIAMETER = 0.30
LANE_SPACING   = ROBOT_DIAMETER

# ----------------------------- posição ---------------------------------------
# "ground_truth": lê a pose exata do simulador.
# "odometry": integra os encoders das rodas, como faria um robô real. Nesta
#   cena as rodas deslizam muito nos giros no lugar: medido em 120 s, os
#   encoders registraram 44% a mais de rotação do que a real e o rumo errou
#   mais de 100°, o que entorta as faixas. Por isso não é o padrão.
POSE_SOURCE = "ground_truth"

# ----------------------------- velocidades -----------------------------------
LANE_SPEED    = 0.25         # avanço nas faixas [m/s]
CONTOUR_SPEED = 0.12         # avanço contornando obstáculos [m/s]
TURN_RATE     = 1.0          # giro no lugar [rad/s]
MAX_ROT       = 1.5          # limite de velocidade angular [rad/s]
BACK_SPEED    = 0.10         # ré ao desencalhar [m/s]

# ----------------------------- controle --------------------------------------
K_HEADING  = 2.5             # manter rumo: rad/s por rad de erro
K_WALL     = 5.0             # seguir parede: rad/s por metro de erro
K_WALL_D   = 1.5             # termo derivativo do seguir parede
ARC_RADIUS = 0.20            # raio da curva ao dobrar uma quina convexa [m]

# Seguir a parede numa faixa: o erro de distância vira um desvio de rumo em
# relação à faixa, limitado. Sem o limite o robô aponta para a parede ao
# corrigir a distância e os sensores da frente a confundem com um obstáculo.
K_WALL_ANGLE   = 2.0                  # rad de desvio por metro de erro
WALL_MAX_ANGLE = math.radians(15.0)   # desvio máximo em relação à faixa
# O rumo da faixa só é reestimado pela parede com o robô já estável ao lado dela.
WALL_STABLE_ERROR = 0.02     # |distância - SIDE_TARGET| máximo [m]
WALL_STABLE_RATE  = 0.02     # variação máxima da distância [m/s]
WALL_HEADING_GAIN = 0.02     # fração do erro de rumo incorporada por passo

# ----------------------------- limiares --------------------------------------
# Distâncias frontais contam a partir dos sensores da frente (3 cm à frente
# do eixo das rodas). Laterais contam a partir do centro do eixo das rodas.
FRONT_STOP  = 0.20           # obstáculo à frente: parar de avançar [m]
SIDE_TARGET = 0.22           # distância desejada do obstáculo lateral [m]
SIDE_LOST   = 0.40           # acima disso o obstáculo lateral "sumiu" [m]
SIDE_MIN    = 0.17           # abaixo disso, na faixa, afasta-se da lateral [m]
#   Girando no lugar, a traseira do robô descreve um círculo de ~0,19 m em
#   torno do eixo das rodas. SIDE_TARGET > 0,19 garante que ele gire rente à
#   parede sem raspar nela.

TURN_TOLERANCE = math.radians(1.5)   # erro aceito ao terminar um giro
TURN_TIMEOUT   = 6.0                 # teto de um giro [s]

# Retorno à faixa depois de um obstáculo estreito: o robô precisa ter se
# afastado de verdade e avançado além do ponto onde começou a contornar.
RETURN_MIN_OFFSET = 0.08     # [m]
RETURN_MIN_ALONG  = 0.10     # [m]

# Fim da varredura: o contorno trouxe o robô de volta, no sentido contrário
# ao da faixa, por pelo menos esta distância, sem afastamento suficiente.
EXHAUSTED_BACK = LANE_SPACING  # [m]
# Trocas de lado seguidas, sem nenhuma faixa nova, que caracterizam um bolsão.
MAX_FLIPS = 2

CONTOUR_MAX_PATH = 4.0       # contorno longo demais: recomeça a estratégia [m]

# Desencalhe: comandando avanço mas sem sair do lugar.
STUCK_TIME     = 2.5         # janela de observação [s]
STUCK_DISTANCE = 0.03        # deslocamento mínimo esperado na janela [m]
BACK_TIME      = 0.8         # duração da ré [s]
STUCK_WINDOW   = 20.0        # janela para contar encalhes repetidos [s]
STUCK_LIMIT    = 3           # encalhes na janela que fazem recomeçar
ESCAPE_MIN     = math.radians(90.0)    # giro mínimo ao recomeçar
ESCAPE_MAX     = math.radians(180.0)   # giro máximo ao recomeçar

VERBOSE = True               # imprime as trocas de estado

# ----------------------------- sensores criados ------------------------------
# (nome, posição no referencial do robô, direção de detecção, offset)
# O robô aponta para +y e a direita dele é +x. O offset faz o sensor lateral
# começar a detectar só depois da roda, para não enxergar o próprio robô.
_AXLE_Y = 0.07               # o eixo das rodas fica 7 cm à frente da origem
_SPLAY = math.radians(10.0)  # abertura dos sensores de canto para fora
EXTRA_SENSORS = [
    ("sideLeft",   (0.0,   _AXLE_Y, 0.1), (-1.0, 0.0, 0.0), 0.12),
    ("sideRight",  (0.0,   _AXLE_Y, 0.1), (1.0, 0.0, 0.0), 0.12),
    ("frontLeft",  (-0.08, 0.1, 0.1), (-math.sin(_SPLAY), math.cos(_SPLAY), 0.0), 0.0),
    ("frontRight", (0.08,  0.1, 0.1), (math.sin(_SPLAY), math.cos(_SPLAY), 0.0), 0.0),
]
SENSOR_RANGE = 0.8           # alcance dos sensores criados [m]

# --------------------- estado (definido já aqui para que o cleanup -----------
# --------------------- nunca falhe se a inicialização abortar) ---------------
sim = None
robot = -1
rightMotor = -1
leftMotor = -1
frontSensor = -1
sensors = {}                 # nome -> handle dos sensores criados
createdSensors = []

distFront = math.inf         # menor leitura entre os três sensores da frente
frontReadings = {}           # nome do sensor da frente -> leitura
distLeft = math.inf
distRight = math.inf
prevSide = {1: math.inf, -1: math.inf}

# Pose estimada: posição do centro do eixo das rodas e rumo [rad].
poseX = 0.0
poseY = 0.0
poseTheta = 0.0
lastWheelLeft = None
lastWheelRight = None

state = "SEEK"
tState = 0.0
seekHeading = 0.0

# Faixa atual. A varredura avança na direção sweepDir, perpendicular à faixa.
laneHeading = 0.0
sweepDir = 0.0
followWall = False
laneCount = 0
flipsInARow = 0              # trocas de lado desde a última faixa nova

# Contorno atual.
contourOriginX = 0.0
contourOriginY = 0.0
contourMaxOffset = 0.0
contourPath = 0.0

# Giro no lugar.
turnTarget = 0.0
afterTurn = None             # função chamada quando o giro termina

stuckRef = (0.0, 0.0, 0.0)   # (tempo, x, y) do início da janela de encalhe
stuckFrom = "SEEK"           # estado em que o encalhe aconteceu
stuckTimes = []              # instantes dos encalhes recentes


# =============================================================================
#  Utilidades
# =============================================================================

def _wrap(angle):
    """Normaliza um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _log(message):
    if VERBOSE:
        print("[t=%7.2fs] %s" % (sim.getSimulationTime(), message))


def _find_object(name):
    """Busca o objeto pelo nome em qualquer lugar da cena."""
    # noError faz o getObject devolver -1 em vez de lançar erro. Não use
    # try/except aqui: num script Python da cena, um erro lançado pelo
    # CoppeliaSim durante o sysCall_init trava a simulação ("abort execution").
    handle = sim.getObject("/" + name, {'noError': True})
    if handle == -1:
        raise RuntimeError(
            "Objeto '/%s' não encontrado. Confira o nome na Scene Hierarchy." % name
        )
    return handle


def _create_sensor(name, position, direction, offset):
    """Copia o sensor frontal, pendura no robô e aponta para `direction`."""
    handle = sim.copyPasteObjects([frontSensor], 0)[0]
    sim.setObjectParent(handle, robot, True)
    sim.setObjectAlias(handle, name)

    # O sensor detecta ao longo do seu eixo z. Monta a matriz com z = direção
    # desejada e x = vertical; y = z × x completa a base (destra).
    zx, zy, zz = direction
    xx, xy, xz = 0.0, 0.0, 1.0
    yx, yy, yz = zy * xz - zz * xy, zz * xx - zx * xz, zx * xy - zy * xx
    px, py, pz = position
    matrix = [xx, yx, zx, px,
              xy, yy, zy, py,
              xz, yz, zz, pz]
    sim.setObjectMatrix(handle, matrix, robot)

    sim.setFloatProperty(handle, "volume_offset", offset)
    sim.setFloatProperty(handle, "volume_range", SENSOR_RANGE)
    createdSensors.append(handle)
    return handle


def _read_range(handle):
    """Distância lida, ou infinito quando não há detecção."""
    res, dist, *_ = sim.readProximitySensor(handle)
    if res > 0 and dist is not None and dist > 0:
        return dist
    return math.inf


def set_motors(linVel, rotVel):
    rotVel = max(-MAX_ROT, min(MAX_ROT, rotVel))
    v_right = linVel + (TRACK_WIDTH / 2.0) * rotVel
    v_left  = linVel - (TRACK_WIDTH / 2.0) * rotVel
    sim.setJointTargetVelocity(rightMotor, MOTOR_SIGN * v_right / WHEEL_RADIUS)
    sim.setJointTargetVelocity(leftMotor,  MOTOR_SIGN * v_left  / WHEEL_RADIUS)


# =============================================================================
#  Posição
# =============================================================================

def _update_pose():
    global poseX, poseY, poseTheta, lastWheelLeft, lastWheelRight

    if POSE_SOURCE == "ground_truth":
        # Rumo: eixo +y (frente) do robô no mundo. Posição: ponto médio dos
        # motores, que é o centro de giro do robô diferencial.
        m = sim.getObjectMatrix(robot, sim.handle_world)
        poseTheta = math.atan2(m[5], m[1])
        pl = sim.getObjectPosition(leftMotor, sim.handle_world)
        pr = sim.getObjectPosition(rightMotor, sim.handle_world)
        poseX = (pl[0] + pr[0]) / 2.0
        poseY = (pl[1] + pr[1]) / 2.0
        return

    # Odometria: quanto cada roda girou desde o passo anterior. As juntas
    # voltam a -pi depois de pi, por isso a diferença é normalizada.
    qLeft = sim.getJointPosition(leftMotor)
    qRight = sim.getJointPosition(rightMotor)
    if lastWheelLeft is None:
        lastWheelLeft, lastWheelRight = qLeft, qRight
        return
    dLeft = MOTOR_SIGN * _wrap(qLeft - lastWheelLeft) * WHEEL_RADIUS
    dRight = MOTOR_SIGN * _wrap(qRight - lastWheelRight) * WHEEL_RADIUS
    lastWheelLeft, lastWheelRight = qLeft, qRight

    # Cinemática direta: avanço médio e rotação pela diferença entre as rodas.
    # A posição é integrada no rumo do meio do passo, mais preciso em curvas.
    ds = (dLeft + dRight) / 2.0
    dTheta = (dRight - dLeft) / TRACK_WIDTH
    mid = poseTheta + dTheta / 2.0
    poseX += ds * math.cos(mid)
    poseY += ds * math.sin(mid)
    poseTheta = _wrap(poseTheta + dTheta)


# Sensores da frente no referencial do robô: (posição x, y) e direção (x, y).
_FRONT_GEOMETRY = {
    "center":     ((0.0, 0.1), (0.0, 1.0)),
    "frontLeft":  ((-0.08, 0.1), (-math.sin(_SPLAY), math.cos(_SPLAY))),
    "frontRight": ((0.08, 0.1), (math.sin(_SPLAY), math.cos(_SPLAY))),
}
MAX_WALL_SKEW = math.radians(60.0)   # acima disso a estimativa é descartada


def _wall_heading_ahead(side):
    """Rumo paralelo à parede à frente, virando para o lado `side`.

    Cada sensor da frente com leitura dá um ponto da parede. A reta entre os
    dois pontos mais afastados lateralmente dá a inclinação da parede. Sem
    dois pontos, ou com inclinação exagerada, assume parede perpendicular.
    """
    points = []
    for name, ((px, py), (dx, dy)) in _FRONT_GEOMETRY.items():
        d = frontReadings.get(name, math.inf)
        if d != math.inf:
            points.append((px + d * dx, py + d * dy))
    skew = 0.0
    if len(points) >= 2:
        points.sort()
        (x1, y1), (x2, y2) = points[0], points[-1]
        if x2 - x1 > 0.05:
            skew = math.atan2(y2 - y1, x2 - x1)
            if abs(skew) > MAX_WALL_SKEW:
                skew = 0.0
    # A direta do robô aponta para poseTheta - 90°; a parede está girada `skew`.
    return _wrap(poseTheta + side * math.pi / 2.0 + skew)


def _sweep_side():
    """+1 se a varredura avança para a esquerda da faixa, -1 se para a direita."""
    return 1 if math.sin(sweepDir - laneHeading) > 0 else -1


def _side_distance(side):
    """Leitura do sensor lateral: +1 esquerda, -1 direita."""
    return distLeft if side > 0 else distRight


# =============================================================================
#  Comportamentos
# =============================================================================

def _hold_heading(heading, speed):
    """Anda reto no rumo `heading`, afastando-se de laterais muito próximas."""
    rot = K_HEADING * _wrap(heading - poseTheta)
    if distLeft < SIDE_MIN:
        rot -= K_WALL * (SIDE_MIN - distLeft)
    if distRight < SIDE_MIN:
        rot += K_WALL * (SIDE_MIN - distRight)
    set_motors(speed, rot)


def _side_rate(side, dt):
    """Variação da leitura lateral [m/s]; 0 se não havia leitura anterior."""
    dist = _side_distance(side)
    rate = 0.0
    if dist <= SIDE_LOST and prevSide[side] != math.inf and dt > 0:
        rate = (dist - prevSide[side]) / dt
    prevSide[side] = dist if dist <= SIDE_LOST else math.inf
    return rate


def _follow_wall_lane(side, speed, dt):
    """Na faixa: segue a parede do lado `side` sem se afastar muito do rumo.

    Devolve False se a parede não está visível.
    """
    global laneHeading
    dist = _side_distance(side)
    rate = _side_rate(side, dt)
    if dist > SIDE_LOST:
        return False
    # Longe da parede: desvia o rumo para o lado dela; perto: para longe.
    deviation = K_WALL_ANGLE * (dist - SIDE_TARGET)
    deviation = max(-WALL_MAX_ANGLE, min(WALL_MAX_ANGLE, deviation))
    set_motors(speed, K_HEADING * _wrap(laneHeading + side * deviation - poseTheta))
    # Estável ao lado da parede: o rumo do robô é o rumo da parede.
    if abs(dist - SIDE_TARGET) < WALL_STABLE_ERROR and abs(rate) < WALL_STABLE_RATE:
        laneHeading = _wrap(laneHeading + WALL_HEADING_GAIN * _wrap(poseTheta - laneHeading))
    return True


def _follow_side(side, speed, dt):
    """Segue o obstáculo do lado `side` a SIDE_TARGET de distância.

    Devolve False se o obstáculo lateral não está visível.
    """
    dist = _side_distance(side)
    if dist > SIDE_LOST:
        prevSide[side] = math.inf
        return False
    # Longe demais: gira para o lado do obstáculo; perto demais: para longe.
    derivative = 0.0
    if prevSide[side] != math.inf and dt > 0:
        derivative = (dist - prevSide[side]) / dt
    prevSide[side] = dist
    rot = side * (K_WALL * (dist - SIDE_TARGET) + K_WALL_D * derivative)
    set_motors(speed, rot)
    return True


# =============================================================================
#  Transições
# =============================================================================

def _enter(newState):
    global state, tState, stuckRef
    state = newState
    tState = 0.0
    stuckRef = (sim.getSimulationTime(), poseX, poseY)


def _start_turn(target, then, reason):
    global turnTarget, afterTurn
    turnTarget = _wrap(target)
    afterTurn = then
    _log("%s: girando %+.0f°" % (reason, math.degrees(_wrap(turnTarget - poseTheta))))
    _enter("TURN")


def _start_lane(heading, sweep, wall):
    """Começa a faixa com rumo `heading`; a varredura avança para `sweep`."""
    global laneHeading, sweepDir, followWall, laneCount, flipsInARow
    laneHeading = _wrap(heading)
    sweepDir = _wrap(sweep)
    followWall = wall
    laneCount += 1
    flipsInARow = 0
    prevSide[1] = prevSide[-1] = math.inf
    _log("faixa %d: rumo %+.0f°, varredura para a %s%s" % (
        laneCount, math.degrees(laneHeading),
        "esquerda" if _sweep_side() > 0 else "direita",
        ", seguindo a parede" if wall else ""))
    _enter("LANE")


def _start_contour():
    global contourOriginX, contourOriginY, contourMaxOffset, contourPath
    contourOriginX, contourOriginY = poseX, poseY
    contourMaxOffset = 0.0
    contourPath = 0.0
    _log("faixa %d: obstáculo a %.2f m, contornando" % (laneCount, distFront))
    _enter("CONTOUR")


def _restart(reason):
    """Sai de uma armadilha: gira para o lado mais livre e volta ao passo 1."""
    global seekHeading, stuckTimes
    side = 1 if distLeft >= distRight else -1
    seekHeading = _wrap(poseTheta + side * random.uniform(ESCAPE_MIN, ESCAPE_MAX))
    stuckTimes = []
    _log("%s, recomeçando a estratégia" % reason)
    _start_turn(seekHeading, lambda: _enter("SEEK"), "saída")


def _lane_coordinates():
    """(avanço ao longo da faixa, afastamento na direção da varredura)."""
    dx = poseX - contourOriginX
    dy = poseY - contourOriginY
    along = dx * math.cos(laneHeading) + dy * math.sin(laneHeading)
    offset = dx * math.cos(sweepDir) + dy * math.sin(sweepDir)
    return along, offset


def _check_stuck():
    """Comandando avanço sem sair do lugar por STUCK_TIME: desencalha."""
    global stuckRef, stuckFrom, stuckTimes
    now = sim.getSimulationTime()
    t0, x0, y0 = stuckRef
    if now - t0 < STUCK_TIME:
        return False
    moved = math.hypot(poseX - x0, poseY - y0)
    stuckRef = (now, poseX, poseY)
    if moved < STUCK_DISTANCE:
        _log("encalhado (%.3f m em %.1f s), dando ré" % (moved, STUCK_TIME))
        stuckFrom = state
        stuckTimes = [t for t in stuckTimes if now - t <= STUCK_WINDOW] + [now]
        _enter("BACK")
        return True
    return False


# =============================================================================
#  Máquina de estados
# =============================================================================

def _step(dt):
    global contourPath, contourMaxOffset, flipsInARow

    if state == "SEEK":
        # 1. Reto até o primeiro obstáculo.
        if distFront <= FRONT_STOP:
            # Gira para o lado mais livre; a parede fica do outro lado e a
            # varredura se afasta dela.
            side = 1 if distLeft >= distRight else -1
            heading = _wall_heading_ahead(side)
            sweep = heading + side * math.pi / 2.0
            _start_turn(heading, lambda: _start_lane(heading, sweep, True),
                        "primeiro obstáculo")
        elif not _check_stuck():
            _hold_heading(seekHeading, LANE_SPEED)

    elif state == "TURN":
        error = _wrap(turnTarget - poseTheta)
        if abs(error) <= TURN_TOLERANCE or tState >= TURN_TIMEOUT:
            set_motors(0.0, 0.0)
            afterTurn()
        else:
            # Proporcional perto do alvo para não passar do ponto.
            rot = max(-TURN_RATE, min(TURN_RATE, 3.0 * error))
            if abs(rot) < 0.25:
                rot = math.copysign(0.25, error)
            set_motors(0.0, rot)

    elif state == "LANE":
        if distFront <= FRONT_STOP:
            _start_contour()
            return
        if _check_stuck():
            return
        wallSide = -_sweep_side()
        if not (followWall and _follow_wall_lane(wallSide, LANE_SPEED, dt)):
            _hold_heading(laneHeading, LANE_SPEED)

    elif state == "CONTOUR":
        sweepSide = _sweep_side()
        obstacleSide = -sweepSide
        along, offset = _lane_coordinates()
        contourMaxOffset = max(contourMaxOffset, offset)
        contourPath += abs(CONTOUR_SPEED) * dt

        # 4. Afastou-se um diâmetro: próxima faixa, no sentido oposto.
        if offset >= LANE_SPACING:
            heading = laneHeading + math.pi
            sweep = sweepDir
            _start_turn(heading, lambda: _start_lane(heading, sweep, False),
                        "afastamento de %.2f m" % offset)
            return

        # Obstáculo estreito: voltou à linha da faixa depois dele.
        if (contourMaxOffset >= RETURN_MIN_OFFSET and offset <= 0.0
                and along >= RETURN_MIN_ALONG):
            heading = laneHeading
            sweep = sweepDir
            _start_turn(heading, lambda: _start_lane(heading, sweep, False),
                        "obstáculo contornado")
            return

        frontBlocked = distFront <= FRONT_STOP

        # Fim da varredura: o contorno trouxe o robô de volta ao longo da faixa
        # sem que ele conseguisse se afastar. Inverte o lado da varredura.
        if along <= -EXHAUSTED_BACK:
            flips = flipsInARow + 1
            if flips >= MAX_FLIPS:
                _restart("preso num corredor")
                return
            _log("fim da varredura nesse sentido, invertendo")
            _start_lane(laneHeading + math.pi, sweepDir + math.pi, True)
            flipsInARow = flips
            return

        if contourPath >= CONTOUR_MAX_PATH:
            _restart("contorno longo demais")
            return

        # 3. Contorno pelo lado da varredura.
        if frontBlocked:
            # Parede à frente: gira no lugar para o lado da varredura.
            set_motors(0.0, sweepSide * TURN_RATE)
            return
        if _check_stuck():
            return
        if not _follow_side(obstacleSide, CONTOUR_SPEED, dt):
            # Perdeu o obstáculo (quina convexa): curva em direção a ele.
            set_motors(CONTOUR_SPEED, obstacleSide * CONTOUR_SPEED / ARC_RADIUS)

    elif state == "BACK":
        if tState < BACK_TIME:
            set_motors(-BACK_SPEED, 0.0)
        else:
            # Buscando o primeiro obstáculo: encalhar é tê-lo encontrado.
            # Numa faixa ou contornando: trata o encalhe como obstáculo à frente.
            if len(stuckTimes) >= STUCK_LIMIT:
                _restart("encalhes repetidos")
            elif stuckFrom == "SEEK":
                _enter("SEEK")
            else:
                _start_contour()

    else:
        set_motors(0.0, 0.0)


# =============================================================================
#  Callbacks do CoppeliaSim
# =============================================================================

def sysCall_init():
    global sim, robot, rightMotor, leftMotor, frontSensor, sensors
    global state, seekHeading, stuckRef

    sim = require('sim')

    rightMotor  = _find_object("rightMotor")
    leftMotor   = _find_object("leftMotor")
    frontSensor = _find_object("proximitySensor")
    robot = sim.getObjectParent(leftMotor)

    sensors = {}
    for name, position, direction, offset in EXTRA_SENSORS:
        sensors[name] = _create_sensor(name, position, direction, offset)

    _update_pose()
    seekHeading = poseTheta
    state = "SEEK"
    stuckRef = (sim.getSimulationTime(), poseX, poseY)
    _log("controlador de faixas pronto (posição: %s, faixas a cada %.2f m)"
         % (POSE_SOURCE, LANE_SPACING))


def sysCall_sensing():
    global distFront, distLeft, distRight
    _update_pose()
    frontReadings["center"] = _read_range(frontSensor)
    frontReadings["frontLeft"] = _read_range(sensors["frontLeft"])
    frontReadings["frontRight"] = _read_range(sensors["frontRight"])
    distFront = min(frontReadings.values())
    distLeft  = _read_range(sensors["sideLeft"])
    distRight = _read_range(sensors["sideRight"])


def sysCall_actuation():
    global tState
    dt = sim.getSimulationTimeStep()
    _step(dt)
    tState += dt


def sysCall_cleanup():
    if rightMotor != -1:
        sim.setJointTargetVelocity(rightMotor, 0.0)
    if leftMotor != -1:
        sim.setJointTargetVelocity(leftMotor, 0.0)
    if createdSensors:
        sim.removeObjects(createdSensors)
