"""Cinemática do KUKA youBot: base omnidirecional e braço de 5 juntas.

Este módulo é puro Python — não conhece ROS nem CoppeliaSim. Recebe números e
devolve números, o que permite testá-lo sem simulador (ver test/).

São duas cinemáticas independentes, e é a primeira vez no repositório que
aparece a segunda:

    base   4 rodas suecas  ->  cinemática INVERSA de um robô HOLONÔMICO
    braço  5 juntas        ->  cinemática DIRETA e INVERSA de um manipulador

A diferença entre a base daqui e o robô diferencial dos exercícios anteriores
é o que o nome holonômico quer dizer: lá, das três coordenadas do plano
(x, y, θ), o robô só comandava duas (v e ω) e não tinha como andar de lado —
restrição não-holonômica, e é por isso que o controlador ρ-α-β precisava girar
antes de andar. Aqui as quatro rodas suecas dão as três: (vx, vy, ω) são
independentes, o robô anda de lado sem girar, e o controle de posição fica bem
mais simples (ver base_control.py).

De onde vêm os números
----------------------
Toda constante geométrica daqui foi MEDIDA na cena, não copiada de catálogo:
`scene/measure_arm.py` zera as juntas, lê as posições pela Remote API e
imprime estes mesmos valores; o mesmo script confere a cinemática direta
contra a posição real da garra (pior erro medido: 1,1 mm, que é o offset
lateral de 1 mm do eixo da ferramenta mais a discretização da leitura).

Referencial
-----------
Tudo no frame do dummy `/youBot/youBot_ref`, que fica no centro da base, no
plano do chão (z = 0 é o chão) e com o eixo **+y para a frente do robô** — a
mesma convenção estranha do robô dos exercícios anteriores. Quem converte para
a convenção do ROS (+x para a frente) é a ponte, como ela já fazia antes.
"""

import math

# =============================================================================
#  Base omnidirecional
# =============================================================================

#: Raio das rodas suecas [m] (centro da roda a 0,050 m do chão).
WHEEL_RADIUS = 0.050
#: Meia-bitola: distância do centro ao plano de uma roda, no eixo x [m].
HALF_TRACK = 0.158
#: Meia-base: distância do centro ao eixo de uma roda, no eixo y [m].
HALF_BASE = 0.228

#: Nome das juntas de rolamento, na ordem (frente-esq, frente-dir, trás-esq,
#: trás-dir). A ordem é a deste módulo; a ponte cuida de achá-las na cena.
WHEELS = ('fl', 'fr', 'rl', 'rr')

#: Torque máximo do motor de cada roda [N.m], lido da cena.
MAX_WHEEL_FORCE = 2.5
#: Velocidade de roda a partir da qual o motor satura [rad/s].
#:
#: Medido em `scene/calibrate_base.py`: pedindo 8,0 rad/s a roda faz 5,4, porque
#: o torque bate no limite de MAX_WHEEL_FORCE. Pedir mais do que isto não anda
#: mais rápido — só desacopla o comando do movimento, e o controlador de
#: posição passa a mirar uma velocidade que a base não tem como dar.
MAX_WHEEL_SPEED = 5.0

#: Ganho de escorregamento da base, MEDIDO (ver scene/calibrate_base.py).
#:
#: A roda sueca escorrega por construção: dos rolos a 45°, só parte do giro vira
#: deslocamento. Medido nesta base, o corpo faz cerca de 0,67 do que a
#: cinemática pede, nas três liberdades. Não é erro de modelo — é física da
#: roda — e quem quiser que "0,1 m/s" signifique 0,1 m/s multiplica o comando
#: por 1/0,67 (é o `cmd_gain` de base_control).
SLIP_GAIN = 0.67


def wheel_velocities(forward, left, turn,
                     radius=WHEEL_RADIUS,
                     half_track=HALF_TRACK, half_base=HALF_BASE):
    """Velocidade angular de cada roda [rad/s] para um twist do corpo.

    forward   velocidade para a frente do robô (+y dele) [m/s]
    left      velocidade para a esquerda do robô (-x dele) [m/s]
    turn      velocidade de giro, anti-horária vista de cima [rad/s]

    Devolve (fl, fr, rl, rr), na ordem de WHEELS.

    A roda sueca (ou mecanum) tem rolos a 45° no aro: ela transmite força só
    numa diagonal e escorrega livremente na outra. Com as quatro montadas em
    diagonais alternadas, a soma das quatro contribuições dá as três
    liberdades do plano. O padrão de sinais é o do próprio script do modelo
    (`/youBot/Script`, função setMovement), que é a referência do fabricante:

        fl = -(forward + left + turn)      rl = -(forward - left + turn)
        fr = -(forward - left - turn)      rr = -(forward + left - turn)

    O sinal global negativo é como as juntas estão montadas na cena: velocidade
    NEGATIVA faz o robô andar para a frente (o mesmo detalhe do robô
    diferencial dos exercícios anteriores).

    O braço de alavanca do giro é (half_track + half_base), e não a distância
    do centro à roda: para a roda sueca o que conta é a projeção do braço na
    direção em que ela transmite força, que está a 45°. A soma dos dois
    semi-lados é o resultado dessa projeção, e `scene/calibrate_base.py`
    confere o número medindo a rotação real do corpo no simulador.
    """
    lever = (half_track + half_base) * turn
    return (
        -(forward - left - lever) / radius,   # fl
        -(forward + left + lever) / radius,   # fr
        -(forward + left - lever) / radius,   # rl
        -(forward - left + lever) / radius,   # rr
    )


def scale_to_limit(wheels, max_speed):
    """Reduz as quatro rodas no mesmo fator até caberem no limite [rad/s].

    Escalar as quatro juntas mantém a direção do movimento e a curvatura: o
    robô faz o mesmo caminho, só mais devagar. Saturar uma roda só mudaria a
    direção do deslocamento, que é o erro clássico nesse tipo de base.
    """
    fastest = max(abs(w) for w in wheels)
    if fastest <= max_speed or fastest < 1e-9:
        return wheels
    factor = max_speed / fastest
    return tuple(w * factor for w in wheels)


# =============================================================================
#  Braço: geometria medida
# =============================================================================

class ArmGeometry:
    """Medidas do braço do youBot, no frame youBot_ref com as juntas em zero.

    A cadeia é a de um manipulador antropomórfico de 5 eixos:

        junta 0   eixo VERTICAL     gira o braço inteiro em torno de z
        junta 1   eixo +x  ─┐
        junta 2   eixo +x   ├─ as três giram no MESMO plano: cadeia planar 3R
        junta 3   eixo +x  ─┘
        junta 4   eixo da ferramenta  gira a garra (não move a ponta)

    É essa estrutura que torna a inversa analítica: a junta 0 resolve o azimute
    do alvo, as juntas 1-2-3 resolvem um problema PLANO de três elos, e a junta
    4 só orienta os dedos. Sem ela (com os eixos fora de um plano comum) seria
    preciso um método numérico.

    Como as juntas 1, 2 e 3 giram em torno de x, a coordenada x de cada junta
    não muda com elas: os pequenos desvios laterais do braço real (4 cm na
    junta 1, 3 cm na 2) são constantes e se cancelam ao longo da cadeia — a
    ponta da garra fica a 1 mm do plano da junta 0. Esse 1 mm é `tool_offset`,
    e é o único resíduo do modelo plano.
    """

    #: Posição do eixo da junta 0 no frame youBot_ref [m]. O braço não fica no
    #: centro da base: está 0,166 m à frente dele.
    axis_y = 0.1662
    #: Altura do ombro (junta 1) e seu afastamento radial do eixo da junta 0 [m].
    shoulder_r = 0.0330
    shoulder_z = 0.2459
    #: Elos do plano: ombro->cotovelo, cotovelo->pulso, pulso->ponta da garra.
    #: O último soma junta3->junta4 (0,0964) com junta4->pastilhas (0,0849),
    #: que são colineares porque a junta 4 gira em torno do próprio elo.
    upper_arm = 0.1550
    forearm = 0.1348
    wrist_to_tip = 0.0964 + 0.0849
    #: Desvio da ponta em relação ao plano do braço [m] (ver acima).
    tool_offset = -0.0007

    #: Limites das juntas, lidos da cena com sim.getJointInterval [rad].
    limits = (
        (-2.950, 2.950),    # junta 0
        (-1.571, 1.309),    # junta 1  (assimétrico!)
        (-2.286, 2.286),    # junta 2
        (-1.780, 1.780),    # junta 3
        (-1.571, 1.571),    # junta 4
    )

    #: Garra: as duas juntas prismáticas. Fechada em (0, 0), as pastilhas ficam
    #: a 40,3 mm de centro a centro; abertas em (0,025, -0,050), a 90,3 mm.
    #: Descontada a espessura de 14 mm das pastilhas, o vão útil vai de 26 mm
    #: (fechada) a 76 mm (aberta): é essa a faixa de largura que ela segura.
    gripper_closed = (0.000, 0.000)
    gripper_open = (0.025, -0.050)
    gripper_gap_closed = 0.0263
    gripper_gap_open = 0.0763

    @property
    def reach(self):
        """Alcance máximo do braço, do eixo da junta 0 à ponta [m]."""
        return (self.shoulder_r + self.upper_arm + self.forearm
                + self.wrist_to_tip)


ARM = ArmGeometry()


def wrap(angle):
    """Normaliza um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def within_limits(q, geom=ARM, tolerance=1e-6):
    """True se cada junta está dentro do intervalo lido da cena."""
    return all(low - tolerance <= value <= high + tolerance
               for value, (low, high) in zip(q, geom.limits))


# =============================================================================
#  Braço: cinemática direta
# =============================================================================

def arm_fk(q, geom=ARM):
    """Posição da ponta da garra (x, y, z) no frame youBot_ref, em metros.

    Convenção dos ângulos do plano: `a` é o ângulo ABSOLUTO de um elo medido
    desde a vertical +z, positivo na direção radial em que o braço aponta.
    Como um giro positivo em torno de +x inclina o elo para -y, cada `a` é o
    acumulado das juntas com sinal trocado:

        a1 = -q1        a2 = -(q1 + q2)        a3 = -(q1 + q2 + q3)

    No plano (r, z), com r medido desde o eixo da junta 0:

        r = r_ombro + L1 sen a1 + L2 sen a2 + L3 sen a3
        z = z_ombro + L1 cos a1 + L2 cos a2 + L3 cos a3

    E a junta 0 leva esse par para o espaço, girando o plano em torno de z. Um
    giro de q0 em torno de +z leva o versor +y para (-sen q0, cos q0): é essa a
    direção radial do braço.
    """
    a1 = -q[1]
    a2 = -(q[1] + q[2])
    a3 = -(q[1] + q[2] + q[3])

    r = (geom.shoulder_r
         + geom.upper_arm * math.sin(a1)
         + geom.forearm * math.sin(a2)
         + geom.wrist_to_tip * math.sin(a3))
    z = (geom.shoulder_z
         + geom.upper_arm * math.cos(a1)
         + geom.forearm * math.cos(a2)
         + geom.wrist_to_tip * math.cos(a3))

    radial = (-math.sin(q[0]), math.cos(q[0]))
    # O offset da ferramenta é perpendicular ao plano do braço, que é a direção
    # radial girada de 90°.
    normal = (radial[1], -radial[0])
    return (r * radial[0] + geom.tool_offset * normal[0],
            geom.axis_y + r * radial[1] + geom.tool_offset * normal[1],
            z)


def arm_plane(point, geom=ARM):
    """(azimute q0, raio r, altura z) de um ponto do frame youBot_ref.

    É a primeira metade da inversa: decide para onde a junta 0 tem de girar, e
    reduz o alvo 3D a um par (r, z) no plano do braço.
    """
    dx = point[0]
    dy = point[1] - geom.axis_y
    radius = math.hypot(dx, dy)
    if radius < 1e-9:
        return 0.0, 0.0, point[2]
    # Inversa de radial = (-sen q0, cos q0).
    q0 = math.atan2(-dx, dy)
    # O offset lateral da ferramenta desloca a ponta para fora do plano; o
    # braço compensa girando um pouco mais a junta 0. O ângulo é pequeno
    # (1 mm sobre ~250 mm), mas sai de graça.
    if abs(geom.tool_offset) > 1e-9 and radius > abs(geom.tool_offset):
        q0 += math.asin(geom.tool_offset / radius)
        radius = math.sqrt(radius * radius - geom.tool_offset ** 2)
    return wrap(q0), radius, point[2]


# =============================================================================
#  Braço: cinemática inversa
# =============================================================================

def arm_ik(target, approach=math.pi, roll=0.0, geom=ARM, elbow='up'):
    """Juntas (q0..q4) que põem a ponta da garra em `target`.

    target     (x, y, z) no frame youBot_ref [m]
    approach   ângulo absoluto do último elo, desde +z e positivo na direção
               radial: math.pi é a garra apontando para BAIXO (preensão por
               cima, que é a desta tarefa), math.pi/2 é apontando para fora
               (preensão lateral)
    roll       giro da junta 4, que orienta a abertura dos dedos [rad]
    elbow      'up' ou 'down': os dois jeitos de o cotovelo dobrar

    Devolve a tupla (q0..q4), ou levanta Unreachable explicando o motivo — que
    é informação útil para o nó de missão decidir reposicionar a base em vez
    de insistir.

    Como o último elo tem direção FIXADA por `approach`, a posição do pulso
    sai direto do alvo, e sobra um problema de dois elos (2R) — o caso de
    livro, com solução fechada:

        pulso = alvo - L3 * direção(approach)
        cos(cotovelo) = (D² - L1² - L2²) / (2 L1 L2)        lei dos cossenos

    É por isso que fixar `approach` é o que torna a inversa analítica: um braço
    de 5 juntas tem uma liberdade de sobra para pôr a ponta num ponto, e
    `approach` é a escolha que consome essa liberdade.
    """
    q0, radius, height = arm_plane(target, geom)

    # --- 1. Do alvo para o pulso ------------------------------------------
    wrist_r = radius - geom.wrist_to_tip * math.sin(approach)
    wrist_z = height - geom.wrist_to_tip * math.cos(approach)

    # --- 2. Cadeia de dois elos até o pulso --------------------------------
    dr = wrist_r - geom.shoulder_r
    dz = wrist_z - geom.shoulder_z
    distance = math.hypot(dr, dz)
    upper, fore = geom.upper_arm, geom.forearm

    if distance > upper + fore:
        raise Unreachable(
            f'alvo a {distance:.3f} m do ombro, e o braço estica {upper + fore:.3f} m '
            f'(faltam {distance - upper - fore:.3f} m: aproxime a base)')
    if distance < abs(upper - fore):
        raise Unreachable(
            f'alvo a {distance:.3f} m do ombro, dentro do raio morto de '
            f'{abs(upper - fore):.3f} m (afaste a base)')

    cos_elbow = (distance * distance - upper * upper - fore * fore) / (2.0 * upper * fore)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))    # tira o erro numérico
    bend = math.acos(cos_elbow)
    if elbow == 'up':
        bend = -bend
    elif elbow != 'down':
        raise ValueError("elbow deve ser 'up' ou 'down'")

    # Ângulos absolutos dos elos, medidos de +z e positivos para +r. O
    # atan2(dr, dz) é a direção do ombro ao pulso nessa mesma convenção.
    a1 = math.atan2(dr, dz) - math.atan2(fore * math.sin(bend),
                                         upper + fore * math.cos(bend))
    a2 = a1 + bend

    # --- 3. De volta para as juntas ----------------------------------------
    q = (q0, -a1, -(a2 - a1), -(approach - a2), roll)

    if not within_limits(q, geom):
        details = ', '.join(
            f'q{i}={value:+.2f} fora de [{low:+.2f}, {high:+.2f}]'
            for i, (value, (low, high)) in enumerate(zip(q, geom.limits))
            if not low - 1e-6 <= value <= high + 1e-6)
        raise Unreachable(f'solução fora dos limites das juntas: {details}')
    return q


def arm_ik_best(target, approach=math.pi, roll=0.0, geom=ARM):
    """A inversa tentando os dois cotovelos, e devolvendo o que couber.

    Prefere 'up' (cotovelo para cima), que é a postura que mantém o braço longe
    do chão e do próprio corpo do robô na preensão por cima.
    """
    errors = []
    for elbow in ('up', 'down'):
        try:
            return arm_ik(target, approach, roll, geom, elbow)
        except Unreachable as error:
            errors.append(f'cotovelo {elbow}: {error}')
    raise Unreachable('; '.join(errors))


class Unreachable(Exception):
    """O alvo não tem solução de cinemática inversa, com o motivo na mensagem."""


# =============================================================================
#  Garra
# =============================================================================

def gripper_targets(opening, geom=ARM):
    """Posições das duas juntas da garra para uma abertura de 0 (fechada) a 1.

    As duas juntas são prismáticas e assimétricas (uma vai de 0 a +25 mm, a
    outra de -50 mm a 0): interpolar as duas juntas é mais simples e mais
    robusto do que inverter a abertura em milímetros.
    """
    opening = max(0.0, min(1.0, opening))
    return tuple(closed + opening * (open_ - closed)
                 for closed, open_ in zip(geom.gripper_closed, geom.gripper_open))


def gripper_opening(j2, geom=ARM):
    """Abertura (0 a 1) a partir da posição da junta 2 da garra.

    É a inversa de `gripper_targets`. Usa a junta 2 porque é ela quem comanda:
    a junta 1 apenas a segue (ver `coppelia_bridge.drive_gripper`), e numa
    parada contra o objeto a seguidora fica para trás.
    """
    span = geom.gripper_open[1] - geom.gripper_closed[1]
    return max(0.0, min(1.0, (j2 - geom.gripper_closed[1]) / span))


#: Espessura de cada pastilha dos dedos [m], medida na cena. Entra na conta
#: porque o que segura o objeto é a FACE interna do dedo, não o centro dele.
FINGER_THICKNESS = 0.014


def gripper_gap(j1, j2, geom=ARM):
    """Vão livre entre as faces internas dos dedos, em metros.

    As duas juntas são prismáticas, então a distância entre os dedos é afim nas
    posições delas. Os dois pontos medidos na cena fixam a reta:

        juntas (0, 0)            -> dedos a 40,3 mm de centro a centro
        juntas (0,025, -0,050)   -> dedos a 90,3 mm

    isto é, 50 mm de abertura para 75 mm de curso somado, de onde o fator 2/3.
    Descontada a espessura das pastilhas, sai o vão útil: de 26 a 76 mm.

    Serve para saber se a garra PEGOU algo: fechando no vazio ela chega aos
    26 mm; parada acima disso, há um objeto entre os dedos. É assim que
    `arm_control` detecta a preensão, sem precisar de sensor de força.
    """
    centers = 0.0403 + (j1 - j2) * (2.0 / 3.0)
    return centers - FINGER_THICKNESS


def gripper_opening_for(width, clearance=0.012, geom=ARM):
    """Abertura (0 a 1) para envolver um objeto de `width` metros de largura.

    `clearance` é a folga somada de cada lado, para a garra descer em volta do
    objeto sem raspá-lo. Devolve None quando o objeto não cabe na garra: é o
    caso da banana em tamanho original (100 mm de largura contra 76 mm de vão
    útil), e foi por isso que a cena usa bananas reduzidas.
    """
    needed = width + 2.0 * clearance
    if needed > geom.gripper_gap_open:
        return None
    span = geom.gripper_gap_open - geom.gripper_gap_closed
    return max(0.0, min(1.0, (needed - geom.gripper_gap_closed) / span))


# =============================================================================
#  Pose de preensão
# =============================================================================

def grasp_roll(object_yaw, q0, geom=ARM):
    """Junta 4 para os dedos fecharem ATRAVESSADOS no objeto.

    object_yaw   azimute do eixo MAIOR do objeto, no frame youBot_ref [rad]
    q0           valor da junta 0 na solução da inversa [rad]

    Na preensão por cima o eixo da ferramenta fica vertical, e então a junta 4
    gira a direção de abertura dos dedos no plano horizontal. Medindo no
    simulador (`scene/measure_arm.py --roll`), o azimute da abertura é

        azimute = q0 - q4

    exatamente — a junta 0 leva os dedos consigo e a junta 4 desconta, porque
    na preensão por cima o eixo dela aponta para baixo. (O sinal foi medido, e
    não deduzido: era o candidato natural a erro silencioso.)

    Os dedos têm de fechar PERPENDICULARES ao eixo maior do objeto — uma banana
    de 124 mm de comprimento não cabe entre eles, mas os 60 mm da largura
    cabem — então a abertura pedida é `object_yaw` + 90°, e daí

        q4 = q0 - (object_yaw + 90°)

    O resultado é posto no intervalo da junta 4 (±90°) pela simetria da garra:
    abrir os dedos na direção ψ ou em ψ + 180° é a mesma preensão.
    """
    roll = wrap(q0 - (object_yaw + math.pi / 2.0))
    low, high = geom.limits[4]
    for candidate in (roll, wrap(roll + math.pi), wrap(roll - math.pi)):
        if low <= candidate <= high:
            return candidate
    # Nenhum dos dois lados cabe: devolve o mais próximo do limite, que ainda
    # pega o objeto de través se ele for mais curto que o vão.
    return max(low, min(high, roll))


def base_pose_for_grasp(target_xy, approach_yaw, reach, geom=ARM):
    """Pose (x, y, yaw) da BASE para o braço alcançar o objeto por cima.

    target_xy      (x, y) do objeto, no frame do mundo
    approach_yaw   rumo com que o robô chega ao objeto [rad], na convenção do
                   ROS (o robô olhando para +x do próprio corpo)
    reach          distância pedida do EIXO DA JUNTA 0 ao objeto [m]

    O braço não fica no centro da base: está `axis_y` = 0,166 m à frente dele.
    Então a distância que o controlador de posição tem de cumprir, do CENTRO do
    robô ao objeto, é `reach + axis_y`. Esquecer esse deslocamento é o erro que
    deixa o objeto 17 cm fora do alcance com o robô aparentemente no lugar
    certo.
    """
    distance = reach + geom.axis_y
    return (target_xy[0] - distance * math.cos(approach_yaw),
            target_xy[1] - distance * math.sin(approach_yaw),
            approach_yaw)


# =============================================================================
#  As duas convenções de "frente"
# =============================================================================
#
# A cena e a convenção do ROS discordam sobre qual eixo é a frente do robô:
#
#     youBot_ref (a cena)   frente = +y,  esquerda = -x
#     ROS (REP 103)         frente = +x,  esquerda = +y
#
# A ponte publica a pose do robô já na convenção do ROS, como a ponte do
# exercício anterior fazia, e é nela que o planejador e o controlador de
# posição trabalham. Mas a cinemática inversa do braço é escrita no frame da
# cena, porque é lá que as juntas foram medidas. Então todo ponto que vai do
# mundo do ROS para o braço passa por aqui — uma função com nome, usada em um
# só lugar, em vez de um sinal trocado espalhado pelo código.

def ros_to_ref(point):
    """Ponto do frame do robô na convenção ROS -> frame youBot_ref da cena."""
    return (-point[1], point[0], point[2])


def ref_to_ros(point):
    """Ponto do frame youBot_ref da cena -> frame do robô na convenção ROS."""
    return (point[1], -point[0], point[2])
