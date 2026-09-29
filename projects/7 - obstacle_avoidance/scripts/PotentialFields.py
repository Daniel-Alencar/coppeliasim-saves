# =============================================================================
#  Navegação por campos potenciais — pega bananas, desvia de poops
#
#  O robô é atraído pela banana alvo e repelido por cada poop. A soma das duas
#  contribuições dá uma força resultante, e é a direção dela que comanda o
#  robô a cada passo. Não existe caminho nem lista de pontos: a trajetória
#  aparece sozinha do campo.
#
#      F = F_atrativa(banana alvo) + Σ F_repulsiva(poop)
#
#  As posições vêm dos sinais 'banana' e 'poop' da cena, no formato
#  [x1, y1, x2, y2, ...], lidos com sim.unpackTable.
#
#  Como usar: cole este arquivo num child script Python pendurado no /myRobot
#  (na cena pega_banana_potential_field.ttt dá para reaproveitar o
#  /myRobot/python_controler, que já vem desabilitado) e dê play.
#
#  Também roda como cliente externo, pela ZeroMQ Remote API:
#      /usr/bin/python3 PotentialFields.py
#  Nesse modo cada chamada ao simulador custa alguns milissegundos, então o
#  laço fica mais lento que o script embarcado, que roda a cada passo da
#  simulação.
# =============================================================================

import math

# ----------------------------- geometria -------------------------------------
WHEEL_RADIUS = 0.05          # raio da roda [m]
TRACK_WIDTH = 0.2            # distância entre as rodas [m]
# Nesta cena as juntas estão montadas de forma que velocidade negativa faz o
# robô andar para a frente (mesma convenção do python_controler da cena).
MOTOR_SIGN = -1.0
# A frente do robô é o eixo +y do referencial dele.

# ----------------------------- campo potencial -------------------------------
K_ATT = 1.0                  # ganho da força atrativa
ATT_RANGE = 1.0              # acima desta distância a atração satura [m]

K_REP = 0.08                 # ganho da força repulsiva
D0_POOP = 0.55               # raio de influência de cada poop [m]
D_MIN = 0.08                 # distância mínima usada na conta, evita estouro [m]

F_MAX = 3.0                  # saturação do módulo da resultante

# ----------------------------- controle --------------------------------------
V_MAX = 0.25                 # velocidade de avanço [m/s]
W_MAX = 1.8                  # velocidade de giro [rad/s]
K_HEADING = 3.0              # rad/s por rad de erro de rumo
# Com erro de rumo maior que isto o robô gira parado, em vez de avançar torto.
TURN_IN_PLACE = math.radians(75.0)

# A cena só recolhe a banana quando ela passa sob o sensor poopximity, que
# aponta para baixo no centro do robô. Por isso o alvo só conta como alcançado
# bem em cima dele, e não a 10 cm: senão o robô "pula" bananas que não pegou.
COLLECT_RADIUS = 0.05        # distância que conta como banana alcançada [m]
SLOW_RADIUS = 0.35           # abaixo desta distância o robô desacelera [m]
V_MIN = 0.06                 # velocidade mínima na aproximação final [m/s]
# Se ficar rondando o alvo sem conseguir passar por cima, desiste e vai para a
# próxima, em vez de orbitar para sempre.
GIVE_UP_RADIUS = 0.30        # [m]
GIVE_UP_TIME = 12.0          # [s]

# ----------------------------- mínimos locais --------------------------------
# O campo potencial pode empatar: a repulsão dos poops cancela a atração e o
# robô fica parado ou oscilando sem chegar na banana. Quando o progresso em
# direção ao alvo para, entra uma componente tangencial (perpendicular à
# atração) que contorna o empate. Continua sendo força, não caminho traçado.
STALL_TIME = 2.0             # segundos sem progresso que caracterizam o empate
STALL_PROGRESS = 0.05        # aproximação mínima esperada nesse tempo [m]
SWIRL_TIME = 2.5             # duração da componente tangencial [s]
SWIRL_GAIN = 1.2             # peso dela em relação à atração

VERBOSE = True               # imprime cada banana alcançada


def _wrap(angle):
    """Normaliza um ângulo para (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _unpack_pairs(flat):
    """[x1, y1, x2, y2, ...] -> [(x1, y1), (x2, y2), ...]."""
    if not flat:
        return []
    # O unpackTable devolve um dicionário quando a tabela Lua vem com chaves
    # numéricas esparsas; nesse caso a ordem é a das chaves 1, 2, 3...
    if isinstance(flat, dict):
        flat = [flat[k] for k in sorted(flat, key=lambda v: int(v))]
    return [(float(flat[i]), float(flat[i + 1])) for i in range(0, len(flat) - 1, 2)]


class PotentialFieldNavigator:
    """Campo potencial: atrai para a banana alvo, repele dos poops."""

    def __init__(self, sim, robot, left_motor, right_motor):
        self.sim = sim
        self.robot = robot
        self.left_motor = left_motor
        self.right_motor = right_motor

        # Os sinais só existem depois que o /buildScene roda o sysCall_init
        # dele, e a ordem entre os scripts da cena não é garantida. Por isso o
        # mapa é lido no primeiro passo de atuação, não aqui.
        self.bananas = None
        self.poops = None
        self.pending = []                   # bananas que ainda faltam
        self.target = None                  # banana perseguida agora
        self.collected = 0
        self.waiting_since = None           # desde quando espera os sinais

        # Detecção de empate do campo (mínimo local).
        self.best_distance = math.inf       # menor distância já obtida do alvo
        self.stall_since = None             # desde quando não há progresso
        self.swirl_until = -math.inf        # até quando a tangencial fica ligada
        self.swirl_sign = 1.0               # para que lado ela desvia
        self.near_since = None              # desde quando ronda o alvo de perto
        self.finished = False

    # ------------------------------------------------------------------ cena

    def _read_signal(self, name):
        """Lê um sinal da cena e desempacota, ou None se ele ainda não existe.

        Usa getBufferSignal, e não getStringSignal. O conteúdo é uma tabela Lua
        empacotada, ou seja, bytes quaisquer. Num script Python embarcado, a
        ponte com o Lua tenta mandar o resultado do getStringSignal como texto
        UTF-8, não consegue, e derruba o script com
        "TEXT: not UTF-8 text" — um erro que nem dá para capturar aqui, porque
        acontece na resposta. O getBufferSignal entrega os mesmos bytes sem
        passar por texto, e lê o sinal criado com setStringSignal pela cena.
        """
        getter = getattr(self.sim, 'getBufferSignal', None)
        if getter is None:                      # versões antigas do CoppeliaSim
            getter = self.sim.getStringSignal
        packed = getter(name)
        if packed is None:
            return None
        return self.sim.unpackTable(packed)

    def load_map(self, now):
        """Tenta ler as posições dos sinais. Devolve True quando conseguir."""
        banana = self._read_signal('banana')
        poop = self._read_signal('poop')
        if banana is None or poop is None:
            # O /buildScene cria os sinais no sysCall_init dele, que pode rodar
            # depois deste script. Espera o próximo passo e tenta de novo.
            if self.waiting_since is None:
                self.waiting_since = now
            elif now - self.waiting_since > 5.0:
                self.waiting_since = now
                faltando = 'banana' if banana is None else 'poop'
                self._log(f"ainda esperando o sinal '{faltando}' do /buildScene")
            return False

        self.bananas = _unpack_pairs(banana)
        self.poops = _unpack_pairs(poop)
        self.pending = list(self.bananas)
        self._log(f'{len(self.bananas)} bananas e {len(self.poops)} poops lidos dos sinais')
        return True

    def _log(self, message):
        if VERBOSE:
            print(f'[campo potencial] {message}')

    def pose(self):
        """(x, y, rumo) do robô no mundo; a frente é o eixo +y do robô."""
        m = self.sim.getObjectMatrix(self.robot, self.sim.handle_world)
        # Matriz 3x4 em ordem de linhas: a segunda coluna é o eixo y do robô.
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

    # ------------------------------------------------------------- alvo

    def choose_target(self, x, y):
        """Banana pendente mais próxima. Só o alvo atual atrai o robô."""
        if not self.pending:
            return None
        return min(self.pending, key=lambda b: math.hypot(b[0] - x, b[1] - y))

    def reset_progress(self):
        self.best_distance = math.inf
        self.stall_since = None
        self.swirl_until = -math.inf
        self.near_since = None

    # ------------------------------------------------------------- forças

    def attractive(self, x, y, target):
        """Puxa para a banana alvo, com módulo saturado a K_ATT."""
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return 0.0, 0.0, distance
        # Longe do alvo o módulo é constante; perto, cai junto com a distância.
        scale = K_ATT * min(1.0, distance / ATT_RANGE) / distance
        return dx * scale, dy * scale, distance

    def repulsive(self, x, y):
        """Soma das repulsões dos poops dentro do raio de influência.

        Cada poop usa a forma clássica do campo potencial:
        F = K_REP * (1/d - 1/D0) / d² , apontando do poop para o robô.
        """
        fx = fy = 0.0
        for px, py in self.poops:
            dx, dy = x - px, y - py
            distance = math.hypot(dx, dy)
            if distance > D0_POOP:
                continue
            distance = max(distance, D_MIN)
            magnitude = K_REP * (1.0 / distance - 1.0 / D0_POOP) / (distance * distance)
            fx += magnitude * dx / distance
            fy += magnitude * dy / distance
        return fx, fy

    def swirl(self, now, distance, ax, ay):
        """Componente tangencial que desempata um mínimo local.

        Devolve (0, 0) enquanto o robô estiver se aproximando do alvo.
        """
        # Progresso é reduzir a menor distância já alcançada até o alvo.
        if distance < self.best_distance - STALL_PROGRESS:
            self.best_distance = distance
            self.stall_since = None
        elif self.stall_since is None:
            self.stall_since = now

        if self.stall_since is not None and now - self.stall_since >= STALL_TIME:
            # Escolhe um lado e mantém por SWIRL_TIME, para não ficar alternando.
            if now > self.swirl_until:
                self.swirl_sign = -self.swirl_sign
                self.swirl_until = now + SWIRL_TIME
                self._log('campo empatado, contornando pela tangente')
            self.stall_since = None
            self.best_distance = distance

        if now < self.swirl_until:
            # Perpendicular à atração, girada para o lado escolhido.
            return -ay * SWIRL_GAIN * self.swirl_sign, ax * SWIRL_GAIN * self.swirl_sign
        return 0.0, 0.0

    # ------------------------------------------------------------- laço

    def step(self, now):
        """Um ciclo de controle. Devolve False quando não há mais bananas."""
        if self.finished:
            return False

        # Primeiro passo: lê o mapa dos sinais da cena.
        if self.bananas is None:
            if not self.load_map(now):
                self.stop()
                return True

        x, y, theta = self.pose()

        # Alvo alcançado? O script /dirt_script da cena recolhe a banana quando
        # ela passa sob o sensor poopximity, no centro do robô.
        if self.target is not None:
            to_target = math.hypot(self.target[0] - x, self.target[1] - y)
            if to_target <= COLLECT_RADIUS:
                self.pending.remove(self.target)
                self.collected += 1
                self._log(f'banana {self.collected}/{len(self.bananas)} alcançada')
                self.target = None
                self.reset_progress()
            elif to_target <= GIVE_UP_RADIUS:
                # Rondando de perto sem conseguir passar por cima: a repulsão de
                # um poop vizinho pode estar impedindo. Desiste e segue.
                if self.near_since is None:
                    self.near_since = now
                elif now - self.near_since > GIVE_UP_TIME:
                    self.pending.remove(self.target)
                    self._log('não consegui passar por cima desta banana, indo para a próxima')
                    self.target = None
                    self.reset_progress()
            else:
                self.near_since = None

        if self.target is None:
            self.target = self.choose_target(x, y)
            self.reset_progress()
            if self.target is None:
                self.stop()
                self.finished = True
                self._log(f'todas as {self.collected} bananas coletadas, parando')
                return False

        # Resultante: atração da banana + repulsão dos poops (+ tangente, se travado).
        ax, ay, distance = self.attractive(x, y, self.target)
        rx, ry = self.repulsive(x, y)
        sx, sy = self.swirl(now, distance, ax, ay)
        fx, fy = ax + rx + sx, ay + ry + sy

        magnitude = math.hypot(fx, fy)
        if magnitude > F_MAX:
            fx, fy = fx * F_MAX / magnitude, fy * F_MAX / magnitude
            magnitude = F_MAX

        if magnitude < 1e-6:
            # Sem direção definida: gira no lugar até o campo voltar a apontar.
            self.set_velocity(0.0, W_MAX * 0.5)
            return True

        # A direção da resultante é o rumo desejado; o erro vira giro.
        error = _wrap(math.atan2(fy, fx) - theta)
        w = max(-W_MAX, min(W_MAX, K_HEADING * error))
        if abs(error) > TURN_IN_PLACE:
            v = 0.0                      # muito torto: acerta o rumo primeiro
        else:
            # Avança conforme aponta para a força, e desacelera na chegada para
            # passar em cima da banana em vez de raspar por perto.
            speed = V_MAX
            if distance < SLOW_RADIUS:
                speed = max(V_MIN, V_MAX * distance / SLOW_RADIUS)
            v = speed * math.cos(error)
        self.set_velocity(v, w)
        return True


# =============================================================================
#  Modo script embarcado na cena (child script Python do /myRobot)
# =============================================================================

def sysCall_init():
    sim = require('sim')                                   # noqa: F821
    self.sim = sim                                         # noqa: F821
    robot = sim.getObject('/myRobot')
    self.navigator = PotentialFieldNavigator(              # noqa: F821
        sim, robot,
        sim.getObject('/myRobot/leftMotor'),
        sim.getObject('/myRobot/rightMotor'),
    )


def sysCall_actuation():
    self.navigator.step(self.sim.getSimulationTime())      # noqa: F821


def sysCall_cleanup():
    try:
        self.navigator.stop()                              # noqa: F821
    except Exception:
        pass


# =============================================================================
#  Modo cliente externo (ZeroMQ Remote API)
# =============================================================================

def main():
    import time
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    sim = RemoteAPIClient().require('sim')
    if sim.getSimulationState() == sim.simulation_stopped:
        sim.startSimulation()
        time.sleep(0.2)
    # Os sinais podem demorar alguns passos para aparecer; o step() espera.

    robot = sim.getObject('/myRobot')
    navigator = PotentialFieldNavigator(
        sim, robot,
        sim.getObject('/myRobot/leftMotor'),
        sim.getObject('/myRobot/rightMotor'),
    )
    try:
        while navigator.step(sim.getSimulationTime()):
            if sim.getSimulationState() == sim.simulation_stopped:
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            navigator.stop()
        except Exception:
            pass
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
