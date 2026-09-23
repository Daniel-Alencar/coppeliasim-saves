"""Ponte ROS 2 <-> CoppeliaSim para o docking do /myRobot.

Expõe pelo ROS 2 tudo o que um controlador de docking externo precisa, lendo e
escrevendo os sinais da cena pela ZeroMQ Remote API:

    CoppeliaSim                              ROS 2 (namespace myRobot)
    <handle>Battery        (float)  --->     battery                       BatteryState
    <handle>Charging       (int)    --->     charging                      std_msgs/Bool
    <handle>signalStrength (float)  --->     charging_base/strengthSignal  std_msgs/Float32
    <handle>relativeAngle  (float)  --->     charging_base/relativeAngle   std_msgs/Float32
    <handle>Docking        (int)    <---     docking                       std_msgs/Bool
    motores                         <---     cmd_vel                       geometry_msgs/Twist

<handle> é o handle inteiro do robô na cena, por exemplo 84Battery.

Os tópicos são relativos: com o namespace myRobot do launch viram
/myRobot/battery, /myRobot/cmd_vel etc.

Como o cmd_vel chega aos motores (parâmetro motor_mode)
-------------------------------------------------------
A cena "Evaluation scene3.2_students.ttt" já traz o script
/myRobot/python_controler habilitado, e ele chama setJointTargetVelocity nos
dois motores a CADA passo de simulação. Escrever nas juntas pela Remote API
disputa com ele a cada passo, e o robô anda aos solavancos. O script prevê essa
situação e oferece uma via oficial de override, dois sinais float que ele lê e
apaga a cada passo:

    <handle>leftVel  / <handle>rightVel   (m/s na roda, não rad/s)

Daí os dois modos:

    motor_mode='joint'  (padrão)  chama setJointTargetVelocity direto, que é o
        que o enunciado pede. EXIGE desabilitar o /myRobot/python_controler na
        cena, senão ele sobrescreve os comandos.

    motor_mode='signal'           escreve os sinais de override. Convive com o
        python_controler (joystick, rótulos e checkbox "docking" seguem vivos),
        mas é MUITO pior: o script apaga o sinal assim que o lê, e cada chamada
        da Remote API custa ~12 ms com a simulação rodando, então na prática só
        parte dos passos recebe comando. Medido nesta cena, pedindo 0,2 m/s e
        escrevendo no ritmo máximo por 4 s:

            motor_mode='joint'   0,689 m  ->  0,172 m/s   (86 % do pedido)
            motor_mode='signal'  0,194 m  ->  0,047 m/s   (24 % do pedido)

        Use 'signal' só se precisar do joystick da cena junto com o ROS 2.

Esta ponte não implementa o docking autônomo: só a comunicação.
"""

import math        # math.dist, math.nan
import signal      # tratamento manual do Ctrl+C (SIGINT) e do SIGTERM
import threading   # threading.Event: flag de parada segura entre sinal e laço
import time        # time.monotonic: relógio que nunca volta, ideal para prazos

# Coppelia ZeroMQ Remote API: chama as funções sim.* a partir de outro processo.
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32

# Nomes dos sinais do beacon, na ordem em que são procurados. O primeiro par é o
# do enunciado; o segundo é o que o script /chargingBase/beacon da cena
# "Evaluation scene3.2_students.ttt" escreve quando recebe o sinal "Beacon".
BEACON_SIGNALS = [
    ('signalStrength', 'relativeAngle'),
    ('StrengthSignal', 'RelativeAngle'),
]


class DockingBridge(Node):
    """Nó ROS 2 que liga os sinais de docking do CoppeliaSim a tópicos ROS."""

    def __init__(self):
        super().__init__('remoteAPI_ROS2_bridge')

        # --- Parâmetros ------------------------------------------------------
        self.declare_parameter('host', 'localhost')      # onde está o CoppeliaSim
        self.declare_parameter('port', 23000)            # porta da ZeroMQ Remote API
        self.declare_parameter('robot', '/myRobot')      # caminho do robô na cena
        self.declare_parameter('wheel_radius', 0.05)     # raio da roda, em metros
        # 0.0 = medir a distância entre as rodas na própria cena.
        self.declare_parameter('wheel_separation', 0.0)
        # Nesta cena as juntas estão montadas de modo que velocidade negativa
        # faz o robô andar para a frente (o python_controler da cena usa -vel/r).
        self.declare_parameter('motor_sign', -1.0)
        # 'signal' = sinais <handle>leftVel/<handle>rightVel, convivendo com o
        # python_controler da cena; 'joint' = setJointTargetVelocity direto.
        self.declare_parameter('motor_mode', 'joint')
        self.declare_parameter('max_wheel_speed', 10.0)  # limite por roda, em rad/s
        self.declare_parameter('cmd_timeout', 0.5)       # segundos até parar sem comando
        # Segundos sem leitura nova do beacon até considerar que o sinal sumiu.
        self.declare_parameter('beacon_timeout', 0.5)
        # Com a bateria em 0 % o robô não anda, como fazia o python_controler.
        self.declare_parameter('stop_when_battery_empty', True)
        # Com a simulação rodando, cada chamada da Remote API custa ~12 ms
        # (medido): ela só é atendida entre passos de simulação. Isso dá um teto
        # de ~80 chamadas por segundo para a ponte inteira, e é o que limita
        # estas duas frequências — subi-las não acelera nada, só enfileira.
        self.declare_parameter('rate', 10.0)             # sensores, em Hz (~5 chamadas por ciclo)
        self.declare_parameter('motor_rate', 20.0)       # motores, em Hz (2 chamadas por ciclo)
        self.declare_parameter('autostart', True)        # dar play se a simulação estiver parada

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        robot = self.get_parameter('robot').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.motor_sign = self.get_parameter('motor_sign').value
        self.motor_mode = self.get_parameter('motor_mode').value
        if self.motor_mode not in ('signal', 'joint'):
            raise SystemExit(
                f'motor_mode inválido: "{self.motor_mode}". Use "signal" ou "joint".'
            )
        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.beacon_timeout = self.get_parameter('beacon_timeout').value
        self.stop_when_battery_empty = self.get_parameter('stop_when_battery_empty').value

        # --- Conexão com o CoppeliaSim ---------------------------------------
        self.sim = self.connect(host, port)
        self.robotHandle = self.find(robot)
        self.leftMotorHandle = self.find(f'{robot}/leftMotor')
        self.rightMotorHandle = self.find(f'{robot}/rightMotor')
        # Os sinais do robô são nomeados com o handle dele na frente, por
        # exemplo "84Battery". str() do handle é exatamente esse prefixo.
        self.prefix = str(self.robotHandle)
        self.check_motor_scripts(robot)

        self.wheel_separation = self.get_parameter('wheel_separation').value
        if self.wheel_separation <= 0.0:
            self.wheel_separation = self.measure_wheel_separation()
        self.get_logger().info(
            f'robô {robot} (handle {self.robotHandle}) | raio da roda: '
            f'{self.wheel_radius:.3f} m | distância entre rodas: '
            f'{self.wheel_separation:.3f} m'
        )

        # --- Estado da simulação ---------------------------------------------
        self.started_here = (
            self.get_parameter('autostart').value
            and self.sim.getSimulationState() == self.sim.simulation_stopped
        )
        if self.started_here:
            self.sim.startSimulation()
        elif self.sim.getSimulationState() == self.sim.simulation_paused:
            self.get_logger().warn(
                'a simulação está pausada: nada muda até o play.'
            )

        # --- Estado interno --------------------------------------------------
        # (esquerda, direita) em m/s NA RODA. Guardar em m/s, e não em rad/s, é o
        # que permite os dois motor_mode: é a unidade que o python_controler
        # espera nos sinais, e vira rad/s dividindo pelo raio no modo 'joint'.
        self.velocity = (0.0, 0.0)
        self.deadline = 0.0            # instante em que o último cmd_vel expira
        self.battery = None            # último nível lido, para o corte em 0 %
        self.last_battery = None       # nível anterior, para saber se sobe ou desce
        self.charging = False          # último estado de carga conhecido
        self.beacon = (0.0, math.nan)  # (força, ângulo) da última leitura válida
        self.beacon_time = -math.inf   # instante dessa leitura
        self.battery_empty_warned = False

        # --- Comunicação ROS -------------------------------------------------
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Bool, 'docking', self.docking_callback, 10)
        self.battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self.charging_pub = self.create_publisher(Bool, 'charging', 10)
        self.strength_pub = self.create_publisher(Float32, 'charging_base/strengthSignal', 10)
        self.angle_pub = self.create_publisher(Float32, 'charging_base/relativeAngle', 10)

        # Dois timers: os sensores não precisam da pressa dos motores.
        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.create_timer(
            1.0 / self.get_parameter('motor_rate').value, self.apply_motors
        )
        self.get_logger().info(
            f'ponte pronta (motor_mode={self.motor_mode}). comandos em '
            f'{self.resolve_topic_name("cmd_vel")} e '
            f'{self.resolve_topic_name("docking")}'
        )

    # =========================================================================
    #  Conexão e cena
    # =========================================================================

    def connect(self, host, port):
        """Conecta ao CoppeliaSim, explicando o motivo quando não dá."""
        try:
            self.client = RemoteAPIClient(host, port)
            return self.client.require('sim')
        except Exception:
            raise SystemExit(
                f'Não consegui falar com o CoppeliaSim em {host}:{port}.\n'
                '  - O simulador está aberto, com a cena de docking?\n'
                '  - Ele responde? Um child script da cena preso em laço trava a\n'
                '    thread principal: a porta continua aberta, mas nenhuma chamada\n'
                '    é respondida. Pare a simulação e desabilite esse script.'
            )

    def find(self, path):
        """Busca um objeto e, se não achar, mostra o que existe na cena."""
        handle = self.sim.getObject(path, {'noError': True})
        if handle != -1:
            return handle
        existentes = [
            self.sim.getObjectAlias(h, 2)
            for h in self.sim.getObjectsInTree(self.sim.handle_scene)
        ]
        raise SystemExit(
            f'Objeto "{path}" não existe na cena aberta.\n'
            'Objetos disponíveis:\n  '
            + '\n  '.join(existentes)
            + '\nAjuste o parâmetro robot, por exemplo:\n'
            '  ros2 launch robot_docking robot_docking.launch.py robot:=/meuRobo'
        )

    def check_motor_scripts(self, robot):
        """Confere se os scripts da cena combinam com o motor_mode escolhido.

        Só os scripts que escrevem nos motores importam aqui. Os demais
        (bateria, odometria, encoders, sensor de docking) precisam continuar
        habilitados nos dois modos.
        """
        rivals = []
        for h in self.sim.getObjectsInTree(
            self.sim.getObject(robot), self.sim.sceneobject_script
        ):
            if self.sim.getBoolProperty(h, 'scriptDisabled'):
                continue
            if 'setJointTargetVelocity' in self.sim.getStringProperty(h, 'code'):
                rivals.append(self.sim.getObjectAlias(h, 2))

        if self.motor_mode == 'joint' and rivals:
            # Os dois escrevem nas mesmas juntas: quem escrever por último vence,
            # e o script da cena escreve a cada passo de simulação.
            self.get_logger().warn(
                'motor_mode=joint, mas estes scripts da cena escrevem nos motores '
                f'a cada passo e vão sobrescrever o cmd_vel: {", ".join(rivals)}. '
                'Desabilite-os na cena, ou use motor_mode:=signal.'
            )
        elif self.motor_mode == 'signal' and not rivals:
            # Ninguém lê os sinais leftVel/rightVel: o robô não vai sair do lugar.
            self.get_logger().warn(
                'motor_mode=signal, mas nenhum script habilitado do robô lê os '
                f'sinais {self.prefix}leftVel/{self.prefix}rightVel. Habilite o '
                'python_controler na cena, ou use motor_mode:=joint.'
            )

    def measure_wheel_separation(self):
        """Distância entre os dois motores, lida direto da cena."""
        left = self.sim.getObjectPosition(self.leftMotorHandle, self.sim.handle_world)
        right = self.sim.getObjectPosition(self.rightMotorHandle, self.sim.handle_world)
        return math.dist(left, right)

    # =========================================================================
    #  ROS -> CoppeliaSim
    # =========================================================================

    def cmd_vel_callback(self, msg: Twist):
        """Guarda as velocidades de roda pedidas pelo Twist; apply_motors aplica."""
        v = msg.linear.x    # m/s, positivo = para a frente
        w = msg.angular.z   # rad/s, positivo = virar à esquerda

        # Cinemática inversa do robô diferencial: cada roda está a L/2 do centro,
        # então sua velocidade linear é v ∓ w·L/2. É a mesma conta que o
        # python_controler faz com os sliders do joystick.
        half_track = self.wheel_separation / 2.0
        left = v - w * half_track     # m/s
        right = v + w * half_track    # m/s

        # Satura mantendo a proporção entre as rodas (mesma curva, mais lenta).
        # max_wheel_speed está em rad/s; vezes o raio dá o limite em m/s.
        limit = self.max_wheel_speed * self.wheel_radius
        excess = max(abs(left), abs(right)) / limit
        if excess > 1.0:
            left, right = left / excess, right / excess

        self.velocity = (left, right)
        self.deadline = time.monotonic() + self.cmd_timeout

    def docking_callback(self, msg: Bool):
        """Liga (1) ou desliga (0) o modo de docking na cena.

        Quem consome esse sinal é o python_controler: ele reage a 1, apaga o
        sinal e marca o checkbox "docking" do joystick, que é a confirmação
        visível de que o comando chegou. Ele ignora o 0 e não existe, nesta
        cena, quem desmarque o checkbox ou desligue o modo — o comportamento
        de docking em si é o que será implementado na próxima atividade.
        """
        mode = 1 if msg.data else 0
        self.sim.setInt32Signal(self.prefix + 'Docking', mode)
        self.get_logger().info(f'modo de docking: {mode}')

    # =========================================================================
    #  CoppeliaSim -> ROS
    # =========================================================================

    def read_battery(self):
        """Nível da bateria em %, ou None antes do primeiro valor da cena."""
        return self.sim.getFloatSignal(self.prefix + 'Battery')

    def read_charging(self, battery):
        """Estado de carga, robusto ao sinal que a própria cena apaga.

        O script /chargingBase/beacon escreve Charging = 1 a cada segundo com o
        robô na base e Charging = 0 uma única vez quando ele sai. O script
        /myRobot/battery lê e apaga o sinal a cada segundo. Por isso a ponte
        guarda o último valor visto e, se ele for perdido, confere com a
        bateria, que sobe carregando e desce descarregando.
        """
        value = self.sim.getInt32Signal(self.prefix + 'Charging')
        if value is not None:
            self.charging = value == 1

        if battery is not None and self.last_battery is not None:
            if battery > self.last_battery:
                self.charging = True
            elif battery < self.last_battery:
                self.charging = False
        if battery is not None:
            self.last_battery = battery
        return self.charging

    def read_beacon(self):
        """(força, ângulo) do beacon; (0, nan) quando o robô não o detecta.

        O beacon da cena só calcula a leitura para o robô cujo handle está no
        sinal global "Beacon", e só escreve quando o robô está dentro do feixe.
        Como ele nunca apaga os sinais, a ponte os apaga depois de ler: se não
        chegar leitura nova em beacon_timeout segundos, o robô saiu do feixe.
        """
        self.sim.setInt32Signal('Beacon', self.robotHandle)

        now = time.monotonic()
        for strength_name, angle_name in BEACON_SIGNALS:
            strength = self.sim.getFloatSignal(self.prefix + strength_name)
            angle = self.sim.getFloatSignal(self.prefix + angle_name)
            if strength is not None and angle is not None:
                self.sim.clearFloatSignal(self.prefix + strength_name)
                self.sim.clearFloatSignal(self.prefix + angle_name)
                self.beacon = (strength, angle)
                self.beacon_time = now
                break

        if now - self.beacon_time > self.beacon_timeout:
            return 0.0, math.nan
        return self.beacon

    def write_motors(self, left, right):
        """Entrega (esquerda, direita), em m/s na roda, conforme o motor_mode."""
        if self.motor_mode == 'signal':
            # O python_controler lê estes dois sinais, divide pelo raio da roda e
            # aplica o sinal negativo. Por isso aqui vai m/s, cru.
            self.sim.setFloatSignal(self.prefix + 'leftVel', float(left))
            self.sim.setFloatSignal(self.prefix + 'rightVel', float(right))
        else:
            # Sem o script no meio, as duas conversões são nossas.
            self.sim.setJointTargetVelocity(
                self.leftMotorHandle, self.motor_sign * left / self.wheel_radius
            )
            self.sim.setJointTargetVelocity(
                self.rightMotorHandle, self.motor_sign * right / self.wheel_radius
            )

    def apply_motors(self):
        """Executado pelo timer dos motores: repõe o último comando na cena.

        Precisa repor a cada ciclo porque o python_controler apaga os sinais
        assim que os lê: um ciclo sem escrita e ele volta a obedecer o joystick.
        """
        # Sem comando recente o robô para sozinho, mesmo que o controlador caia.
        if time.monotonic() >= self.deadline:
            self.velocity = (0.0, 0.0)
        left, right = self.velocity

        # Mesmo corte que o python_controler faz: bateria zerada, robô parado.
        if self.stop_when_battery_empty and self.battery is not None and self.battery <= 0.0:
            left = right = 0.0
            if not self.battery_empty_warned:
                self.get_logger().warn('bateria em 0 %: motores parados.')
                self.battery_empty_warned = True
        elif self.battery is not None and self.battery > 0.0:
            self.battery_empty_warned = False

        self.write_motors(left, right)

    def step(self):
        """Executado pelo timer dos sensores: lê a cena e publica no ROS."""
        battery = self.read_battery()
        self.battery = battery

        # --- Carga e bateria -------------------------------------------------
        charging = self.read_charging(battery)
        self.charging_pub.publish(Bool(data=charging))

        if battery is not None:
            state = BatteryState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.header.frame_id = 'myRobot'
            # Grandezas que a cena não simula ficam NaN, como pede a mensagem.
            state.voltage = math.nan
            state.current = math.nan
            state.charge = math.nan
            state.capacity = math.nan
            state.design_capacity = math.nan
            state.temperature = math.nan
            state.percentage = battery / 100.0   # a mensagem usa 0 a 1
            if charging and battery >= 100.0:
                state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
            elif charging:
                state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
            else:
                state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
            state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
            state.present = True
            self.battery_pub.publish(state)

        # --- Beacon ----------------------------------------------------------
        strength, angle = self.read_beacon()
        self.strength_pub.publish(Float32(data=float(strength)))
        self.angle_pub.publish(Float32(data=float(angle)))

    def stop(self):
        """Para os motores e, se foi a ponte que deu o play, para a simulação."""
        self.write_motors(0.0, 0.0)
        # No modo signal as juntas ficariam com o último alvo caso o script não
        # chegue a ler o sinal zero; zerá-las também é barato e garante a parada.
        self.sim.setJointTargetVelocity(self.leftMotorHandle, 0.0)
        self.sim.setJointTargetVelocity(self.rightMotorHandle, 0.0)
        if self.started_here:
            self.sim.stopSimulation()


def main(args=None):
    # O Ctrl+C só levanta uma flag. Deixar o rclpy ou o KeyboardInterrupt cortarem
    # o laço no meio de uma chamada ZeroMQ invalida o socket, e aí o comando de
    # parada dos motores nunca chega ao simulador.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_requested.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_requested.set())

    try:
        node = DockingBridge()
    except SystemExit as erro:
        print(erro)
        rclpy.shutdown()
        return 1
    try:
        while not stop_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
