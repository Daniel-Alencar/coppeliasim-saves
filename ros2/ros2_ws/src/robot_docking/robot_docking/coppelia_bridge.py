"""Ponte ROS 2 <-> CoppeliaSim para o docking do /myRobot.

Expõe pelo ROS 2 tudo o que um controlador de docking externo precisa, lendo e
escrevendo os sinais da cena pela ZeroMQ Remote API:

    CoppeliaSim                              ROS 2 (namespace myRobot)
    <handle>Battery        (float)  --->     battery                       BatteryState
    <handle>Charging       (int)    --->     charging                      std_msgs/Bool
    <handle>signalStrength (float)  --->     charging_base/strengthSignal  std_msgs/Float32
    <handle>relativeAngle  (float)  --->     charging_base/relativeAngle   std_msgs/Float32
    <handle>Docking        (int)    <---     docking                       std_msgs/Bool
    leftMotor / rightMotor (juntas) <---     cmd_vel                       geometry_msgs/Twist

<handle> é o handle inteiro do robô na cena, por exemplo 84Battery.

Os tópicos são relativos: com o namespace myRobot do launch viram
/myRobot/battery, /myRobot/cmd_vel etc.

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
        self.declare_parameter('max_wheel_speed', 10.0)  # limite por roda, em rad/s
        self.declare_parameter('cmd_timeout', 0.5)       # segundos até parar sem comando
        # Segundos sem leitura nova do beacon até considerar que o sinal sumiu.
        self.declare_parameter('beacon_timeout', 0.5)
        # Com a bateria em 0 % o robô não anda, como fazia o python_controler.
        self.declare_parameter('stop_when_battery_empty', True)
        self.declare_parameter('rate', 20.0)             # frequência do laço step(), em Hz
        self.declare_parameter('autostart', True)        # dar play se a simulação estiver parada

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        robot = self.get_parameter('robot').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.motor_sign = self.get_parameter('motor_sign').value
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
        self.warn_motor_scripts(robot)

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
        self.velocity = (0.0, 0.0)     # (esquerda, direita) em rad/s, já sem o motor_sign
        self.deadline = 0.0            # instante em que o último cmd_vel expira
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

        self.create_timer(1.0 / self.get_parameter('rate').value, self.step)
        self.get_logger().info(
            f'ponte pronta. comandos em {self.resolve_topic_name("cmd_vel")} e '
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

    def warn_motor_scripts(self, robot):
        """Avisa sobre scripts do robô que também escrevem nos motores.

        Só esses disputam o controle com a ponte. Os demais (bateria,
        odometria, encoders, sensor de docking) precisam continuar habilitados.
        """
        scripts = self.sim.getObjectsInTree(
            self.sim.getObject(robot), self.sim.sceneobject_script
        )
        for h in scripts:
            if self.sim.getBoolProperty(h, 'scriptDisabled'):
                continue
            if 'setJointTargetVelocity' in self.sim.getStringProperty(h, 'code'):
                self.get_logger().warn(
                    f'o script {self.sim.getObjectAlias(h, 2)} está habilitado e '
                    'escreve nos motores a cada passo, sobrescrevendo o cmd_vel. '
                    'Desabilite-o na cena.'
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
        """Guarda as velocidades de roda pedidas pelo Twist; o próximo step() aplica."""
        v = msg.linear.x    # m/s, positivo = para a frente
        w = msg.angular.z   # rad/s, positivo = virar à esquerda

        # Cinemática inversa do robô diferencial: cada roda está a L/2 do centro.
        half_track = self.wheel_separation / 2.0
        left = (v - w * half_track) / self.wheel_radius
        right = (v + w * half_track) / self.wheel_radius

        # Satura mantendo a proporção entre as rodas (mesma curva, mais lenta).
        excess = max(abs(left), abs(right)) / self.max_wheel_speed
        if excess > 1.0:
            left, right = left / excess, right / excess

        self.velocity = (left, right)
        self.deadline = time.monotonic() + self.cmd_timeout

    def docking_callback(self, msg: Bool):
        """Liga (1) ou desliga (0) o modo de docking na cena."""
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

    def step(self):
        """Executado pelo timer: aplica o comando atual e publica as leituras."""
        battery = self.read_battery()

        # --- Motores ---------------------------------------------------------
        if time.monotonic() >= self.deadline:
            self.velocity = (0.0, 0.0)
        left, right = self.velocity
        if self.stop_when_battery_empty and battery is not None and battery <= 0.0:
            left = right = 0.0
            if not self.battery_empty_warned:
                self.get_logger().warn('bateria em 0 %: motores parados.')
                self.battery_empty_warned = True
        elif battery is not None and battery > 0.0:
            self.battery_empty_warned = False
        self.sim.setJointTargetVelocity(self.leftMotorHandle, self.motor_sign * left)
        self.sim.setJointTargetVelocity(self.rightMotorHandle, self.motor_sign * right)

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
