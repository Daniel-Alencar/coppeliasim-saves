"""Ponte ROS 2 <-> CoppeliaSim para a cena do youBot.

Só comunicação, como as pontes dos exercícios anteriores: ela não planeja, não
controla e não decide nada. O A* está no nó `mission`, a lei de controle da
base em `base_control` e a cinemática do braço em `arm_control` — e nenhum dos
três conhece o CoppeliaSim.

    CoppeliaSim                                 ROS 2 (namespace youbot)
    pose do youBot_ref             --->   pose          geometry_msgs/PoseStamped
                                   --->   /tf           world -> base_link -> arm_tip
    relógio da simulação           --->   sim_time      std_msgs/Float64
    filhos de /bananas             --->   bananas       geometry_msgs/PoseArray
                                   --->   banana_info   std_msgs/String (JSON)
    filhos de /poops               --->   poops         geometry_msgs/PoseArray
    pose e tamanho do /deposit     --->   deposit       std_msgs/String (JSON)
    juntas do braço e da garra     --->   arm_state     sensor_msgs/JointState
                                   --->   gripper_gap   std_msgs/Float64
    quatro rodas suecas            <---   cmd_vel       geometry_msgs/Twist
    cinco juntas do braço          <---   arm_joints    std_msgs/Float64MultiArray
    as duas juntas da garra        <---   gripper       std_msgs/Float64
    parar a simulação              <---   done          std_msgs/Bool

O que muda em relação à ponte do exercício do A*
------------------------------------------------
1. O `cmd_vel` tem TRÊS componentes úteis, e não duas. A base é holonômica:
   `linear.x` é para a frente, `linear.y` é para a esquerda e `angular.z` é o
   giro, e as três são independentes. A ponte do robô diferencial ignorava
   `linear.y` porque aquele robô não tinha como obedecer.
2. Há um braço. Cinco juntas em controle de posição, mais a garra, que tem um
   mecanismo próprio (ver `drive_gripper`).
3. As bananas têm ORIENTAÇÃO, e ela importa: a garra tem de fechar atravessada
   no eixo maior da banana, então o `bananas` publica a pose inteira e não só a
   posição. O tamanho de cada uma vai em `banana_info`, porque é ele que decide
   a abertura da garra — e assim uma cena montada com outra escala de banana
   continua funcionando sem mexer em código.

O eixo da frente do robô
------------------------
A frente do youBot é o eixo +y do `youBot_ref`, não o +x. É a mesma convenção
estranha do robô dos exercícios anteriores, e a conversão é feita aqui: o que
sai em `pose` já é o rumo na convenção do ROS, com yaw = 0 apontando para o +x
do mundo. Quem precisa do outro referencial (a cinemática do braço) usa
`kinematics.ros_to_ref`.

Nunca escreva a orientação do /youBot
-------------------------------------
O frame do objeto `/youBot` nasce girado em (-90°, 0°, -90°). Forçar
`setObjectOrientation` para [0, 0, 0] põe o robô de cabeça para baixo e a
física o expele pelo chão (medido: -27 m em 10 s, com o braço obedecendo
perfeitamente aos comandos enquanto despencava). A ponte só LÊ pose, e lê do
`youBot_ref`, que nasce alinhado ao mundo.
"""

import json
import math
import signal
import threading
import time

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, TransformStamped, Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
import tf2_ros

from youbot_banana import kinematics
from youbot_banana.coppelia_scene import (
    ARM_JOINTS, GRIPPER_JOINTS, WHEEL_JOINTS, bbox, find, find_joints,
    group_children, restore_scripts, script_names, silence_joint_scripts,
    tree_bbox)


def quaternion_from_matrix(m):
    """Quaternion (x, y, z, w) da matriz 3x4 do CoppeliaSim, em ordem de linhas."""
    trace = m[0] + m[5] + m[10]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((m[9] - m[6]) / s, (m[2] - m[8]) / s, (m[4] - m[1]) / s, 0.25 * s)
    if m[0] > m[5] and m[0] > m[10]:
        s = math.sqrt(1.0 + m[0] - m[5] - m[10]) * 2.0
        return (0.25 * s, (m[1] + m[4]) / s, (m[2] + m[8]) / s, (m[9] - m[6]) / s)
    if m[5] > m[10]:
        s = math.sqrt(1.0 + m[5] - m[0] - m[10]) * 2.0
        return ((m[1] + m[4]) / s, 0.25 * s, (m[6] + m[9]) / s, (m[2] - m[8]) / s)
    s = math.sqrt(1.0 + m[10] - m[0] - m[5]) * 2.0
    return ((m[2] + m[8]) / s, (m[6] + m[9]) / s, 0.25 * s, (m[4] - m[1]) / s)


def yaw_quaternion(yaw):
    """(z, w) de uma rotação só em torno do eixo vertical."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class CoppeliaBridge(Node):
    """Nó ROS 2 que liga a cena do youBot a tópicos ROS."""

    def __init__(self):
        super().__init__('coppelia_bridge')

        # --- Conexão e cena ---------------------------------------------------
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 23000)
        self.declare_parameter('robot', '/youBot')
        self.declare_parameter('bananas_path', '/bananas')
        self.declare_parameter('poops_path', '/poops')
        self.declare_parameter('deposit_path', '/deposit')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tip_frame', 'arm_tip')
        self.declare_parameter('autostart', True)
        self.declare_parameter('disable_rival_scripts', True)

        # --- Base -------------------------------------------------------------
        # Ganho de comando: a roda sueca escorrega, e o corpo faz 0,64 do que a
        # cinemática pede (medido em scene/calibrate_base.py). Multiplicar por
        # 1/0,64 faz "0,1 m/s" significar 0,1 m/s, o que deixa os ganhos do
        # controlador de posição querendo dizer o que dizem.
        self.declare_parameter('cmd_gain', 1.0 / kinematics.SLIP_GAIN)
        self.declare_parameter('max_wheel_speed', kinematics.MAX_WHEEL_SPEED)
        self.declare_parameter('cmd_timeout', 0.5)       # [s] sem comando = parar

        # --- Braço ------------------------------------------------------------
        # As juntas do braço vêm do modelo com 6,28 rad/s de velocidade máxima.
        # Nessa velocidade um salto de posição sacode a banana para fora da
        # garra (medido: ela escapava no levantamento). O movimento suave é
        # feito em arm_control, interpolando a trajetória; este teto é a
        # segunda linha de defesa, para um comando mal-dado não virar um golpe.
        self.declare_parameter('arm_max_speed', 1.5)     # [rad/s] por junta
        self.declare_parameter('gripper_force', 20.0)    # [N] nos dedos
        self.declare_parameter('gripper_speed', 0.04)    # [m/s] de fechamento

        # --- Ritmo ------------------------------------------------------------
        # Mesma escolha da ponte do exercício anterior, e pelo mesmo motivo: em
        # curso livre cada chamada da Remote API espera a vez entre dois passos
        # de simulação, e o laço de controle fica à mercê da carga da máquina.
        # Em stepping a ponte é quem dá os passos, e então cada passo usa um
        # comando calculado a partir da pose daquele mesmo passo.
        self.declare_parameter('stepping', True)
        self.declare_parameter('rate', 20.0)             # [Hz] em curso livre
        self.declare_parameter('object_rate', 4.0)        # [Hz] para bananas/poops
        self.declare_parameter('stop_on_done', True)

        self.frame_id = self.get_parameter('frame_id').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tip_frame = self.get_parameter('tip_frame').value

        # --- Conexão ----------------------------------------------------------
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        robot_path = self.get_parameter('robot').value
        self.sim = self.connect(host, port)

        try:
            self.robot = find(self.sim, robot_path, 'o robô')
            self.reference = find(self.sim, f'{robot_path}/youBot_ref',
                                  'o frame de referência da base')
            self.wheels = find_joints(self.sim, self.robot, WHEEL_JOINTS,
                                      'juntas de rolamento')
            self.arm = find_joints(self.sim, self.robot, ARM_JOINTS,
                                   'juntas do braço')
            self.gripper = find_joints(self.sim, self.robot, GRIPPER_JOINTS,
                                       'juntas da garra')
            # As pastilhas visuais dos dedos dão o ponto de preensão; são elas
            # que `kinematics.arm_fk` prevê.
            self.pads = [find(self.sim, f'{robot_path}/Rectangle2'),
                         find(self.sim, f'{robot_path}/Rectangle3')]
        except LookupError as error:
            raise SystemExit(str(error))

        self.silenced = []
        if self.get_parameter('disable_rival_scripts').value:
            self.silenced = silence_joint_scripts(self.sim, self.robot)
            if self.silenced:
                self.get_logger().info(
                    'scripts da cena calados enquanto a ponte roda (eles '
                    'comandam as mesmas juntas a cada passo): '
                    + ', '.join(script_names(self.sim, self.silenced)))

        self.prepare_joints()

        # --- Objetos da cena ---------------------------------------------------
        self.bananas = group_children(self.sim,
                                      self.get_parameter('bananas_path').value)
        self.poops = group_children(self.sim,
                                    self.get_parameter('poops_path').value)
        # Tamanho de cada banana, medido na cena uma única vez: é o que decide
        # a abertura da garra.
        for banana in self.bananas:
            banana['size'] = bbox(self.sim, banana['handle'])
        # Os poops não se mexem: as posições são lidas uma vez só.
        self.poop_points = [self.position(poop['handle'])[:2]
                            for poop in self.poops]
        self.deposit = self.read_deposit()

        self.get_logger().info(
            f'{len(self.bananas)} bananas e {len(self.poops)} poops na cena'
            + (f'; depósito em ({self.deposit["x"]:+.2f}, '
                f'{self.deposit["y"]:+.2f}), borda a '
                f'{self.deposit["rim"]:.3f} m'
                if self.deposit else '; SEM DEPÓSITO'))
        if not self.bananas:
            self.get_logger().warn(
                'nenhuma banana: confira o parâmetro bananas_path, ou monte a '
                'cena com scene/build_scene.py')

        # --- Estado da simulação ------------------------------------------------
        self.started_here = (
            self.get_parameter('autostart').value
            and self.sim.getSimulationState() == self.sim.simulation_stopped)
        if self.started_here:
            self.sim.startSimulation()
        elif self.sim.getSimulationState() == self.sim.simulation_paused:
            self.get_logger().warn('a simulação está pausada: nada muda até o play.')

        # --- Estado interno -----------------------------------------------------
        self.twist = (0.0, 0.0, 0.0)      # (frente, esquerda, giro) pedidos
        self.deadline = 0.0               # quando o último cmd_vel expira
        self.arm_target = None            # 5 posições de junta, ou None
        self.gripper_target = 1.0         # 1 = aberta, 0 = fechando
        self.done = threading.Event()

        # --- Comunicação ROS -----------------------------------------------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose', 10)
        self.sim_time_pub = self.create_publisher(Float64, 'sim_time', 10)
        self.banana_pub = self.create_publisher(PoseArray, 'bananas', latched)
        self.banana_info_pub = self.create_publisher(String, 'banana_info', latched)
        self.poop_pub = self.create_publisher(PoseArray, 'poops', latched)
        self.deposit_pub = self.create_publisher(String, 'deposit', latched)
        self.arm_state_pub = self.create_publisher(JointState, 'arm_state', 10)
        self.gap_pub = self.create_publisher(Float64, 'gripper_gap', 10)

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float64MultiArray, 'arm_joints',
                                 self.arm_callback, 10)
        self.create_subscription(Float64, 'gripper', self.gripper_callback, 10)
        self.create_subscription(Bool, 'done', self.done_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # --- Ritmo --------------------------------------------------------------
        self.step_time = self.sim.getFloatParam(
            self.sim.floatparam_simulation_time_step)
        self.sim_time = self.sim.getSimulationTime()
        self.stepping = self.get_parameter('stepping').value
        self.cycles = 0
        if self.stepping:
            self.object_period = max(1, round(
                1.0 / (self.step_time * self.get_parameter('object_rate').value)))
            self.client.setStepping(True)
            self.get_logger().info(
                f'modo stepping: um ciclo por passo de '
                f'{self.step_time * 1000:.0f} ms')
        else:
            self.object_period = 1
            self.create_timer(1.0 / self.get_parameter('rate').value, self.publish_state)
            self.create_timer(1.0 / self.get_parameter('rate').value, self.actuate)
            self.create_timer(1.0 / self.get_parameter('object_rate').value,
                              self.publish_objects)

        self.publish_objects()
        self.get_logger().info(
            f'ponte pronta. comandos em {self.resolve_topic_name("cmd_vel")}, '
            f'{self.resolve_topic_name("arm_joints")} e '
            f'{self.resolve_topic_name("gripper")}')

    # =========================================================================
    #  Conexão e preparação
    # =========================================================================

    def connect(self, host, port):
        """Conecta ao CoppeliaSim, explicando o motivo quando não dá."""
        try:
            self.client = RemoteAPIClient(host, port)
            return self.client.require('sim')
        except Exception:
            raise SystemExit(
                f'Não consegui falar com o CoppeliaSim em {host}:{port}.\n'
                '  - O simulador está aberto com a cena youbot_banana.ttt?\n'
                '  - Monte a cena com: python3 scene/build_scene.py\n'
                '  - Um script da cena preso em laço trava a thread principal:\n'
                '    a porta continua aberta, mas nenhuma chamada é respondida.')

    def prepare_joints(self):
        """Põe as juntas no modo e no limite que os nós ROS esperam.

        - As do braço em controle de POSIÇÃO, com a velocidade limitada.
        - A garra no mecanismo do próprio fabricante: a junta 2 em controle de
          VELOCIDADE (ela trava contra o objeto) e a junta 1 seguindo em
          posição. Ver `drive_gripper` para o porquê.
        """
        max_speed = self.get_parameter('arm_max_speed').value
        for joint in self.arm:
            self.sim.setObjectInt32Param(joint, self.sim.jointintparam_dynctrlmode,
                                         self.sim.jointdynctrl_position)
            self.sim.setObjectFloatParam(joint, self.sim.jointfloatparam_maxvel,
                                         max_speed)
        force = self.get_parameter('gripper_force').value
        self.sim.setObjectInt32Param(self.gripper[1],
                                     self.sim.jointintparam_dynctrlmode,
                                     self.sim.jointdynctrl_velocity)
        self.sim.setObjectInt32Param(self.gripper[0],
                                     self.sim.jointintparam_dynctrlmode,
                                     self.sim.jointdynctrl_position)
        for joint in self.gripper:
            self.sim.setJointTargetForce(joint, force)

    def read_deposit(self):
        """Centro, tamanho e altura da borda do depósito, ou None se não houver.

        O tamanho é da ÁRVORE inteira (`tree_bbox`), e não do shape da raiz: a
        bandeja é feita de cinco peças, e perguntar à primeira daria a largura
        de uma parede. É deste número que a missão tira a que distância parar.
        """
        handle = self.sim.getObject(self.get_parameter('deposit_path').value,
                                    {'noError': True})
        if handle == -1:
            return None
        width, depth, height, x, y, top = tree_bbox(self.sim, handle)
        return {'x': x, 'y': y, 'width': width, 'depth': depth,
                'height': height, 'rim': top}

    def position(self, handle):
        return self.sim.getObjectPosition(handle, self.sim.handle_world)

    # =========================================================================
    #  Um ciclo
    # =========================================================================

    def cycle(self):
        """Um passo de simulação, com o ciclo de controle inteiro dentro dele.

        A ordem importa, e é a mesma da ponte do exercício anterior: o estado
        sai primeiro, o nó dá tempo aos controladores de responderem àquele
        estado, e só então os comandos vão para as juntas e o passo acontece.
        """
        self.publish_state()
        rclpy.spin_once(self, timeout_sec=0.02)   # espera a resposta a este estado
        for _ in range(6):                        # e esvazia o que mais chegou
            rclpy.spin_once(self, timeout_sec=0.0)
        self.actuate()
        self.cycles += 1
        if self.cycles % self.object_period == 0:
            self.publish_objects()
        self.sim.step()
        # Em stepping o relógio é sabido sem gastar chamada.
        self.sim_time += self.step_time
        self.sim_time_pub.publish(Float64(data=self.sim_time))

    # =========================================================================
    #  CoppeliaSim -> ROS
    # =========================================================================

    def read_pose(self):
        """(x, y, yaw) da base no mundo, já na convenção do ROS.

        `getObjectMatrix` devolve a matriz 3x4 em ordem de linhas. A frente do
        robô é o eixo +y dele, que é a segunda coluna: (m[1], m[5]). O yaw
        publicado é a direção desse eixo no mundo, de modo que no tópico `pose`
        a frente do robô é o +x — a convenção do ROS e a que os controladores
        esperam.
        """
        m = self.sim.getObjectMatrix(self.reference, self.sim.handle_world)
        return m[3], m[7], math.atan2(m[5], m[1])

    def publish_state(self):
        """Pose da base, TF, estado das juntas e abertura da garra."""
        x, y, yaw = self.read_pose()
        stamp = self.get_clock().now().to_msg()
        z, w = yaw_quaternion(yaw)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        self.pose_pub.publish(pose)

        base_tf = TransformStamped()
        base_tf.header.stamp = stamp
        base_tf.header.frame_id = self.frame_id
        base_tf.child_frame_id = self.base_frame
        base_tf.transform.translation.x = x
        base_tf.transform.translation.y = y
        base_tf.transform.rotation.z = z
        base_tf.transform.rotation.w = w

        # A ponta da garra, no frame da base e já na convenção do ROS. É a
        # medida com que se confere a cinemática rodando: se arm_fk e esta TF
        # discordarem, o modelo não corresponde à cena.
        pads = [self.sim.getObjectPosition(pad, self.reference) for pad in self.pads]
        tip = kinematics.ref_to_ros(
            tuple((pads[0][i] + pads[1][i]) / 2.0 for i in range(3)))
        tip_tf = TransformStamped()
        tip_tf.header.stamp = stamp
        tip_tf.header.frame_id = self.base_frame
        tip_tf.child_frame_id = self.tip_frame
        tip_tf.transform.translation.x = tip[0]
        tip_tf.transform.translation.y = tip[1]
        tip_tf.transform.translation.z = tip[2]
        tip_tf.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform([base_tf, tip_tf])

        state = JointState()
        state.header.stamp = stamp
        state.name = list(ARM_JOINTS) + list(GRIPPER_JOINTS)
        state.position = [self.sim.getJointPosition(joint)
                          for joint in self.arm + self.gripper]
        self.arm_state_pub.publish(state)

        gap = kinematics.gripper_gap(state.position[5], state.position[6])
        self.gap_pub.publish(Float64(data=gap))

        if not self.stepping:
            self.sim_time = self.sim.getSimulationTime()
            self.sim_time_pub.publish(Float64(data=self.sim_time))

    def publish_objects(self):
        """Bananas (com orientação e tamanho), poops e cesto."""
        poses = PoseArray()
        poses.header.stamp = self.get_clock().now().to_msg()
        poses.header.frame_id = self.frame_id
        info = []
        for banana in self.bananas:
            matrix = self.sim.getObjectMatrix(banana['handle'],
                                              self.sim.handle_world)
            quaternion = quaternion_from_matrix(matrix)
            pose = Pose()
            pose.position.x = matrix[3]
            pose.position.y = matrix[7]
            pose.position.z = matrix[11]
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = quaternion
            poses.poses.append(pose)
            # O yaw do eixo MAIOR da banana: é o que a garra precisa para
            # fechar atravessada nela. O eixo maior do modelo é o x local, e a
            # primeira coluna da matriz é esse eixo visto do mundo.
            info.append({
                'path': banana['path'],
                'name': banana['name'],
                'yaw': math.atan2(matrix[4], matrix[0]),
                'length': banana['size'][0],
                'width': banana['size'][1],
                'height': banana['size'][2],
            })
        self.banana_pub.publish(poses)
        self.banana_info_pub.publish(String(data=json.dumps(info)))

        poops = PoseArray()
        poops.header = poses.header
        for x, y in self.poop_points:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.orientation.w = 1.0
            poops.poses.append(pose)
        self.poop_pub.publish(poops)

        if self.deposit:
            self.deposit_pub.publish(String(data=json.dumps(self.deposit)))

    # =========================================================================
    #  ROS -> CoppeliaSim
    # =========================================================================

    def cmd_vel_callback(self, msg: Twist):
        """Guarda o twist pedido. As três componentes valem: a base é holonômica."""
        self.twist = (msg.linear.x, msg.linear.y, msg.angular.z)
        self.deadline = time.monotonic() + self.get_parameter('cmd_timeout').value

    def arm_callback(self, msg: Float64MultiArray):
        """Cinco posições de junta do braço, em radianos."""
        if len(msg.data) != len(self.arm):
            self.get_logger().warn(
                f'arm_joints com {len(msg.data)} valores; esperava {len(self.arm)}')
            return
        self.arm_target = list(msg.data)

    def gripper_callback(self, msg: Float64):
        """Abertura pedida: 1 = aberta, 0 = fechando até travar no objeto."""
        self.gripper_target = max(0.0, min(1.0, msg.data))

    def done_callback(self, msg: Bool):
        if msg.data and not self.done.is_set():
            self.get_logger().info('missão terminou: encerrando a ponte')
            self.done.set()

    def actuate(self):
        """Escreve nas juntas o último comando recebido de cada nó."""
        self.drive_wheels()
        self.drive_arm()
        self.drive_gripper()

    def drive_wheels(self):
        """Twist do corpo -> velocidade das quatro rodas suecas."""
        # Sem comando recente o robô para sozinho, mesmo que o controlador caia.
        if time.monotonic() >= self.deadline:
            self.twist = (0.0, 0.0, 0.0)
        gain = self.get_parameter('cmd_gain').value
        speeds = kinematics.wheel_velocities(self.twist[0] * gain,
                                             self.twist[1] * gain,
                                             self.twist[2] * gain)
        speeds = kinematics.scale_to_limit(
            speeds, self.get_parameter('max_wheel_speed').value)
        for joint, speed in zip(self.wheels, speeds):
            self.sim.setJointTargetVelocity(joint, speed)

    def drive_arm(self):
        """Posições alvo das cinco juntas do braço."""
        if self.arm_target is None:
            return
        for joint, target in zip(self.arm, self.arm_target):
            self.sim.setJointTargetPosition(joint, target)

    def drive_gripper(self):
        """A garra, no mecanismo do próprio modelo do youBot.

        O script original da cena (`.../Rectangle7/Script`) faz assim:

            junta 2  em VELOCIDADE, +-0,04 m/s
            junta 1  em POSIÇÃO, seguindo -0,5 * posição da junta 2

        e é o jeito certo. Fechar as duas por POSIÇÃO até o fim esmaga o objeto
        entre os dedos — a garra fecha até 26 mm sobre uma banana de 50 mm — e o
        contato acumula força até expelir a banana pela cena (medido: ela voava
        30 cm). Em velocidade, o motor simplesmente TRAVA contra o objeto e
        segura com a força limitada: os dedos param em 69 mm, em contato
        estável, e o objeto fica preso.

        A `junta 1` seguir a `junta 2` é o acoplamento mecânico dos dois dedos,
        que no modelo tem razão -0,5.
        """
        speed = self.get_parameter('gripper_speed').value
        # Fechando quando a abertura pedida é menor que a atual, abrindo quando
        # é maior. O alvo 0 vira "feche até travar", que é o que a preensão quer.
        opening = kinematics.gripper_opening(
            self.sim.getJointPosition(self.gripper[1]))
        direction = 1.0 if self.gripper_target < opening - 0.02 else (
            -1.0 if self.gripper_target > opening + 0.02 else 0.0)
        self.sim.setJointTargetVelocity(self.gripper[1], direction * speed)
        self.sim.setJointTargetPosition(
            self.gripper[0], self.sim.getJointPosition(self.gripper[1]) * -0.5)

    # =========================================================================
    #  Saída
    # =========================================================================

    def stop(self):
        """Para tudo, devolve os scripts da cena e fecha a simulação."""
        try:
            for joint in self.wheels:
                self.sim.setJointTargetVelocity(joint, 0.0)
            self.sim.setJointTargetVelocity(self.gripper[1], 0.0)
            if self.stepping:
                # Sem isto a simulação fica esperando um passo que ninguém mais
                # vai pedir, e o simulador parece travado.
                self.sim.step()
                self.client.setStepping(False)
            restore_scripts(self.sim, self.silenced)
            if self.started_here or (self.done.is_set()
                                     and self.get_parameter('stop_on_done').value):
                self.sim.stopSimulation()
        except Exception as error:
            self.get_logger().warn(f'na saída: {error}')


def main(args=None):
    # O Ctrl+C só levanta uma flag: deixar o rclpy cortar o laço no meio de uma
    # chamada ZeroMQ invalida o socket, e aí o comando de parada nunca chega ao
    # simulador e o robô continua andando.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    stop_requested = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_requested.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_requested.set())

    try:
        node = CoppeliaBridge()
    except SystemExit as error:
        print(error)
        rclpy.shutdown()
        return 1

    try:
        while not stop_requested.is_set() and not node.done.is_set():
            if node.stepping:
                node.cycle()
            else:
                rclpy.spin_once(node, timeout_sec=0.1)
        if node.done.is_set():
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
