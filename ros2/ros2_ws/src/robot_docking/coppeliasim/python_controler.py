"""Child script (Python) do /myRobot: joystick e ponte para comandos externos.

CÓPIA DE REFERÊNCIA. Este arquivo é o script que já está embutido na cena
"Evaluation scene3.2_students.ttt", pendurado em /myRobot/python_controler.
Editar aqui não muda a cena; serve para ler e entender.

O que ele faz, a cada passo de simulação:

    sliders do joystick ──┐
                          ├──▶ (rightVel, leftVel) ──▶ setJointTargetVelocity
    sinais <h>leftVel  ───┘                            nos dois motores
           <h>rightVel     (override externo, tem prioridade)

E, de quebra, atualiza os rótulos da janela do joystick com a bateria e a
odometria, e marca o checkbox "docking" quando o sinal <h>Docking chega.

<h> é o handle do robô na cena, então os sinais se chamam "84leftVel",
"84Docking" e assim por diante. Ver FLUXO_COMUNICACAO.md.

Atenção para dois detalhes que afetam quem escreve uma ponte ROS 2:

  - Os sinais de override são de USO ÚNICO: sysCall_actuation os lê e os APAGA.
    Quem quiser comandar por eles precisa reescrevê-los a cada passo.
  - Este script escreve nas juntas a CADA passo. Uma ponte externa que também
    chame setJointTargetVelocity disputa o controle com ele.
"""

import math

# Conversões de ângulo. Note que a cena usa 3.14, e não math.pi: as contas de
# max_rotVel herdam esse arredondamento.
deg2rad = 3.14 / 180.
rad2deg = 180. / 3.14


# =============================================================================
#  Inicialização
# =============================================================================

def sysCall_init():
    sim = require('sim')

    # Handles dos objetos da cena. self.* sobrevive entre as chamadas dos
    # sysCall_*; variáveis locais comuns, não.
    self.robotHandle = sim.getObject("/myRobot")
    self.rightMotorHandle = sim.getObject("/rightMotor")
    self.leftMotorHandle = sim.getObject("/leftMotor")
    self.proximitySensorHandle = sim.getObject("/proximitySensor")
    print("python_controller: Keyboard listener started... (press ESC to stop simulation)")

    global ui, simUI, linVel, rotVel, L, max_linVel, max_rotVel, wheelradius

    # Comando atual do joystick, em m/s e rad/s no referencial do robô.
    linVel = 0
    rotVel = 0

    # Parâmetros geométricos, fixos aqui (a cena real mede 0.05 m e 0.2 m).
    wheelradius = 0.05   # m, raio da roda
    L = 0.2              # m, distância entre as duas rodas

    # Limites de fundo de escala dos sliders.
    max_linVel = 0.5             # m/s
    max_rotVel = 90 * deg2rad    # rad/s

    # Load the simUI plugin
    sim = require('sim')
    simUI = require('simUI')

    # Define the UI
    #
    # Cada widget se liga a uma função deste arquivo pelo atributo on-click ou
    # on-change; os ids numéricos são usados depois para atualizar os rótulos:
    #   1 = slider de rotação     3000 = rótulo da velocidade angular
    #   2 = slider linear         4000 = rótulo da velocidade linear
    #  10 = checkbox de docking   4100 = bateria      4200 = odometria
    xml = '''
    <ui title="joystick" closeable="true" resizable="true" activate="true" layout="vbox">
        
        <!-- Top button -->
        <button text="STOP" on-click="stopButtonPressed" />

        <!-- Vertical slider centered -->
        <group layout="hbox" flat="true">

            <label text="   0 m/s  " id="4000" word-wrap="false" />
            <button text="0 Lin Vel" on-click="linVelButtonPressed" />
            <vslider id="2" minimum="-50" maximum="50" value="0" on-change="vslider_changed" />
            <checkbox id="10" text="docking" on-change="dockingButtonPressed" />            
            <stretch />
        </group>

        <!-- Horizontal slider with button at left -->
        <group layout="hbox" flat="true">
            <label text="   0 deg/s  " id="3000" word-wrap="false" />
            <button text="0 rot vel" on-click="rotButtonPressed" />
            <hslider id="1" minimum="-50" maximum="50" value="0" on-change="hslider_changed" />
        </group>
        <group layout="hbox" flat="true">
            <label text="battery: --- % "id="4100" word-wrap="false"   />
            <label text="odometry (x,y,theta): (_,_,_) "id="4200" word-wrap="false"   />
        </group>

    </ui>
    '''
    ui = simUI.create(xml)


# =============================================================================
#  Callbacks da interface do joystick
# =============================================================================

def vslider_changed(ui, id, newVal):
    """Slider vertical: velocidade linear, de -50..50 para -max..+max m/s."""
    global linVel
    linVel = (newVal) / 50 * max_linVel
    #print(f"python_controller: vertical slider value: {newVal}", linVel )
    simUI.setLabelText(ui, 4000, str(round(linVel, 2)) + " m/s ")


def hslider_changed(ui, id, newVal):
    """Slider horizontal: velocidade angular. O sinal é invertido para que
    arrastar para a direita gire o robô para a direita."""
    global rotVel
    rotVel = -(newVal) / 50 * max_rotVel
    #print(f"python_controller: Horizontal slider value: {newVal}", rotVel  )
    simUI.setLabelText(ui, 3000, str(round(rotVel, 1)) + " deg/s ")


def stopButtonPressed(ui, id):
    """Botão STOP: zera os dois eixos e devolve os sliders ao centro."""
    global linVel, rotVel
    linVel = 0
    rotVel = 0
    #print("python_controller: stopping")
    simUI.setSliderValue(ui, 1, 0)
    simUI.setSliderValue(ui, 2, 0)
    simUI.setLabelText(ui, 3000, "0 deg/s ")
    simUI.setLabelText(ui, 4000, "0 m/s ")


def linVelButtonPressed(ui, id):
    """Zera só a velocidade linear."""
    global linVel
    linVel = 0
    #print("python_controller: lin vel is 0")
    simUI.setSliderValue(ui, 2, 0)
    simUI.setLabelText(ui, 4000, "0 m/s ")


def rotButtonPressed(ui, id):
    """Zera só a velocidade angular."""
    global rotVel
    rotVel = 0
    print("python_controller: rot vel is 0")
    simUI.setSliderValue(ui, 1, 0)
    simUI.setLabelText(ui, 3000, "0 deg/s ")


def dockingButtonPressed(ui, id, newVal):
    """Checkbox "docking": anuncia a mudança de modo na cena.

    newVal vem do Qt: 2 = marcado, 0 = desmarcado. O broadcastMsg é recebido
    por sysCall_msg de outros scripts — nesta cena, porém, ninguém o escuta: o
    comportamento de docking é o que o aluno deve implementar.
    """
    val = False
    if newVal == 2:
        val = True
    #print(f"python_controller: docking mode {val}")
    msg = {'id': 'dockingMode', 'data': [val]}
    sim.broadcastMsg(msg)


# =============================================================================
#  Laço de atuação, chamado a cada passo de simulação
# =============================================================================

def sysCall_actuation():

    # inverse kinematic equations
    #
    # Cada roda está a L/2 do centro, então sua velocidade linear é a do robô
    # mais ou menos a contribuição do giro. Resultado em m/s NA RODA.
    rightVel = linVel + L / 2 * rotVel
    leftVel = linVel - L / 2 * rotVel

    # check if for external motor control commands
    #
    # Override externo: quem escrever estes sinais manda, no lugar do joystick.
    # São de uso único — lidos e apagados aqui, precisam ser reescritos a cada
    # passo por quem estiver comandando de fora (por exemplo a ponte ROS 2 em
    # motor_mode=signal).
    rightVelExt = sim.getFloatSignal(str(self.robotHandle) + "rightVel")
    leftVelExt = sim.getFloatSignal(str(self.robotHandle) + "leftVel")
    if leftVelExt is not None:
        sim.clearFloatSignal(str(self.robotHandle) + "leftVel")
        leftVel = leftVelExt
        #print ('overridden left vel')
    if rightVelExt is not None:
        sim.clearFloatSignal(str(self.robotHandle) + "rightVel")
        rightVel = rightVelExt
        #print ('overridden right vel')

    # check and update battery status
    #
    # Quem escreve <h>Battery é o script /myRobot/battery, a cada 1 s.
    batt = sim.getFloatSignal(str(self.robotHandle) + "Battery")
    if batt is not None:
        simUI.setLabelText(ui, 4100, "Battery: " + str(round(batt, 2)) + " % ")
    if batt == 0:
        #battery is dead
        rightVel = 0
        leftVel = 0

    #check and set external docking mode setting
    #
    # Só reage a 1: recebido, apaga o sinal e marca o checkbox, o que dispara
    # dockingButtonPressed. Não há caminho para DESLIGAR o modo por sinal.
    forceDocking = sim.getInt32Signal(str(self.robotHandle) + "Docking")
    if forceDocking == 1:
        sim.clearInt32Signal(str(self.robotHandle) + "Docking")
        simUI.setCheckboxValue(ui, 10, 2, False)

    # check and update odometry
    #
    # O /myRobot/odometry publica (x, y, theta) numa propriedade do objeto, e
    # não num sinal. Abaixo, as tentativas anteriores de transporte que o autor
    # deixou registradas.
    #odomPack=sim.getStringSignal(str(self.robotHandle)+"Odometry")
    #odomPack = sim.getStringProperty(self.robotHandle, "customData.Odometry", {'noError' : True})
    #print("python_controler :",odomPack,self.robotHandle)
    #odomPack = sim.getBufferProperty(sim.handle_app, str(self.robotHandle)+"Odometry", {'noError' : True})
    #if odomPack:
    #    odom = sim.unpackTable(odomPack)
    odomPack = sim.getBufferProperty(self.robotHandle, "customData.Odometry", {'noError': True})

    if odomPack:
        odom = sim.unpackTable(odomPack)
        odomStr = f"({odom[0]:.2f}, {odom[1]:.2f}, {math.degrees(odom[2]):.2f})"
        #print("python_controler: ",odomStr, odom)
        simUI.setLabelText(ui, 4200, f"odometry (x,y,theta): " + odomStr)
        #simUI.setLabelText(ui,4200,"odometry (x,y,theta): (: "+str(round(batt,2))+") ")

    # Repetição do corte por bateria vazia, já feito acima. Inofensivo, mas
    # necessário se o bloco da odometria acima vier a alterar as velocidades.
    if batt == 0:
        #battery is dead
        rightVel = 0
        leftVel = 0

    # update motors veloicity
    #
    # Duas conversões de uma vez: m/s -> rad/s dividindo pelo raio, e o sinal
    # negativo porque, do jeito que as juntas estão montadas nesta cena,
    # velocidade negativa faz o robô andar para a frente.
    sim.setJointTargetVelocity(self.rightMotorHandle, -rightVel / wheelradius)
    sim.setJointTargetVelocity(self.leftMotorHandle, -leftVel / wheelradius)

    # Leitura do sensor de proximidade. O resultado não é usado: os três "nil"
    # são apenas nomes descartáveis (em Python, nil não é palavra reservada).
    state, dist, nil, nil, nil = sim.readProximitySensor(self.proximitySensorHandle)

    # Check if any key was pressed
    #_, keys, _ = sim.getSimulatorMessage()
    #key=chr(keys[0])

    #if key == 'w':
    #    print("Key pressed:", key)
    #pass


def sysCall_sensing():
    # put your sensing code here
    pass


def sysCall_cleanup():
    simUI.destroy(ui)
    pass
