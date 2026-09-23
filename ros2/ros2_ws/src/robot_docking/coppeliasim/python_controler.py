import math
deg2rad=3.14/180.
rad2deg=180./3.14

def sysCall_init():
    sim = require('sim')
    
    self.robotHandle=sim.getObject("/myRobot")
    self.rightMotorHandle=sim.getObject("/rightMotor")
    self.leftMotorHandle=sim.getObject("/leftMotor")
    self.proximitySensorHandle=sim.getObject("/proximitySensor")
    print("python_controller: Keyboard listener started... (press ESC to stop simulation)")
    
    global ui, simUI, linVel, rotVel, L, max_linVel , max_rotVel, wheelradius
    
    linVel=0
    rotVel=0
    wheelradius=0.05
    L=0.2 
    max_linVel = 0.5 # m/s
    max_rotVel = 90 * deg2rad # rad/seg
    # Load the simUI plugin
    sim = require('sim')
    simUI = require('simUI')    
    
    # Define the UI
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

def vslider_changed(ui, id, newVal):
    global linVel
    linVel=(newVal)/50 * max_linVel 
    #print(f"python_controller: vertical slider value: {newVal}", linVel )
    simUI.setLabelText(ui,4000,str(round(linVel,2))+" m/s ")


def hslider_changed(ui, id, newVal):
    global rotVel
    rotVel=-(newVal)/50 * max_rotVel 
    #print(f"python_controller: Horizontal slider value: {newVal}", rotVel  ) 
    simUI.setLabelText(ui,3000,str(round(rotVel,1))+" deg/s ")


def stopButtonPressed(ui, id):
    global linVel, rotVel
    linVel=0
    rotVel=0
    #print("python_controller: stopping")
    simUI.setSliderValue(ui,1,0)
    simUI.setSliderValue(ui,2,0)
    simUI.setLabelText(ui,3000,"0 deg/s ")
    simUI.setLabelText(ui,4000,"0 m/s ")
   
def linVelButtonPressed(ui, id):
    global linVel
    linVel=0
    #print("python_controller: lin vel is 0")
    simUI.setSliderValue(ui,2,0)
    simUI.setLabelText(ui,4000,"0 m/s ")

def rotButtonPressed(ui, id):
    global rotVel
    rotVel=0
    print("python_controller: rot vel is 0")
    simUI.setSliderValue(ui,1,0)
    simUI.setLabelText(ui,3000,"0 deg/s ")

def dockingButtonPressed(ui,id,newVal):
    val = False
    if newVal == 2 :
        val = True
    #print(f"python_controller: docking mode {val}")
    msg = {'id': 'dockingMode', 'data': [val]}
    sim.broadcastMsg(msg) 
    
def sysCall_actuation():

    # inverse kinematic equations
    rightVel = linVel + L/2 * rotVel
    leftVel  = linVel - L/2 * rotVel

    # check if for external motor control commands
    rightVelExt = sim.getFloatSignal(str(self.robotHandle)+"rightVel")
    leftVelExt  = sim.getFloatSignal(str(self.robotHandle)+"leftVel")
    if leftVelExt is not None:
        sim.clearFloatSignal(str(self.robotHandle)+"leftVel")
        leftVel = leftVelExt 
        #print ('overridden left vel')
    if rightVelExt is not None:
        sim.clearFloatSignal(str(self.robotHandle)+"rightVel")
        rightVel = rightVelExt 
        #print ('overridden right vel')

    # check and update battery status
    batt=sim.getFloatSignal(str(self.robotHandle)+"Battery")
    if batt is not None: 
        simUI.setLabelText(ui,4100,"Battery: "+str(round(batt,2))+" % ")
    if batt == 0:
        #battery is dead
        rightVel = 0
        leftVel  = 0
    #check and set external docking mode setting
    forceDocking=sim.getInt32Signal(str(self.robotHandle)+"Docking")
    if forceDocking == 1:
        sim.clearInt32Signal(str(self.robotHandle)+"Docking")
        simUI.setCheckboxValue(ui,10,2,False)
    
    # check and update odometry
    #odomPack=sim.getStringSignal(str(self.robotHandle)+"Odometry")
    #odomPack = sim.getStringProperty(self.robotHandle, "customData.Odometry", {'noError' : True})
    #print("python_controler :",odomPack,self.robotHandle)
    #odomPack = sim.getBufferProperty(sim.handle_app, str(self.robotHandle)+"Odometry", {'noError' : True})
    #if odomPack:
    #    odom = sim.unpackTable(odomPack)   
    odomPack = sim.getBufferProperty(self.robotHandle, "customData.Odometry", {'noError' : True})

    if odomPack: 
        odom = sim.unpackTable(odomPack)
        odomStr = f"({odom[0]:.2f}, {odom[1]:.2f}, {math.degrees(odom[2]):.2f})"        
        #print("python_controler: ",odomStr, odom)
        simUI.setLabelText(ui, 4200, f"odometry (x,y,theta): "+odomStr)
        #simUI.setLabelText(ui,4200,"odometry (x,y,theta): (: "+str(round(batt,2))+") ")
    if batt == 0:
        #battery is dead
        rightVel = 0
        leftVel  = 0    
    
    # update motors veloicity
    sim.setJointTargetVelocity(self.rightMotorHandle,-rightVel/wheelradius)
    sim.setJointTargetVelocity(self.leftMotorHandle, -leftVel /wheelradius)
    state,dist,nil,nil,nil=sim.readProximitySensor(self.proximitySensorHandle)


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



