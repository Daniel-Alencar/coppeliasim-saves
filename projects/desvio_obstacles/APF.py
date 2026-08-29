import numpy as np

deg2rad=3.14/180.
rad2deg=180./3.14

def compute_forces(q = (0, 0), q_goal = (1, 1), q_obstacles = []):
    # Attractive force constant
    k_attractive = 1.0
    f_attractive = k_attractive * (q_goal - q)

    k_repulsive = 1.0
    # Safe distance from obstacles
    d_safe = 0.5
    f_repulsive = []
    for q_obs in q_obstacles:
        f_repulsive.append(k_repulsive * (1/np.linalg.norm(q - q_obs) - 1/d_safe) * (q - q_obs) / np.linalg.norm(q - q_obs))

    f = f_attractive + np.sum(f_repulsive, axis=0)
    return f

def sysCall_init():
    sim = require('sim')

    self.rightMotorHandle=sim.getObject("/rightMotor")
    self.leftMotorHandle=sim.getObject("/leftMotor")
    self.proximitySensorHandle=sim.getObject("/proximitySensor")
    print("Keyboard listener started... (press ESC to stop simulation)")
    
    global ui, simUI, linVel, rotVel, L, max_linVel , max_rotVel, wheelradius

    # Define the initial values for the linear and rotational velocities, wheel radius, and maximum velocities
    linVel=0
    rotVel=0
    wheelradius=0.05
    L=0.2  
    max_linVel = 0.5
    max_rotVel = 90 * deg2rad
    # Load the simUI plugin
    sim = require('sim')
    simUI = require('simUI')    
    
    # Define the UI
    xml = '''
    <ui title="Slider Example" closeable="true" resizable="true" activate="false" layout="vbox">
        
        <!-- Top button -->
        <button text="0 Lin Vel" on-click="stopButtonPressed" />

        <!-- Vertical slider centered -->
        <group layout="hbox" flat="true">
            <stretch />
            <vslider id="2" minimum="-50" maximum="50" value="0" on-change="vslider_changed" />
            <stretch />
        </group>

        <!-- Horizontal slider with button at left -->
        <group layout="hbox" flat="true">
            <button text="0 rot vel" on-click="rotButtonPressed" />
            <hslider id="1" minimum="-50" maximum="50" value="0" on-change="hslider_changed" />
        </group>

    </ui>
    '''
    ui = simUI.create(xml)

def vslider_changed(ui, id, newVal):
    global linVel
    linVel=(newVal)/50 * max_linVel 
    print(f"vertical slider value: {newVal}", linVel )

def hslider_changed(ui, id, newVal):
    global rotVel
    rotVel=(newVal)/50 * max_rotVel 
    print(f"Horizontal slider value: {newVal}", rotVel  )

def stopButtonPressed(ui, id):
    print("stopping")

def rotButtonPressed(ui, id):
    print("rot vel is 0")
    
def sysCall_actuation():
    # L/2 é o raio do robô, então a velocidade de cada roda é calculada com base na velocidade linear e angular desejada.
    rightVel = linVel + L/2 * rotVel
    leftVel  = linVel - L/2 * rotVel

    print (rightVel,leftVel)
    sim.setJointTargetVelocity(self.rightMotorHandle,-rightVel/wheelradius)
    sim.setJointTargetVelocity(self.leftMotorHandle, -leftVel /wheelradius)
    state,dist,nil,nil,nil=sim.readProximitySensor(self.proximitySensorHandle)
    # print(state,dist)
    # Check if any key was pressed
    _, keys, _ = sim.getSimulatorMessage()
    key=chr(keys[0])

    if key == 'w':
        print("Key pressed:", key)
    pass

def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    simUI.destroy(ui)
    pass



