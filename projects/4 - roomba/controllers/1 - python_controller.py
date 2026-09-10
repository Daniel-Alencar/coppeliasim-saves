deg2rad=3.14/180.
rad2deg=180./3.14

def sysCall_init():
    sim = require('sim')

    self.rightMotorHandle=sim.getObject("/rightMotor")
    self.leftMotorHandle=sim.getObject("/leftMotor")
    self.proximitySensorHandle=sim.getObject("/proximitySensor")

    global linVel, rotVel, L, max_linVel, max_rotVel, wheelradius, forwardVel

    wheelradius=0.05
    L=0.2
    max_linVel = 0.5 # m/s
    max_rotVel = 90 * deg2rad # rad/seg

    # comando continuo: segue sempre em frente, sem rotacao
    forwardVel = max_linVel
    linVel = forwardVel
    rotVel = 0

    print(f"Andando para a frente continuamente a {forwardVel} m/s")

def sysCall_actuation():
    dist = 0
    global linVel, rotVel

    # inverse kinematic equations
    rightVel = linVel + L/2 * rotVel
    leftVel  = linVel - L/2 * rotVel

    sim.setJointTargetVelocity(self.rightMotorHandle,-rightVel/wheelradius)
    sim.setJointTargetVelocity(self.leftMotorHandle, -leftVel /wheelradius)

    state,dist,nil,nil,nil=sim.readProximitySensor(self.proximitySensorHandle)

    if dist > 0 and dist <= 0.7:
        print(f"Obstáculo detectado a {dist} m, parando o robô")
        linVel = 0
        rotVel = 0.5
    else:
        linVel = forwardVel
        rotVel = 0

def sysCall_sensing():
    # put your sensing code here
    pass

def sysCall_cleanup():
    pass
