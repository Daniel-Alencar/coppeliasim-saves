-- Child script (Lua, não threaded) para o /myRobot.
-- Assina as velocidades de roda publicadas pelo motor_publisher_node e aplica
-- nos motores. Requer o plugin simROS2 e o ROS 2 carregado no terminal que
-- abriu o CoppeliaSim (source /opt/ros/jazzy/setup.bash).

sim = require('sim')
simROS2 = require('simROS2')

function sysCall_init()
    leftMotor = sim.getObject('../leftMotor')
    rightMotor = sim.getObject('../rightMotor')

    leftSub = simROS2.createSubscription('/car_control/my_robot/left_motor', 'std_msgs/msg/Float32', 'leftMotorCallback')
    rightSub = simROS2.createSubscription('/car_control/my_robot/right_motor', 'std_msgs/msg/Float32', 'rightMotorCallback')
end

function leftMotorCallback(msg)
    sim.setJointTargetVelocity(leftMotor, msg.data)
end

function rightMotorCallback(msg)
    sim.setJointTargetVelocity(rightMotor, msg.data)
end

function sysCall_cleanup()
    simROS2.shutdownSubscription(leftSub)
    simROS2.shutdownSubscription(rightSub)
    sim.setJointTargetVelocity(leftMotor, 0)
    sim.setJointTargetVelocity(rightMotor, 0)
end
