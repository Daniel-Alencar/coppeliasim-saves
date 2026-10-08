function sysCall_init()
    sim = require('sim')
    parentHandle=sim.getObjectParent(sim.getObject('.'))
    jointName=sim.getObjectAlias(parentHandle)
    jointHandle=sim.getObject("/"..jointName)
    previousJointPosition=sim.getJointPosition(jointHandle)
    totalJointPosition=previousJointPosition 
    oldTime=sim.getSimulationTime()-0.000001 --trick to have a dt not ==0 at the first loop
end

function sysCall_actuation()
    dt=sim.getSimulationTime()-oldTime
   
    local dth=sim.getJointPosition(jointHandle)-previousJointPosition
    if (dth>=0) then
        dth=math.mod(dth+math.pi,2*math.pi)-math.pi
    else
        dth=math.mod(dth-math.pi,2*math.pi)+math.pi
    end
   
    phidot=dth/dt --actuator velocity in rad/sec
   
    previousJointPosition=sim.getJointPosition(jointHandle)
    totalJointPosition=totalJointPosition+dth

    oldTime=sim.getSimulationTime()
end

function getEncoder()
    return phidot,totalJointPosition
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
