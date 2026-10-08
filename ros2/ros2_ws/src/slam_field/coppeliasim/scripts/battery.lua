function sysCall_init()
    sim = require('sim')

    --battery parameters:
    max_battery_value = 100
    energy_decay_rate = 0--0.005 -- % per dt
    update_frequency  = 1000  -- update timefor the battery activity in ms 
    init_battery_value= 100   -- initial bat. % value
    energy_charge_rate= 1--0.1   -- % per dt 
    robotHandle=sim.getObjectParent(sim.getObject('.')) --sim.getObject("/battery"))
    initTime=sim.getSimulationTime() 
    charging = false
    batt=init_battery_value
end

function sysCall_actuation()
    if sim.getSimulationTime()-initTime >= update_frequency/1000 then
        --check if we are charging
        val=sim.getInt32Signal(robotHandle..'Charging')
        if val~=nil then 
            print("battery: charging "..robotHandle..'Charging')
            sim.clearInt32Signal(robotHandle..'Charging')
            if val==1 then charging=true else charging=false end
        end
        if charging then
            batt=batt+energy_charge_rate
            if batt> max_battery_value then batt=max_battery_value end
print("battery: myRobot is charging...",batt,"%")
        else
            -- using the battery
            batt=batt-energy_decay_rate
            if batt< 0 then batt=0 end 
        end
    
        --sim.addStatusbarMessage('My Battery: '..batt)
        sim.setFloatSignal(robotHandle..'Battery',batt)
        initTime=sim.getSimulationTime() 
    end
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
