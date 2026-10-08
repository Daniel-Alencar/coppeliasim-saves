function sysCall_init()
    sim = require('sim')
    simUI = require('simUI')    
    size=2.1 -- area size         
    rndTexture={}
    nbTexture= 15-- number of textures to use in the size*size grids of the alldirty.jpg texture
    for i=1, nbTexture do
        local y=-0.4+ math.floor((i-1)/5)*0.2
        local x=0.4-(i-1)%5*0.2
        rndTexture[i]={x,y}
    end
    
    h={}
    pos={}
    pooppos={}
    dirtycell=20    
    score=0
    poops = 0
    
    poopsensorHandle=sim.getObject('../poopximity')
    
    -- poop related init  
    --poopNumber=2
    poopydrytime1=4  -- 4s= drying time to reach this poopystate (4 is wet 1 is dry)
    poopydrytime2=3
    poopydrytime3=2
    poopydrytime4=1
    
    roboth=sim.getObject('/myRobot')
    
    -- Define small UI
    xml = [[
        <ui title="Score" closeable="true" resizable="false" activate="false" placement="absolute" pos="1450,20" layout="vbox">
            <group layout="hbox" flat="true">
                <label text="Score:" style="* {font-size: 14px; font-weight: bold;}" />
                <label id="10" text="0" style="* {font-size: 18px; color: #006400; font-weight: bold;}" />
            </group>
            <button text="Reset" on-click="resetScore" />
        </ui>
        ]]


    xml = [[
        <ui title="Score" closeable="true" resizable="false" activate="false" placement="absolute" pos="1450,20" layout="vbox">
            <group layout="hbox" flat="true">
                <label text="Score:" style="* {font-size: 14px; font-weight: bold;}" />
                <label id="10" text="0" style="* {font-size: 18px; color: #006400; font-weight: bold;}" />
                <label text="Poops:" style="* {font-size: 14px; font-weight: bold;}" />
                <label id="20" text="0" style="* {font-size: 18px; color: #006400; font-weight: bold;}" />
             </group>
            <button text="Reset" on-click="resetScore" />
        </ui>
        ]]

    
    ui = simUI.create(xml)
    updateScore()
    
--[[    -- Define small POOP UI
    xml2 = [[
        <ui title="Poops" closeable="true" resizable="false" activate="false" placement="absolute" pos="1850,80" layout="vbox">
            <group layout="hbox" flat="true">
                <label text="Poops:" style="* {font-size: 14px; font-weight: bold;}" />
                <label id="10" text="0" style="* {font-size: 18px; color: #006400; font-weight: bold;}" />
            </group>
        </ui>
        ]]
    
    --ui2 = simUI.create(xml2)
    --]]
    updatePoop()    
    
end

function updatePoop()
    simUI.setLabelText(ui, 20, tostring(poops))
end
function incrementPoop()
    poops = poops + 1
    updatePoop()
    --sim.addLog(sim.verbosity_scriptinfos, "Poop updated: " .. poops)
end

function updateScore()
    simUI.setLabelText(ui, 10, tostring(score))
end

function incrementScore()
    score = score + 1
    updateScore()
    --sim.addLog(sim.verbosity_scriptinfos, "Score updated: " .. score)
end

function resetScore(uiHandle, id)
    score = 0
    updateScore()
    --sim.addLog(sim.verbosity_scriptinfos, "Score reset to 0")
end
function sysCall_actuation()
    posrob=sim.getObjectPosition(roboth,-1)

    local res,dist, detectpoints,detectedHandle = sim.readProximitySensor(poopsensorHandle)
    
    if res==1 then
    --print(sim.getObjectAlias(detectedHandle))
        if sim.getObjectAlias(detectedHandle)=='poop' then
            sim.setObjectPosition(detectedHandle,-1,{0,0,1000})
            incrementPoop()
            -- warn to spread poop
            if sim.getSimulationTime()>0.5 then
                sim.callScriptFunction("spreadPoop", sim.getScript(sim.scripttype_simulation, 'room_setup'))
            end    
        end
        if sim.getObjectAlias(detectedHandle)=='Banana' then
            sim.setObjectPosition(detectedHandle,-1,{0,0,1000})
            incrementScore()
        end        
    end
end

function sysCall_cleanup()
    print("final score", score)
    simUI.destroy(ui)
end

-- See the user manual or the available code snippets for additional callback functions and details
