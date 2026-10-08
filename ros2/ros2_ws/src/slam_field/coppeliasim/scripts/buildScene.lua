local NUM_OBJECTS = 20
local AREA_SIZE = 5.0
local MIN_DISTANCE = 0.5

function sysCall_init()

    local scenePath = sim.getStringParam(sim.stringparam_scene_path)

    local poopPath = scenePath .. '/poop.ttm'
    local bananaPath = scenePath .. '/banana.ttm'

    math.randomseed(os.time())

    local poop = {}
    local banana = {}

    --------------------------------------------------
    -- Create poops
    --------------------------------------------------

    for i = 1, NUM_OBJECTS do

        local handle = sim.loadModel(poopPath)

        if handle ~= -1 then

            local x = math.random() * AREA_SIZE - AREA_SIZE / 2
            local y = math.random() * AREA_SIZE - AREA_SIZE / 2

            sim.setObjectPosition(handle, -1, {x, y, 0.015})

            table.insert(poop, x)
            table.insert(poop, y)
        end
    end


    --------------------------------------------------
    -- Create bananas
    --------------------------------------------------

    for i = 1, NUM_OBJECTS do

        local x, y
        local valid = false

        while not valid do

            x = math.random() * AREA_SIZE - AREA_SIZE / 2
            y = math.random() * AREA_SIZE - AREA_SIZE / 2

            valid = true

            -- Check distance from all poops
            for j = 1, #poop, 2 do

                local poopX = poop[j]
                local poopY = poop[j + 1]

                local dx = x - poopX
                local dy = y - poopY

                local distance = math.sqrt(dx * dx + dy * dy)

                if distance < MIN_DISTANCE then
                    valid = false
                    break
                end
            end
        end

        local handle = sim.loadModel(bananaPath)

        if handle ~= -1 then
            sim.setObjectPosition(handle, -1, {x, y, 0.04})
            sim.setObjectOrientation(handle, -1, {90,0,0})

            table.insert(banana, x)
            table.insert(banana, y)
        end
    end


    --------------------------------------------------
    -- Send vectors as signals
    --------------------------------------------------

    sim.setStringSignal('poop', sim.packTable(poop))
    sim.setStringSignal('banana', sim.packTable(banana))


    --------------------------------------------------
    -- Print vectors
    --------------------------------------------------

    local str = "poop=["

    for i = 1, #poop do
        str = str .. string.format("%.3f", poop[i])
        if i < #poop then
            str = str .. ","
        end
    end

    str = str .. "]"
    print(str)


    str = "banana=["

    for i = 1, #banana do
        str = str .. string.format("%.3f", banana[i])
        if i < #banana then
            str = str .. ","
        end
    end

    str = str .. "]"
    print(str)

end