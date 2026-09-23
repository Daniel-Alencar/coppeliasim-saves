function sysCall_init()
    sim = require('sim')
    --some utilisties
    rad2deg=180/math.pi
    deg2rad=math.pi/180
    MPI=math.pi
    robotHandle=sim.getObjectParent(sim.getObject('.'))
    --scriptName=sim.getScriptName(sim.handle_self)
    

    --some robot's model parameters
    lwheelHandle=sim.getObject('/leftWheel')
    rwheelHandle=sim.getObject('/rightWheel')

    res,zMin=sim.getObjectFloatParameter(lwheelHandle,15)
    res,zMax=sim.getObjectFloatParameter(lwheelHandle,18)
    r=(zMax-zMin)/2-- m (wheel radius)

    lwpos=sim.getObjectPosition(lwheelHandle,-1)
    rwpos=sim.getObjectPosition(rwheelHandle,-1)
    b = math.sqrt((lwpos[1]-rwpos[1])^2+(lwpos[2]-rwpos[2])^2+(lwpos[3]-rwpos[3])^2)/2 -- m (distance between wheels /2 )

    lphidot=0 --init of the left actuator speed
    rphidot=0 --init of the right actuator speed
       
    -- initial position for dead recockning
    roboth=sim.getObject('/myRobot')
    posinit=sim.getObjectPosition(roboth,-1)
    oriinit=sim.getObjectOrientation(roboth,-1)
    x0=posinit[1]
    y0=posinit[2]
    th0=oriinit[3]
    vl=0
    vr=0
    dx=0
    dy=0
    sumdx=0
    sumdy=0
    sumdtheta=0
    xold=0
    yold=0
    oldtime=sim.getSimulationTime()

end

function sysCall_actuation()

    rphidot,rTotalJointPosition=sim.callScriptFunction('getEncoder', sim.getScript(sim.scripttype_simulation,"/rightMotor/encoder" ))
    lphidot,lTotalJointPosition=sim.callScriptFunction('getEncoder', sim.getScript(sim.scripttype_simulation,"/leftMotor/encoder" ))


    rTurnCount=math.floor(rTotalJointPosition/(math.pi*2))
    vr=rphidot*r -- right wheel velocity
   
    lTurnCount=math.floor(lTotalJointPosition/(math.pi*2))
    vl=lphidot*r -- left wheel velocity
   
    dt=sim.getSimulationTime() -oldtime
    xdot=(r/2*lphidot)+(r/2*rphidot)
    ydot=0
    thetadot=(r/(2*b))*rphidot-(r/(2*b))*lphidot
   
    ksi_R= {xdot,ydot,thetadot}--state variable in robot frame
   
   
    --print('xdot',xdot,'ydot',ydot,'thetadot',thetadot)
    --ground thruth
    pos=sim.getObjectPosition(roboth,-1)
    ori=sim.getObjectOrientation(roboth,-1)
   
    -- calculating odometry
    v=(vr+vl)/2  -- robot linear velocity
    omega=thetadot -- robot angular velocity
   
    --theta= (vr-vl)/(2*b)*dt + th0
    dtheta= (vr-vl)/(2*b)*dt
    sumdtheta=sumdtheta+dtheta
    theta=sumdtheta+th0
   
    if theta>=math.pi then theta=theta-(2*math.pi) end
    if theta<=-math.pi then theta=theta+(2*math.pi) end
   
    --print('theta ', theta*180/math.pi,'time', dt)
   
   
    if math.abs(vr-vl)>0.01 then
        diffvrvl=math.abs(vr-vl)
    --    print('vr '..vr..' vl '..vl..' diff '..diffvrvl)
    --    dx= x0 + b*(vr+vl)/(vr-vl)*( math.sin((vr-vl)/(2*b) *dt +th0 )- math.sin(th0) )
    --    dy= y0 - b*(vr+vl)/(vr-vl)*( math.cos((vr-vl)/(2*b) *dt +th0 )- math.cos(th0) )
    --    x= x0 + b*(vr+vl)/(vr-vl)*( math.sin(theta)- math.sin(th0) )
    --    y= y0 - b*(vr+vl)/(vr-vl)*( math.cos(theta)- math.cos(th0) )
    --    dx= b*(vr+vl)/(vr-vl)*( math.sin(theta)- math.sin(th0) )
    --    dy= -b*(vr+vl)/(vr-vl)*( math.cos(theta)- math.cos(th0) )
        dx=  b*(vr+vl)/(vr-vl)*( math.sin(omega *dt +theta )- math.sin(theta) )
        dy= -b*(vr+vl)/(vr-vl)*( math.cos(omega *dt +theta )- math.cos(theta) )
       else
--    print('v='..v..' vr='..vr..' vl='..vl..' theta'..theta*180/math.pi)
        v=(vr+vl)/2
        dx= v*math.cos(theta)*dt
        dy= v*math.sin(theta)*dt
    end
    sumdx=sumdx+dx
    sumdy=sumdy+dy
    x=sumdx+x0
    y=sumdy+y0
   
--[[
    print("dx="..dx.." dy="..dy.." dt="..dt.." v"..v.." vr "..vr.." vl "..vl.." r "..r .." lphidot "..lphidot )
    print('\n\nsumdx='..sumdx..' sumdy='..sumdy)
   
    print(string.format('grd thruth: x:%f y:%f theta:%f \nestimated : x:%f y:%f theta:%f',pos[1],pos[2],ori[3]*180/math.pi,x,y,theta*180/math.pi))
    print(string.format('error : dx:%f dy:%f dtheta:%f totalPos: %f',math.abs(x-pos[1]),math.abs(y-pos[2]),math.abs(theta-ori[3])*180/math.pi,math.sqrt((x-pos[1])^2+(y-pos[2])^2) ))
--]]   

------------ TO REMOVE
--local myData = {1, 2, {"Hello", "world", true, {value1 = 63, value2 = "aString"}}}
--sim.setBufferProperty(robotHandle, "customData.Odometry", sim.packTable(myData))
------------
    if x~=nil and y~=nil and theta~=nil then
        --print("odometry: ",x,y,theta,robotHandle)
        sim.setBufferProperty(robotHandle,"customData.Odometry", sim.packTable({x,y,theta}))
        --sim.setStringProperty(robotHandle,robotHandle.."Odometry",data)
        --sim.setStringSignal(robotHandle.."Odometry",data)
    end
   
    oldtime=sim.getSimulationTime()
        
end

function getOdometry()
    return x,y,theta
end