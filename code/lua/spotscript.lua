function sysCall_init()
    corout=coroutine.create(coroutineMain)
    spot=sim.getObject('/spot')
    Base=sim.getObject('./base')
    legBase=sim.getObject('./legBase')
    knee_joint_FL=sim.getObject('./spot_front_left_knee')
    knee_joint_FR=sim.getObject('./spot_front_right_knee')
    knee_joint_BL=sim.getObject('./spot_rear_left_knee')
    knee_joint_BR=sim.getObject('./spot_rear_right_knee')
    --cube=sim.getObject('/CUBE')
    Rigth_Corner=sim.getObject('../CUBE/Corner_1')
    Left_Corner=sim.getObject('../CUBE/Corner_2')
    Center_Corner=sim.getObject('../CUBE/Center_cube_Floor')
    Target_cor_R=sim.getObject('../CUBE/Corner_4')
    Target_cor_L=sim.getObject('../CUBE/Corner_3')
    Point_To_Be=sim.getObject('../CUBE/Point_to_be_infront')
    
    -- add the points of the cube 
    FR_Leg_Cube   = sim.getObject('../CUBE/FR_Leg_Cube')
    FL_Leg_Cube   = sim.getObject('../CUBE/FL_Leg_Cube')
    BR_Leg_Cube   = sim.getObject('../CUBE/BR_Leg_Cube')
    BL_Leg_Cube   = sim.getObject('../CUBE/BL_Leg_Cube')
    
    -- add the points of the spot
    FR_Leg_Leg   = sim.getObject('./FR_Leg_Leg')
    FL_Leg_Leg   = sim.getObject('./FL_Leg_Leg')
    BR_Leg_Leg   = sim.getObject('./BR_Leg_Leg')
    BL_Leg_Leg   = sim.getObject('./BL_Leg_Leg')
    
    
    print(sim.getObjectPosition(Point_To_Be,-1))
    
    knee_joints = {knee_joint_FL, knee_joint_FR, knee_joint_BL, knee_joint_BR}
    
    simLegTips={}
    simLegTargets={}
    LegsForceSensors={}
    
    simLegTips[1]=sim.getObject('./tip_FL')
    simLegTargets[1]=sim.getObject('./tip_FL_target')
    LegsForceSensors[1]=sim.getObject('./spot_front_left_lower_leg_force_sensor')
    
    simLegTips[2]=sim.getObject('./tip_FR')
    simLegTargets[2]=sim.getObject('./tip_FR_target')
    LegsForceSensors[2]=sim.getObject('./spot_front_right_lower_leg_force_sensor')
    
    simLegTips[3]=sim.getObject('./tip_BL')
    simLegTargets[3]=sim.getObject('./tip_BL_target')
    LegsForceSensors[3]=sim.getObject('./spot_rear_left_lower_leg_force_sensor')
    
    simLegTips[4]=sim.getObject('./tip_BR')
    simLegTargets[4]=sim.getObject('./tip_BR_target')
    LegsForceSensors[4]=sim.getObject('./spot_rear_right_lower_leg_force_sensor')
    
    contact_FL = sim.getObject("./spot_front_left_lower_leg_contact")
    contact_FR = sim.getObject("./spot_front_right_lower_leg_contact")
    contact_BL = sim.getObject("./spot_rear_left_lower_leg_contact")
    contact_BR = sim.getObject("./spot_rear_right_lower_leg_contact")
    
    contact = {contact_FL, contact_FR, contact_BL, contact_BR}
 
    initialPos={}
    for i=1,4,1 do
        initialPos[i]=sim.getObjectPosition(simLegTips[i],legBase)
    end
    
    --IK:
    simBase=sim.getObject('.')
    ikEnv=simIK.createEnvironment()
    -- Prepare the ik group, using the convenience function 'simIK.addElementFromScene':
    ikGroup=simIK.createGroup(ikEnv)
    for i=1,#simLegTips,1 do
        simIK.addElementFromScene(ikEnv,ikGroup,simBase,simLegTips[i],simLegTargets[i],simIK.constraint_position)
    end
end


function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
    linear_vel, angular_vel = sim.getObjectVelocity(Base)
    max_linear_vel = math.max(math.abs(linear_vel[1]), math.abs(linear_vel[2]), math.abs(linear_vel[3]))
    max_angular_vel = math.max(math.abs(angular_vel[1]), math.abs(angular_vel[2]), math.abs(angular_vel[3]))
    max_abs_vel = math.max(max_linear_vel, max_angular_vel)
    spot_base = sim.getObjectPosition(sim.getObject('/spot'), -1)
    if (max_abs_vel > 1) then
        sim.setStringSignal('fall',"true")
    elseif (spot_base[3] < 0.3) then
        sim.setStringSignal('fall',"true")
    end
    simIK.setGroupCalculation(ikEnv,ikGroup,simIK.method_damped_least_squares,1,100)
    
    if (sim.getJointPosition(knee_joint_BR)<0) then
        sim.setJointPosition(knee_joint_BR,-sim.getJointPosition(knee_joint_BR))
    end
    if (sim.getJointPosition(knee_joint_BL)<0) then
        sim.setJointPosition(knee_joint_BL,-sim.getJointPosition(knee_joint_BL))
    end
    
    simIK.applyIkEnvironmentToScene(ikEnv,ikGroup)
end


setStepMode=function(stepVelocity,stepAmplitude,stepHeight,movementDirection,rotationMode,movementStrength)
    movData={}
    movData.vel=stepVelocity
    movData.amplitude=stepAmplitude
    movData.height=stepHeight
    movData.dir=math.pi*movementDirection/180
    movData.rot=rotationMode
    movData.strength=movementStrength
end

function callback(m,vel,accel,auxData)
    sim.setObjectMatrix(auxData[1],auxData[2],m)
end

function moveToPose(obj,relObj,pos,euler,vel,accel)
    local auxData={obj,relObj}
    local mStart=sim.getObjectMatrix(obj,relObj)
    local mGoal=sim.buildMatrix(pos,euler)
    sim.moveToPose(-1,mStart,{vel},{accel},{0.1},mGoal,callback,auxData,{1,1,1,0.1})
end

function drawPath(path_x, path_y, path_z)
    local dr1=sim.addDrawingObject(sim.drawing_lines,1,0,-1,9999,{1,0,1})
    local dr2=sim.addDrawingObject(sim.drawing_spherepts,0.01,0,-1,9999,{1,0,1})
    for i=1,#path_x-1,1 do
        local l={path_x[i],path_y[i],path_z[i],path_x[i+1],path_y[i+1],path_z[i+1]}
        --local l={path[(i-1)*2+1],path[(i-1)*2+2],1,path[i*2+1],path[i*2+2],1}
        sim.addDrawingObjectItem(dr1,l)
    end
    for i=1,#path_x,1 do
        sim.addDrawingObjectItem(dr2,{path_x[i],path_y[i],path_z[i]})
    end
    return dr1,dr2
end

function drawPath_body(path_x, path_y, path_z)
    local dr1=sim.addDrawingObject(sim.drawing_lines,1,0,legBase,9999,{1,0,1})
    local dr2=sim.addDrawingObject(sim.drawing_spherepts,0.01,0,-1,9999,{1,0,1})
    for i=1,#path_x-1,1 do
        local l={path_x[i],path_y[i],path_z[i],path_x[i+1],path_y[i+1],path_z[i+1]}
        --local l={path[(i-1)*2+1],path[(i-1)*2+2],1,path[i*2+1],path[i*2+2],1}
        sim.addDrawingObjectItem(dr1,l)
    end
    for i=1,#path_x,1 do
        sim.addDrawingObjectItem(dr2,{path_x[i],path_y[i],path_z[i]})
    end
    return dr1,dr2
end

function draw_polygon(point1, point2, point3)
    local dr1=sim.addDrawingObject(sim.drawing_lines,1,0,-1,9999,{1,0,1})
    --local dr2=sim.addDrawingObject(sim.drawing_spherepts,0.01,0,-1,9999,{1,0,1})
    handle1 = sim.addDrawingObjectItem(dr1,{point1[1], point1[2], point1[3], point2[1], point2[2], point2[3]})
    handle1 = sim.addDrawingObjectItem(dr1,{point2[1], point2[2], point2[3], point3[1], point3[2], point3[3]})
    handle1 = sim.addDrawingObjectItem(dr1,{point3[1], point3[2], point3[3], point1[1], point1[2], point1[3]})
    return dr1
end

function stablize()
    euler_angles = sim.getObjectOrientation(legBase, -1)
    roll = euler_angles[1]
    pitch = euler_angles[2]
    yaw = euler_angles[3]
    yaw, pitch, roll = yaw, pitch, roll/2
    initialO={roll, pitch, yaw}
    initialP={0,0,0}
    initialP = sim.getObjectPosition(legBase, Base)
    vel = 0.01
    accel = 0.005
    moveToPose(legBase,Base,initialP,initialO,vel,accel)
end

function coroutineMain()
    command = {}
    while true do
        --step(0.1, 0, 0.3, 1)
        --step_Up(0.01, 0.02, 3)
        if #command>0 then
            local lev=sim.setThreadAutomaticSwitch(false)
            local cmd=command[1]
            table.remove(command,1)
            if cmd.cmd=='moveBody' then
                move_body_to_COP(cmd.leg_id, cmd.vel, cmd.accel)
                swing_leg(cmd.leg_id, cmd.dx, cmd.dy, cmd.h, cmd.num_of_points)
                sim.setInt32Signal('execDone',cmd.leg_id) -- signal that motion finished
                sim.wait(0.5)
            elseif cmd.cmd=='beiuzer' then
                move_body_to_COP(cmd.leg_id, cmd.vel, cmd.accel)
                --swing_leg_By_Be(cmd.leg_id, cmd.dx, cmd.dy, cmd.h,cmd.cor_num cmd.num_of_points)
                x_f, y_f, x_i, y_i = swing_leg_By_Be(cmd.x1_Agent, cmd.x2_Agent, cmd.leg_id)
                --sim.setInt32Signal('execDone',cmd.leg_id) -- signal that motion finished
                --sim.wait(0.1)
                dx = (x_f-x_i)
                dy = (y_f-y_i)
                --swing_leg(cmd.leg_id, -dx, -dy, 0.2, 30)
                sim.wait(20)
                for i = 1, 4 do
                    if i == 1 then
                        sim.wait(5)
                        move_body_to_COP(i, cmd.vel, cmd.accel)
                        swing_LegFromPointToPoint(FL_Leg_Leg, FL_Leg_Cube, i, 0.2)
                    elseif i == 2 then
                        sim.wait(5)
                        move_body_to_COP(i, cmd.vel, cmd.accel)
                        swing_LegFromPointToPoint(FR_Leg_Leg, FR_Leg_Cube, i, 0.2)
                    elseif i == 3 then
                        move_body_to_COP(i, cmd.vel, cmd.accel)
                        swing_LegFromPointToPoint(BL_Leg_Leg, BL_Leg_Cube, i, 0.2)
                    else
                        move_body_to_COP(i, cmd.vel, cmd.accel)
                        swing_LegFromPointToPoint(BR_Leg_Leg, BR_Leg_Cube, i, 0.2)
                    end
                    sim.wait(2)
                end
                --[[
                swing_LegFromPointToPoint(cmd.leg_id, -dx/4, -dy/4, 0.2, 30)
                leg_p =sim.getObjectPosition(simLegTips[cmd.leg_id], -1)
                --x_tip, y_tip, z_tip = leg_p[1], leg_p[2], leg_p[3]
                --move_body_to_COP(3, cmd.vel, cmd.accel)
                --swing_leg(3, dx/4, dy/4, 0.2, 30)
                --move_body_to_COP(4, cmd.vel, cmd.accel)
                --swing_leg(4, dx/4, dy/4, 0.2, 30)]]--
                --sim.setInt32Signal('execDone',cmd.leg_id) -- signal that motion finished
                sim.wait(0.2)
            end
            --[[
            sim.wait(1)
            fix_position(FL_Leg_Cube, FL_Leg_Leg, 1)
            sim.wait(1000)
            fix_position(FR_Leg_Cube, FR_Leg_Leg, 2)
            sim.wait(0.5)
            fix_position(BL_Leg_Cube, BL_Leg_Leg, 3)
            sim.wait(0.5)
            fix_position(BR_Leg_Cube, BR_Leg_Leg, 4)
            sim.wait(0.5)]]--
            sim.setThreadAutomaticSwitch(lev)
            sim.setInt32Signal('execDone1',cmd.leg_id)
        else
            sim.switchThread()
        end
    end
end

function fix_position(obj_1, obj_2, leg_id) -- (cube_obj, leg_obj)
    position_1 = sim.getObjectPosition(obj_1, -1)
    print('-------------- Pos 1 ------------------')
    print(position_1[1])
    print(position_1[2])
    --sim.wait()
    position_2 = sim.getObjectPosition(obj_2, -1)
    dx = position_1[1] - position_2[1]
    dy = position_1[2] - position_2[2]
    print('-------------- Pos 2 ------------------')
    print(position_2[1])
    print(position_2[2])
    --sim.wait(50)
    print('---------------------------')
    print('dx = ')
    print(dx)
    print('dy = ')
    print(dy)
    if math.abs(dx) < 0.01 and math.abs(dy) < 0.01 then
        print('need to move the leg')
    else 
        step(dx, dy, 0.08, leg_id)
    end
end


function move_body_to_COP(leg_id, vel, accel)
    --[[
    for i=1,4,1 do
        print(sim.getObject("/Floor"))
        print(sim.getObject("/Cuboid"))
        print(collidingObjects)
        result, force, torqe = sim.readForceSensor(LegsForceSensors[i])
        while (force[3]<15) do
            target_pos = sim.getObjectPosition(simLegTargets[i], -1)
            target_pos = {target_pos[1], target_pos[2], target_pos[3] - 0.005}
            sim.setObjectPosition(simLegTargets[i], -1, target_pos)
            result, force, torqe = sim.readForceSensor(LegsForceSensors[i])
            sim.switchThread()
        end
        result, force, torqe = sim.readForceSensor(LegsForceSensors[i])
    end
    ]]
    for i=1,4,1 do
        collidingObjects,collisionPoint,reactionForce,normalVector = sim.getContactInfo(sim.handle_all, contact[i], 0)
        angle = sim.getJointPosition(knee_joints[i])
        --print("angle = ")
        --print(angle)
        while collidingObjects == nil and angle > 0 do
            target_pos = sim.getObjectPosition(simLegTargets[i], -1)
            target_pos = {target_pos[1], target_pos[2], target_pos[3] - 0.005}
            sim.setObjectPosition(simLegTargets[i], -1, target_pos)
            sim.switchThread()
            angle = sim.getJointPosition(knee_joints[i])
            --print("angle = ")
            --print(angle)
            collidingObjects,collisionPoint,reactionForce,normalVector = sim.getContactInfo(sim.handle_all, contact[i], 0)
        end
    end
    initialP={0,0,0}    
    if leg_id == 1 then
        dr1 = draw_polygon(sim.getObjectPosition(simLegTips[2], -1), sim.getObjectPosition(simLegTips[3], -1), sim.getObjectPosition(simLegTips[4],-1))
        COP = find_center_of_polygon_3_legs(2, 3, 4)
    elseif leg_id == 2 then
        dr1 = draw_polygon(sim.getObjectPosition(simLegTips[1],-1), sim.getObjectPosition(simLegTips[3],-1), sim.getObjectPosition(simLegTips[4],-1))
        COP = find_center_of_polygon_3_legs(1, 3, 4)
    elseif leg_id == 3 then
        dr1 = draw_polygon(sim.getObjectPosition(simLegTips[1],-1), sim.getObjectPosition(simLegTips[2],-1), sim.getObjectPosition(simLegTips[4],-1))
        COP = find_center_of_polygon_3_legs(1, 2, 4)
    elseif leg_id == 4 then
        dr1 = draw_polygon(sim.getObjectPosition(simLegTips[1],-1), sim.getObjectPosition(simLegTips[2],-1), sim.getObjectPosition(simLegTips[3],-1))
        COP = find_center_of_polygon_3_legs(1, 2, 3)
    end
    leg_base_pos = sim.getObjectPosition(legBase,-1)

    initialP[1] = leg_base_pos[1] - COP[1] 
    initialP[2] = leg_base_pos[2] - COP[2]
    initialP[3] = leg_base_pos[3] - COP[3] + 0.02

    initialO={0,0,0}

    euler_angles = sim.getObjectOrientation(legBase, -1)
    --print("euler_angles")
    --print(euler_angles)

    roll = euler_angles[1]
    pitch = euler_angles[2]
    yaw = euler_angles[3]
    
    initialO={roll, pitch, yaw}

    moveToPose(legBase,Base,initialP,initialO,vel,accel)
    --[[
    for i=1,4,1 do
        result, force, torqe = sim.readForceSensor(LegsForceSensors[i])
        while (force[3]<10) do
            target_pos = sim.getObjectPosition(simLegTargets[i], -1)
            target_pos = {target_pos[1], target_pos[2], target_pos[3] - 0.005}
            sim.setObjectPosition(simLegTargets[i], -1, target_pos)
            result, force, torqe = sim.readForceSensor(LegsForceSensors[i])
            sim.switchThread()
        end
        result, force, torqe = sim.readForceSensor(LegsForceSensors[i])
    end
    ]]
    for i=1,4,1 do
        collidingObjects,collisionPoint,reactionForce,normalVector = sim.getContactInfo(sim.handle_all, contact[i], 0)
        angle = sim.getJointPosition(knee_joints[i])
        --print("angle = ")
        --print(angle)
        while collidingObjects == nil and angle > 0 do
            target_pos = sim.getObjectPosition(simLegTargets[i], -1)
            target_pos = {target_pos[1], target_pos[2], target_pos[3] - 0.005}
            sim.setObjectPosition(simLegTargets[i], -1, target_pos)
            sim.switchThread()
            angle = sim.getJointPosition(knee_joints[i])
            --print("angle = ")
            --print(angle)
            collidingObjects,collisionPoint,reactionForce,normalVector = sim.getContactInfo(sim.handle_all, contact[i], 0)
        end
    end
    sim.removeDrawingObject(dr1)
    print("finished moving body")
end

function is_com_inside_polygon(leg_id)
    if leg_id == 1 then
        leg_a = sim.getObjectPosition(simLegTips[2], -1)
        leg_b = sim.getObjectPosition(simLegTips[3], -1)
        leg_c = sim.getObjectPosition(simLegTips[4], -1)
    elseif leg_id == 2 then
        leg_a = sim.getObjectPosition(simLegTips[1], -1)
        leg_b = sim.getObjectPosition(simLegTips[3], -1)
        leg_c = sim.getObjectPosition(simLegTips[4], -1)
    elseif leg_id == 3 then
        leg_a = sim.getObjectPosition(simLegTips[1], -1)
        leg_b = sim.getObjectPosition(simLegTips[2], -1)
        leg_c = sim.getObjectPosition(simLegTips[4], -1)
    elseif leg_id == 4 then
        leg_a = sim.getObjectPosition(simLegTips[1], -1)
        leg_b = sim.getObjectPosition(simLegTips[2], -1)
        leg_c = sim.getObjectPosition(simLegTips[3], -1)
    end
    leg_base_pos = sim.getObjectPosition(Base, -1)
    x1 = leg_a[1]
    y1 = leg_a[2]
    x2 = leg_b[1]
    y2 = leg_b[2]
    x3 = leg_c[1]
    y3 = leg_c[2]
    x = leg_base_pos[1]
    y = leg_base_pos[2]
    area = 0.5 * math.abs(x1*(y2-y3) + x2*(y3-y1) + x3*(y1-y2))

    -- calculate the areas of the triangles formed by the point and each edge
    area1 = 0.5 * math.abs(x*(y1-y2) + x1*(y2-y) + x2*(y-y1))
    area2 = 0.5 * math.abs(x*(y2-y3) + x2*(y3-y) + x3*(y-y2))
    area3 = 0.5 * math.abs(x*(y3-y1) + x3*(y1-y) + x1*(y-y3))

    -- check if the sum of the areas of the triangles is equal to the area of the original triangle
    total_area = area1 + area2 + area3
    if math.abs(total_area - area) < 0.000001 then
        return true
    else
        return false
    end
end

function linspace(x0, dx, n)
  local result = {}
  for i=1,n do
    result[i] = x0 + (i-1)*dx/(n-1)
  end
  return result
end

function calc_parabola_vertex(x1, y1, x2, y2, x3, y3)
    denom = (x1-x2) * (x1-x3) * (x2-x3);
    A     = (x3 * (y2-y1) + x2 * (y1-y3) + x1 * (y3-y2)) / denom;
    B     = (x3*x3 * (y1-y2) + x2*x2 * (y3-y1) + x1*x1 * (y2-y3)) / denom;
    C     = (x2 * x3 * (x2-x3) * y1+x3 * x1 * (x3-x1) * y2+x1 * x2 * (x1-x2) * y3) / denom;
    return A,B,C
end

function bezierCurve(t, start, control1, control2, goal)
    local x = math.pow(1 - t, 3) * start[1] + 3 * math.pow(1 - t, 2) * t * control1[1] + 3 * (1 - t) * math.pow(t, 2) * control2[1] + math.pow(t, 3) * goal[1]
    local y = math.pow(1 - t, 3) * start[2] + 3 * math.pow(1 - t, 2) * t * control1[2] + 3 * (1 - t) * math.pow(t, 2) * control2[2] + math.pow(t, 3) * goal[2]
    local z = math.pow(1 - t, 3) * start[3] + 3 * math.pow(1 - t, 2) * t * control1[3] + 3 * (1 - t) * math.pow(t, 2) * control2[3] + math.pow(t, 3) * goal[3]
    return x, y, z
end
function find_center_of_polygon_3_legs(leg1, leg2, leg3)
    -- Calculates the centroid of a triangle given three points in 3D space.
    -- Args:
    -- - p1, p2, p3: Three points in 3D space, represented as tables {x, y, z}.
    -- Returns:
    -- - A table representing the centroid of the triangle formed by p1, p2, and p3.
    -- Calculate the midpoint of each side of the triangle.
    COP = {0, 0, 0}
--[[
    p1 = sim.getObjectPosition(simLegTips[leg1], legBase)
    p2 = sim.getObjectPosition(simLegTips[leg2], legBase)
    p3 = sim.getObjectPosition(simLegTips[leg3], legBase)
]]
    p1 = sim.getObjectPosition(simLegTips[leg1], -1)
    p2 = sim.getObjectPosition(simLegTips[leg2], -1)
    p3 = sim.getObjectPosition(simLegTips[leg3], -1)

    mp1 = {(p2[1] + p3[1]) / 2, (p2[2] + p3[2]) / 2, (p2[3] + p3[3]) / 2}
    mp2 = {(p1[1] + p3[1]) / 2, (p1[2] + p3[2]) / 2, (p1[3] + p3[3]) / 2}
    mp3 = {(p1[1] + p2[1]) / 2, (p1[2] + p2[2]) / 2, (p1[3] + p2[3]) / 2}
    
    -- Calculate the centroid point.
    x = (mp1[1] + mp2[1] + mp3[1]) / 3
    y = (mp1[2] + mp2[2] + mp3[2]) / 3
    z = (mp1[3] + mp2[3] + mp3[3]) / 3
    
    return {x, y, z}
end
-- Function to linearly interpolate between two values
function linspaceUP(startVal, endVal, n)
    if n == 1 then return {startVal} end
    local res = {}
    local step = (endVal - startVal) / (n - 1)
    for i = 0, n - 1 do
        table.insert(res, startVal + i * step)
    end
    return res
end
function swing_leg_trajectory_FromLegToCube(LegObj, CubeObj, leg_id, H)
    local LegPos = sim.getObjectPosition(simLegTips[leg_id], -1)
--sim.getObjectPosition(LegObj, -1)
    local CubePos = sim.getObjectPosition(CubeObj, -1)
    
    local LegX, LegY, LegZ = LegPos[1], LegPos[2], LegPos[3]
    local CubeX, CubeY, CubeZ = CubePos[1], CubePos[2], CubePos[3]
    
    --------------------------------------------------------------------------
        
        -- Define the start and end points
    local x1, y1, z1 = LegX, LegY, LegZ 
    local x2, y2, z2 = CubeX, CubeY, CubeZ

    -- Define the number of points and the peak height
    local num_points = 15
    local peak_height = H

    -- Calculate the midpoint and parabola coefficient
    local h = (x1 + x2) / 2
    local k = peak_height
    local a = (k - z1) / ((h - x1) ^ 2)
    -- Create x, y, and z values
    local x_values = linspaceUP(x1, x2, num_points)
    local y_values = linspaceUP(y1, y2, num_points)
    local z_values = {}
    for i, x in ipairs(x_values) do
        local z = -1 * a * (x - h) ^ 2 + k
        table.insert(z_values, z)
    end
    -- The trajectories are stored in separate tables
    local trajectory_x = x_values
    local trajectory_y = y_values
    local trajectory_z = z_values

    -- You can now use trajectory_x, trajectory_y, and trajectory_z in your simulation
    --------------------------------------------------------------------------
    return trajectory_x, trajectory_y, trajectory_z
end

function swing_leg_target_trajectory(leg_id, dx, dy, h, num_of_points)
    if leg_id == 1 then
        leg = leg_FL
        hip_joint = hip_joint_FL
        tip = sim.getObjectPosition(simLegTips[leg_id], -1)
    elseif leg_id == 2 then
        leg = leg_FR
        hip_joint = hip_joint_FR
        tip = sim.getObjectPosition(simLegTips[leg_id], -1)
    elseif leg_id == 3 then
        leg = leg_BL
        hip_joint = hip_joint_BL
        tip = sim.getObjectPosition(simLegTips[leg_id], -1)
    else
        leg = leg_BR
        hip_joint = hip_joint_BR
        tip = sim.getObjectPosition(simLegTips[leg_id], -1)
    end
    x0, y0, z0 = tip[1], tip[2], tip[3]
    x1, z1 = x0, z0
    x2, z2 = x0 + dx/2, z0 + h
    x3, z3 = x0 + dx, z0 - 0.005
    a, b, c = calc_parabola_vertex(x1, z1, x2, z2, x3, z3)
    -- positions are in world system
    x_pos_in_world_system = linspace(x0, dx, num_of_points)
    y_pos_in_world_system = linspace(y0, dy, num_of_points)
    z_pos_in_world_system = {}
    
    for i=1, #x_pos_in_world_system, 1 do
        x_val=x_pos_in_world_system[i]
        z=(a*(x_val^2))+(b*x_val)+c
        z_pos_in_world_system[i] = z
    end
    return x_pos_in_world_system, y_pos_in_world_system, z_pos_in_world_system
end

function swing_LegFromPointToPoint(LegObj, CubeObj, leg_id, H)
    x_trajectory, y_trajectory, z_trajectory = swing_leg_trajectory_FromLegToCube(LegObj, CubeObj, leg_id, H)
    drawPath(x_trajectory, y_trajectory, z_trajectory)
    for i=1, #x_trajectory, 1 do
        result, force, torqe = sim.readForceSensor(LegsForceSensors[leg_id])
        if i<=2 then
            sim.setObjectPosition(simLegTargets[leg_id], -1, {x_trajectory[i], y_trajectory[i], z_trajectory[i]})
            sim.wait(0.1)
        elseif force[3]<20 then
            sim.setObjectPosition(simLegTargets[leg_id], -1, {x_trajectory[i], y_trajectory[i], z_trajectory[i]})
            sim.wait(0.1)
        else
            break
        end
    end

    sim.wait(0.7)
end

function swing_leg(leg_id, dx, dy, h, num_of_points)
    x_trajectory, y_trajectory, z_trajectory = swing_leg_target_trajectory(leg_id, dx, dy, h, num_of_points)
    drawPath(x_trajectory, y_trajectory, z_trajectory)
    for i=1, #x_trajectory, 1 do
        result, force, torqe = sim.readForceSensor(LegsForceSensors[leg_id])
        if i<=2 then
            sim.setObjectPosition(simLegTargets[leg_id], -1, {x_trajectory[i], y_trajectory[i], z_trajectory[i]})
            sim.wait(0.05)
        elseif force[3]<10 then
            sim.setObjectPosition(simLegTargets[leg_id], -1, {x_trajectory[i], y_trajectory[i], z_trajectory[i]})
            sim.wait(0.05)
        else
            sim.setInt32Signal('execDone1',leg_id)
            return
        end
    end
    sim.wait(0.7)
end
-- yogev 
function swing_leg_target_trajectory_By_be(x_1Agent, x_2Agent, leg_id)
    --[[if cor_num==1 then
        Point_f = sim.getObjectPosition(Rigth_Corner, -1)
        print("Points_y = ")
        print(Point_f)
    elseif cor_num==2 then
        Point_f = sim.getObjectPosition(Center_Corner, -1)
    elseif cor_num==3 then
        Point_f = sim.getObjectPosition(Left_Corner, -1)
    end]]--
    if leg_id == 1 then
        leg = leg_FL
        hip_joint = hip_joint_FL
        Point_f = sim.getObjectPosition(Left_Corner, -1)
        tip = sim.getObjectPosition(simLegTips[leg_id], -1)
    elseif leg_id == 2 then
        leg = leg_FR
        hip_joint = hip_joint_FR
        Point_f = sim.getObjectPosition(Rigth_Corner, -1)
        tip = sim.getObjectPosition(simLegTips[leg_id], -1)
    elseif leg_id == 3 then
        leg = leg_BL
        hip_joint = hip_joint_BL
        tip = sim.getObjectPosition(simLegTips[leg_id], -1)
    else
        leg = leg_BR
        hip_joint = hip_joint_BR
        tip = sim.getObjectPosition(simLegTips[leg_id], -1)
    end
    --x_Point_F, y_Point_F, z_Point_F = Point_f[1], Point_f[2], Point_f[3]
    x_Point_F = 0
    y_Point_F = 0
    z_Point_F = 0
    x_Point_F, y_Point_F, z_Point_F = Point_f[1], Point_f[2], 0.001--Point_f[3]
    x_tip, y_tip, z_tip = tip[1], tip[2], tip[3]
    diff_x = math.abs(x_Point_F - x_tip)
    diff_y = math.abs(y_Point_F - y_tip)
    Radius = math.sqrt(diff_x * diff_x + diff_y * diff_y)
    sq_1 = 1 - x_1Agent * x_1Agent
    sq_2 = 1 - x_2Agent * x_2Agent
    y_p1 =  math.sqrt(sq_1)
    y_p2 =  math.sqrt(sq_2)
    --control_p1 = {x_1Agent, y_p1, 0.3}
    control_p1 = {x_tip + x_1Agent * Radius, y_tip + y_p1 * Radius, 0.1}
    --control_p2 = {x_2Agent, y_p2, 0.3}
    control_p2 = {x_tip + x_2Agent * Radius, y_tip + y_p2 * Radius, 0.1}
    --x2, z2 = x0 + dx/2, z0 + h
    --x3, z3 = x0 + dx, z0 - 0.005
    x_pos_in_world_system = {}
    y_pos_in_world_system = {}
    z_pos_in_world_system = {}
    num_points = 30
    for i = 0, num_points do
        local t = i / num_points
        local x_traj, y_traj, z_traj = bezierCurve(t, tip, control_p1, control_p2, Point_f)
        --[[print("x Traj =")
        print(x_traj)
        print("y Traj =")
        print(y_traj)
        print("z Traj =")
        print(z_traj)]]--
        table.insert(x_pos_in_world_system, x_traj)
        table.insert(y_pos_in_world_system, y_traj)
        table.insert(z_pos_in_world_system, z_traj)
        --table.insert(curve_points, point)
    end
    --[[print(curve_points)
    x_pos_in_world_system = linspace(x0, dx, num_of_points)
    y_pos_in_world_system = linspace(y0, dy, num_of_points)
    z_pos_in_world_system = {}
    ]]
    --[[for i=1, #x_pos_in_world_system, 1 do
        x_val=x_pos_in_world_system[i]
        z=(a*(x_val^2))+(b*x_val)+c
        z_pos_in_world_system[i] = z
    end]]
    return x_pos_in_world_system, y_pos_in_world_system, z_pos_in_world_system
end

function swing_leg_By_Be(x_1Agent, x_2Agent, leg_id)--(leg_id, dx, dy, h, cor_num, num_of_points)
    x_trajectory, y_trajectory, z_trajectory = swing_leg_target_trajectory_By_be(x_1Agent, x_2Agent, leg_id)--(leg_id, dx, dy, h, cor_num, num_of_points)
    drawPath(x_trajectory, y_trajectory, z_trajectory)
    for i=1, #x_trajectory, 1 do
        result, force, torqe = sim.readForceSensor(LegsForceSensors[leg_id])
        print("####################force - 1################")
        print("force 1 = ")
        print(force[1])
        print("####################force - 2################")
        print("force 2 = ")
        print(force[2])
        print("####################force - 3################")
        print("force 3 = ")
        print(force[3])
        if i<=10 then
            sim.setObjectPosition(simLegTargets[leg_id], -1, {x_trajectory[i], y_trajectory[i], z_trajectory[i]})
            sim.wait(0.1)
        elseif force[3]<10 then
            sim.setObjectPosition(simLegTargets[leg_id], -1, {x_trajectory[i], y_trajectory[i], z_trajectory[i]})
            sim.wait(0.1)
        else
            break
        end
    end
    x_f, y_f, x_i, y_i = x_trajectory[29], y_trajectory[29],x_trajectory[1] ,y_trajectory[1]
    --local initial_Pos = {x = x_trajectory[0], y = y_trajectory[0]}
    --local final_Pos = {x = x_trajectory[-1], y = y_trajectory[-1]}
    --[[print(x_trajectory[1])
    print("first")
    print(x_trajectory[99])
    sim.wait(5)]]--
    --diff_x = -x_trajectory[99] + x_trajectory[1]
    --diff_x = final_Pos.x - initial_Pos.x
    --diff_y = -y_trajectory[99] + y_trajectory[1]
    --diff_y = final_Pos.y - initial_Pos.y
    --step(diff_x, diff_y, 0.1, leg_id)
    --[[sim.wait(3)
    print("REVER")
    local reversedVector_x = {}
    local reversedVector_y = {}
    local reversedVector_z = {}
    for i = #x_trajectory, 1, -1 do
        table.insert(reversedVector_x, x_trajectory[i])
        table.insert(reversedVector_y, y_trajectory[i])
        table.insert(reversedVector_z, z_trajectory[i])
    end
    for i=1, #x_trajectory, 1 do
        result, force, torqe = sim.readForceSensor(LegsForceSensors[leg_id])
        if i<=2 then
            sim.setObjectPosition(simLegTargets[leg_id], -1, {reversedVector_x[i], reversedVector_y[i], reversedVector_z[i]})
            sim.wait(0.1)
        elseif force[3]<10 then
            sim.setObjectPosition(simLegTargets[leg_id], -1, {reversedVector_x[i], reversedVector_y[i], reversedVector_z[i]})
            sim.wait(0.1)
        else
            break
        end
    end]]--
    return x_f, y_f, x_i, y_i
end
-- yogev

function step(dx, dy, h, leg)
    local lev=sim.setThreadAutomaticSwitch(false)
    local cmd={}
    cmd.cmd="moveBody"
    cmd.leg_id=leg
    cmd.vel=0.02
    cmd.accel=0.02
    cmd.dx=dx
    cmd.dy=dy
    cmd.h=h
    cmd.num_of_points=30
    command[#command+1]=cmd
    sim.setThreadAutomaticSwitch(lev)
end
function step_Up(x_1Agent, x_2Agent, leg)--(dx, dy, h, leg)
    local lev=sim.setThreadAutomaticSwitch(false)
    local cmd={}
    cmd.cmd="beiuzer"
    cmd.leg_id=leg
    cmd.vel=0.02
    cmd.accel=0.02
    cmd.x1_Agent=x_1Agent
    print("Agent = ")
    print(cmd.x1_Agent)
    cmd.x2_Agent=x_2Agent
    cmd.h=0.01
    command[#command+1]=cmd
    sim.setThreadAutomaticSwitch(lev)
end
