import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

physicsClient = p.connect(p.GUI)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

# control parameters
x_vel_coeff = 0.5 # x velocity control coefficient
y_vel_coeff = 0.25 # y velocity control coefficient
sens_dist = 5 # obstacle sensing distance
sens_width = 0 # obstacle sensing width
obs_coeff = 20 # obstacle avoidance coeffiecient
limb_coeff = 15 # limb-spacing mantaining coefficient
comm_delay = 0.02 # round-trip communication delay, in sec

# velocity control function
def vel_ctrl(body, vel):
    lin_vel,ang_vel = p.getBaseVelocity(body)
    force1 = [x_vel_coeff*(vel[0]-lin_vel[0]) - y_vel_coeff*(vel[1]-lin_vel[1]), 0, 0]
    force2 = [x_vel_coeff*(vel[0]-lin_vel[0]) + y_vel_coeff*(vel[1]-lin_vel[1]), 0, 0]
    if (body == mod1 or body == mod2):
        p.applyExternalForce(body,-1,force1,[0,1/4,0],p.LINK_FRAME)
        p.applyExternalForce(body,-1,force2,[0,-1/4,0],p.LINK_FRAME)
    elif (body == mod3 or body == mod4):
        p.applyExternalForce(body,-1,force1,[0,-1/4,0],p.LINK_FRAME)
        p.applyExternalForce(body,-1,force2,[0,1/4,0],p.LINK_FRAME)

# decentralized velocity control function
def ctrl_dec(body, vel):
    obstacle_pos,obstacle_ort = p.getBasePositionAndOrientation(obs)
    pos,ort = p.getBasePositionAndOrientation(body)
    y_vel = vel[1]
    if (body == mod1):
        # avoid objects
        obstacle_dist_x = obstacle_pos[0]-(pos[0]+0.5)
        obstacle_dist_y = obstacle_pos[1]-pos[1]
        if (obstacle_dist_y-0.25<arm_width/2+sens_width/2 and obstacle_dist_y>(arm_width/2-body_width/2)):
            if (obstacle_dist_x<sens_dist and obstacle_dist_x>0):
                y_vel -= obs_coeff/obstacle_dist_x
        else:
            # maintain limb spacing
            arm_pos,arm_ort = p.getBasePositionAndOrientation(mod2)
            arm_dist = arm_pos[1] - pos[1] + (body_width-arm_width)
            y_vel += limb_coeff*arm_dist
    elif (body == mod2):
        # avoid objects
        obstacle_dist_x = obstacle_pos[0]-(pos[0]+0.5)
        obstacle_dist_y = obstacle_pos[1]-pos[1]
        if (obstacle_dist_y+0.25>-arm_width/2-sens_width/2 and obstacle_dist_y<(body_width/2-arm_width/2)):
            if (obstacle_dist_x<sens_dist and obstacle_dist_x>0):
                y_vel += obs_coeff/obstacle_dist_x
        else:
            # maintain limb spacing
            arm_pos,arm_ort = p.getBasePositionAndOrientation(mod1)
            arm_dist = arm_pos[1] - pos[1] - (body_width-arm_width)
            y_vel += limb_coeff*arm_dist
    elif (body == mod3):
        # avoid objects
        obstacle_dist_x = obstacle_pos[0]-(pos[0]+0.5)
        obstacle_dist_y = obstacle_pos[1]-pos[1]
        if (obstacle_dist_y-0.25<arm_width/2+sens_width/2 and obstacle_dist_y>(arm_width/2-body_width/2)):
            if (obstacle_dist_x<sens_dist and obstacle_dist_x>0):
                y_vel -= obs_coeff/obstacle_dist_x
        else:
            # maintain limb spacing
            arm_pos,arm_ort = p.getBasePositionAndOrientation(mod4)
            arm_dist = arm_pos[1] - pos[1] + (body_width-arm_width)
            y_vel += limb_coeff*arm_dist
        y_vel = vel[1]
    elif (body == mod4):
        # avoid objects
        obstacle_dist_x = obstacle_pos[0]-(pos[0]+0.5)
        obstacle_dist_y = obstacle_pos[1]-pos[1]
        if (obstacle_dist_y+0.25>-arm_width/2-sens_width/2 and obstacle_dist_y<(body_width/2-arm_width/2)):
            if (obstacle_dist_x<sens_dist and obstacle_dist_x>0):
                y_vel += obs_coeff/obstacle_dist_x
        else:
            # maintain limb spacing
            arm_pos,arm_ort = p.getBasePositionAndOrientation(mod3)
            arm_dist = arm_pos[1] - pos[1] - (body_width-arm_width)
            y_vel += limb_coeff*arm_dist
        y_vel = vel[1]
    vel = [vel[0],y_vel,vel[2]]
    vel_ctrl(body, vel)

# centralized velocity control function
def ctrl_cen(vel):
    obstacle_pos,obstacle_ort = p.getBasePositionAndOrientation(obs)
    pos,ort = p.getBasePositionAndOrientation(bodyId)
    y_vel = vel[1]
    # avoid objects
    obstacle_dist_x = obstacle_pos[0]-(pos[0]+0.5+body_length/2+arm_length)
    obstacle_dist_y = obstacle_pos[1]-pos[1]
    if (obstacle_dist_x<sens_dist and obstacle_dist_x>0):
        if (obstacle_dist_y<body_width/2+sens_width/2 and obstacle_dist_y>0):
            y_vel -= obs_coeff/obstacle_dist_x
        elif (obstacle_dist_y>-body_width/2-sens_width/2 and obstacle_dist_y<0):
            y_vel -= obs_coeff/obstacle_dist_x
    vel = [vel[0],y_vel,vel[2]]
    for body in [mod1, mod2, mod3, mod4]:
        vel_ctrl(body, vel)

dels = [0, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1]
#vels = [15, 17.5, 20, 22.5, 25, 27.5, 30]
minArr = []

for d in dels:
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0,-10)
    planeId = p.loadURDF("plane.urdf", [0,0,0])

    sF = 1.2
    arm_height = 0.2*2
    arm_width = 0.1/sF
    arm_length = 0.4/sF
    body_height = 0.1*2
    body_width = .3/sF
    body_length = .4/sF

    # create bodies
    #mod1 = p.loadURDF("cube_small.urdf",[body_length/2+arm_width,body_width/2-arm_width/2,0.5],globalScaling=5)
    mod1 = p.loadURDF("cube_small.urdf",[body_length/2+arm_width+arm_length+0.07,body_width/2-arm_width/2,arm_height/2],globalScaling=1./sF)
    p.changeVisualShape(mod1,-1,rgbaColor=[1,0,0,1])
    mod2 = p.loadURDF("cube_small.urdf",[body_length/2+arm_width+arm_length+0.07,-body_width/2+arm_width/2,arm_height/2],globalScaling=1./sF)
    p.changeVisualShape(mod2,-1,rgbaColor=[0,1,0,1])
    mod3 = p.loadURDF("cube_small.urdf",[-arm_length-body_length/2-arm_width/2-0.10,body_width/2-arm_width/2,arm_height/2],globalScaling=1./sF)
    p.changeVisualShape(mod3,-1,rgbaColor=[1,0,1,1])
    mod4 = p.loadURDF("cube_small.urdf",[-arm_length-body_length/2-arm_width/2-0.10,-body_width/2+arm_width/2,arm_height/2],globalScaling=1./sF)
    p.changeVisualShape(mod4,-1,rgbaColor=[0,0,1,1])

    corrFac = 0.02
    bodyId = p.loadURDF("body.urdf", [0,0,body_height],globalScaling=0.1/sF)

    armId1 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+arm_width/2+corrFac,body_width/2-arm_width/2,arm_height/2], scale = 0.01/sF, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)
    armId2 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+arm_width/2+corrFac,-body_width/2+arm_width/2,arm_height/2], scale = 0.01/sF, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 0, useSelfCollision = 1, frictionCoeff = 0.1, useFaceContact=1)
    armId3 = p.loadSoftBody("arm_hori.obj", basePosition = [-arm_length-body_length/2-arm_width/2-corrFac,body_width/2-arm_width/2,arm_height/2], scale = 0.01/sF, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)
    armId4 = p.loadSoftBody("arm_hori.obj", basePosition = [-arm_length-body_length/2-arm_width/2-corrFac,-body_width/2+arm_width/2,arm_height/2], scale = 0.01/sF, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)

    p.changeVisualShape(bodyId,-1,rgbaColor=[160/255,69/255,19/255,1])

    # attach bodies
    p.createSoftBodyAnchor(armId1,7,bodyId,-1)
    p.createSoftBodyAnchor(armId1,8,bodyId,-1)
    p.createSoftBodyAnchor(armId1,16,bodyId,-1)
    p.createSoftBodyAnchor(armId1,17,bodyId,-1)

    p.createSoftBodyAnchor(armId2,7,bodyId,-1)
    p.createSoftBodyAnchor(armId2,8,bodyId,-1)
    p.createSoftBodyAnchor(armId2,16,bodyId,-1)
    p.createSoftBodyAnchor(armId2,17,bodyId,-1)

    p.createSoftBodyAnchor(armId3,0,bodyId,-1)
    p.createSoftBodyAnchor(armId3,1,bodyId,-1)
    p.createSoftBodyAnchor(armId3,2,bodyId,-1)
    p.createSoftBodyAnchor(armId3,3,bodyId,-1)

    p.createSoftBodyAnchor(armId4,0,bodyId,-1)
    p.createSoftBodyAnchor(armId4,1,bodyId,-1)
    p.createSoftBodyAnchor(armId4,2,bodyId,-1)
    p.createSoftBodyAnchor(armId4,3,bodyId,-1)

    p.createSoftBodyAnchor(armId1,0,mod1,-1)
    p.createSoftBodyAnchor(armId1,1,mod1,-1)
    p.createSoftBodyAnchor(armId1,2,mod1,-1)
    p.createSoftBodyAnchor(armId1,3,mod1,-1)

    p.createSoftBodyAnchor(armId2,0,mod2,-1)
    p.createSoftBodyAnchor(armId2,1,mod2,-1)
    p.createSoftBodyAnchor(armId2,2,mod2,-1)
    p.createSoftBodyAnchor(armId2,3,mod2,-1)

    p.createSoftBodyAnchor(armId3,7,mod3,-1)
    p.createSoftBodyAnchor(armId3,8,mod3,-1)
    p.createSoftBodyAnchor(armId3,16,mod3,-1)
    p.createSoftBodyAnchor(armId3,17,mod3,-1)

    p.createSoftBodyAnchor(armId4,7,mod4,-1)
    p.createSoftBodyAnchor(armId4,8,mod4,-1)
    p.createSoftBodyAnchor(armId4,16,mod4,-1)
    p.createSoftBodyAnchor(armId4,17,mod4,-1)

    # create obstacle
    obstacle_pos = (10,-0.1,0.25) # position of obstacle to avoid
    obs = p.loadURDF("cube_small.urdf",basePosition=obstacle_pos,globalScaling=5)
    p.changeVisualShape(obs,-1,rgbaColor=[0,0,0,1])

    numLinks = p.getMeshData(armId1)
    pos,ort = p.getBasePositionAndOrientation(armId1)

    # run simulation
    useRealTimeSimulation = 0
    if (useRealTimeSimulation):
      p.setRealTimeSimulation(0)

    posArr1 = []
    posArr2 = []
    posArr3 = []
    posArr4 = []
    distArr = []
    timeArr = []

    mode = "cen" # choose whether to use centralized or decentralized control
    target_vel = (20,0,0) # target velocity x,y,z
    #target_vel = (v,0,0) # target velocity x,y,z

    comm_delay = d
    start = time.time()
    start_perm = time.time()
    while time.time() - start_perm < 10:
        if mode == "cen":
            #ctrl_cen(target_vel)
            if time.time()-start > comm_delay:
                ctrl_dec(mod1, target_vel)
                ctrl_dec(mod2, target_vel)
                ctrl_dec(mod3, target_vel)
                ctrl_dec(mod4, target_vel)
                start = time.time()
        elif mode == "dec":
            ctrl_dec(mod1, target_vel)
            ctrl_dec(mod2, target_vel)
            ctrl_dec(mod3, target_vel)
            ctrl_dec(mod4, target_vel)

        pos1,ort = p.getBasePositionAndOrientation(mod1)
        pos2,ort = p.getBasePositionAndOrientation(mod2)
        pos3,ort = p.getBasePositionAndOrientation(mod3)
        pos4,ort = p.getBasePositionAndOrientation(mod4)
        pos,ort  = p.getBasePositionAndOrientation(bodyId)

        obs_dist = pow((pow((pos4[0] - obstacle_pos[0]), 2) + pow((pos4[1] - obstacle_pos[1]), 2)), 0.5)
        distArr.append(obs_dist)
        timeArr.append(time.time() - start_perm)

        posArr1.append(pos1)
        posArr2.append(pos2)
        posArr3.append(pos3)
        posArr4.append(pos4)
        p.stepSimulation()
    minArr.append(min(distArr))
    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

plt.scatter(dels,minArr,color='black')
plt.xlabel("communication delay s")
plt.ylabel("minnimum obstacle distance (m)")
plt.show()
