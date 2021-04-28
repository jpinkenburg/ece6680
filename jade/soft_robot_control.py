import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

physicsClient = p.connect(p.GUI)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0,-10)
planeId = p.loadURDF("plane.urdf", [0,0,0])

arm_height = 1
arm_width = 1
arm_length = 5
body_height = 1
body_width = 3
body_length = 4

# create bodies
#mod1 = p.loadURDF("cube_small.urdf",[body_length/2+arm_width,body_width/2-arm_width/2,0.5],globalScaling=5)
mod1 = p.loadURDF("cube_small.urdf",basePosition=[body_length/2+arm_length+1/4,body_width/2-arm_width/2,arm_height/2],globalScaling=10)
p.changeVisualShape(mod1,-1,rgbaColor=[1,0,0,1])
mod2 = p.loadURDF("cube_small.urdf",basePosition=[body_length/2+arm_length+1/4,-body_width/2+arm_width/2,arm_height/2],globalScaling=10)
p.changeVisualShape(mod2,-1,rgbaColor=[0,1,0,1])
mod3 = p.loadURDF("cube_small.urdf",basePosition=[-body_length/2-arm_length-1/4,body_width/2-arm_width/2,arm_height/2],globalScaling=10)
p.changeVisualShape(mod3,-1,rgbaColor=[1,0,1,1])
mod4 = p.loadURDF("cube_small.urdf",basePosition=[-body_length/2-arm_length-1/4,-body_width/2+arm_width/2,arm_height/2],globalScaling=10)
p.changeVisualShape(mod4,-1,rgbaColor=[0,0,1,1])

bodyId = p.loadURDF("body.urdf", [0,0,0.5])
armId1 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+1/2,body_width/2-arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=400, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)
armId2 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+1/2,-body_width/2+arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=400, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)
armId3 = p.loadSoftBody("arm_hori.obj", basePosition = [-arm_length-body_length/2+1/2,body_width/2-arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=400, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)
armId4 = p.loadSoftBody("arm_hori.obj", basePosition = [-arm_length-body_length/2+1/2,-body_width/2+arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=400, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)
#modeId1 = p.loadSoftBody("arm_hori.obj",basePosition = [body_length/2+arm_width/2,body_width/2-arm_width/2,arm_height/2])
p.changeVisualShape(bodyId,-1,rgbaColor=[160/255,69/255,19/255,1])
p.changeVisualShape(armId1,-1,rgbaColor=[0,0,0,0.8])
p.changeVisualShape(armId2,-1,rgbaColor=[0,0,0,0.8])
p.changeVisualShape(armId3,-1,rgbaColor=[0,0,0,0.8])
p.changeVisualShape(armId4,-1,rgbaColor=[0,0,0,0.8])

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

# decentralized velocity control function
def vel_ctrl_dec(body, vel):
    lin_vel,ang_vel = p.getBaseVelocity(body)
    force1 = [(vel[0]-lin_vel[0]) - 10*(vel[1]-lin_vel[1]), 0, 0]
    force2 = [(vel[0]-lin_vel[0]) + 10*(vel[1]-lin_vel[1]), 0, 0]
    p.applyExternalForce(body,-1,force1,[0,1/4,0],p.LINK_FRAME)
    p.applyExternalForce(body,-1,force2,[0,-1/4,0],p.LINK_FRAME)

# centralized velocity control function
def vel_ctrl_cen(vel):
    pos,ort = p.getBasePositionAndOrientation(bodyId)
    obstacle_dist_x = obstacle_pos[0]-body_length/2-arm_length-pos[0]
    obstacle_dist_y = obstacle_pos[1]-pos[1]
    if (obstacle_dist_x<5 and obstacle_dist_x>0) and (obstacle_dist_y>-body_width/2 and obstacle_dist_y<0): # check for detectable object on left side of vehicle
        y_vel = vel[0]*(obstacle_dist_y-body_width/2)/(obstacle_dist_x-body_length/2-arm_length)
    elif (obstacle_dist_x<5 and obstacle_dist_x>0) and (obstacle_dist_y>0 and obstacle_dist_y<body_width/2): # check for detectable object on right side of vehicle
        y_vel = vel[0]*(obstacle_dist_y+body_width/2)/(obstacle_dist_x-body_length/2-arm_length)
    else:
        y_vel = -5*(pos[1])
    vel = [vel[0],y_vel,vel[2]]
    for body in [mod1, mod2, mod3, mod4]:
        lin_vel,ang_vel = p.getBaseVelocity(body)
        force1 = [(vel[0]-lin_vel[0]) - 10*(vel[1]-lin_vel[1]), 0, 0]
        force2 = [(vel[0]-lin_vel[0]) + 10*(vel[1]-lin_vel[1]), 0, 0]
        p.applyExternalForce(body,-1,force1,[0,1/4,0],p.LINK_FRAME)
        p.applyExternalForce(body,-1,force2,[0,-1/4,0],p.LINK_FRAME)
    sleep(0.02) # model communication delay

mode = "decentralized" # choose whether to use centralized or decentralized control
target_vel = (50,0,0) # target velocity x,y,z
obstacle_pos = (20,1,0) # position of obstacle to avoid
while 1:
  try:
    if mode == "centralized":
        vel_ctrl_cen(target_vel)
    elif mode == "decentralized":
        vel_ctrl_dec(mod1, target_vel)
        vel_ctrl_dec(mod2, target_vel)
        vel_ctrl_dec(mod3, target_vel)
        vel_ctrl_dec(mod4, target_vel)

    pos1,ort = p.getBasePositionAndOrientation(mod1)
    pos2,ort = p.getBasePositionAndOrientation(mod2)
    pos3,ort = p.getBasePositionAndOrientation(mod3)
    pos4,ort = p.getBasePositionAndOrientation(mod4)
    posArr1.append(pos1)
    posArr2.append(pos2)
    posArr3.append(pos3)
    posArr4.append(pos4)
    p.stepSimulation()

  except(KeyboardInterrupt):
    posArr1 = np.array(posArr1)
    posArr2 = np.array(posArr2)
    posArr3 = np.array(posArr3)
    posArr4 = np.array(posArr4)

    masterArr = np.zeros([posArr1.shape[0],posArr1.shape[1],4])
    masterArr[:,:,0] = posArr1
    masterArr[:,:,1] = posArr2
    masterArr[:,:,2] = posArr3
    masterArr[:,:,3] = posArr4

    colors = ['red','green','purple','blue']
    for i in range(4):
      plt.scatter(masterArr[:,0,i],masterArr[:,1,i],color=colors[i])
      plt.scatter(masterArr[0,0,i],masterArr[0,1,i],color='black')
    plt.xlabel("world X position")
    plt.ylabel("world Y Position")
    plt.show()
    raise(EOFError)
