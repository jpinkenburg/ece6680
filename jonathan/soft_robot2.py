import pybullet as p
# from time import sleep
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import time

physicsClient = p.connect(p.GUI)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0,0,0])
p.setGravity(0,0,-10)
arm_height = 5
arm_width = 1
arm_length = 4
body_height = 1
body_width = 3
body_length = 4

# create bodies
mod1 = p.loadURDF("cube_small.urdf",[body_length/2+arm_width+arm_length+0.25,body_width/2-arm_width/2,2.5],globalScaling=5)
p.changeVisualShape(mod1,-1,rgbaColor=[1,0,0,1])
mod2 = p.loadURDF("cube_small.urdf",[body_length/2+arm_width+arm_length+0.25,-body_width/2+arm_width/2,arm_height/2],globalScaling=5)
p.changeVisualShape(mod2,-1,rgbaColor=[0,1,0,1])
mod3 = p.loadURDF("cube_small.urdf",[-arm_length-body_length/2-arm_width/2-0.75,body_width/2-arm_width/2,arm_height/2],globalScaling=5)
p.changeVisualShape(mod3,-1,rgbaColor=[1,0,1,1])
mod4 = p.loadURDF("cube_small.urdf",[-arm_length-body_length/2-arm_width/2-0.75,-body_width/2+arm_width/2,arm_height/2],globalScaling=5)
p.changeVisualShape(mod4,-1,rgbaColor=[0,0,1,1])

corrFac = 0.05
bodyId = p.loadURDF("body.urdf", [0,0,2.5])
armId1 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+arm_width/2+corrFac,body_width/2-arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)
armId2 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+arm_width/2+corrFac,-body_width/2+arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)
armId3 = p.loadSoftBody("arm_hori.obj", basePosition = [-arm_length-body_length/2-arm_width/2-corrFac,body_width/2-arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)
armId4 = p.loadSoftBody("arm_hori.obj", basePosition = [-arm_length-body_length/2-arm_width/2-corrFac,-body_width/2+arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)

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
posArr5 = []

velArr1 = []
velArr2 = []
velArr3 = []
velArr4 = []

mods = [mod1,mod2,mod3,mod4]

# Control related variables
k = 10 # Proportional constant
# goal_vel_x = [0,0,0,0] # Velocity goal of each wheel
goal_vel_x = [50,50,50,50] # Velocity goal of each wheel
goal_pos_y = [0,0,0,0] # Position goal of each wheel
lastControlTime = 0
# target_vel = (50,10,0)
# turns = 1
while 1:
  try:
    # print(turns)
    # if turns == 0:
    #   # Noise
    #   p.applyExternalForce(mod1,-1,[np.random.normal(0, 10),0,0],[0,1/4,0],p.LINK_FRAME)
    #   p.applyExternalForce(mod2,-1,[np.random.normal(0, 10),0,0],[0,1/4,0],p.LINK_FRAME)
    #   p.applyExternalForce(mod3,-1,[np.random.normal(0, 10),0,0],[0,1/4,0],p.LINK_FRAME)
    #   p.applyExternalForce(mod4,-1,[np.random.normal(0, 10),0,0],[0,1/4,0],p.LINK_FRAME)
    #   turns = 0
    # elif turns == 1:
    #   p.applyExternalForce(mod1,-1,[10,0,0],[0,1/4,0],p.LINK_FRAME)
    #   p.applyExternalForce(mod2,-1,[10,0,0],[0,1/4,0],p.LINK_FRAME)
    #   p.applyExternalForce(mod3,-1,[10,0,0],[0,1/4,0],p.LINK_FRAME)
    #   p.applyExternalForce(mod4,-1,[10,0,0],[0,1/4,0],p.LINK_FRAME)
    #   turns = 1

    # Get velocity of each wheel in x direction
    vel_x = []
    for i in range(4):
      temp = p.getBaseVelocity(mods[i])
      temp = temp[0][0] # Get index of x velocity
      vel_x.append(temp)
    # print(vel_x)

    # Get position of each wheel
    pos_y = []
    for i in range(4):
      temp,_ = p.getBasePositionAndOrientation(mods[i])
      pos_y.append(temp[1]) # Get index of y position
    # print(pos_y)

    # noise
    for i in range(4):
        p.applyExternalForce(mods[i],-1,[np.random.normal(0, 3),np.random.normal(0, 3),0],[0,0,0],p.LINK_FRAME)

    # Apply force: either random noise or control
    if time.time() - lastControlTime > 0.005: # Control every .1 second
      # Velocity control for x axis
      control_x = []
      for i in range(4):
        control_x.append(-(vel_x[i] - goal_vel_x[i]) * k)
        if control_x[i] > 100:
            control_x[i] = 100
        elif control_x[i] < -100:
            control_x[i] = -100
    #   print(control_x)

      # Position control for y axis
      control_y = []
      for i in range(4):
        control_y.append(-(pos_y[i] - goal_pos_y[i])*k)
    #   print(control_y)

      # Apply control as force
      for i in range(4):
        # p.applyExternalForce(mods[i],-1,[control_x[i],control_y[i],0],[0,0,0],p.LINK_FRAME)

        p.applyExternalForce(mods[i],-1,[control_x[i] - control_y[i]/5,0,0],[0,1/4,0],p.LINK_FRAME)
        p.applyExternalForce(mods[i],-1,[control_x[i] + control_y[i]/5,0,0],[0,-1/4,0],p.LINK_FRAME)
      
      lastControlTime = time.time()


    # Store
    pos1,ort = p.getBasePositionAndOrientation(mod1)
    pos2,ort = p.getBasePositionAndOrientation(mod2)
    pos3,ort = p.getBasePositionAndOrientation(mod3)
    pos4,ort = p.getBasePositionAndOrientation(mod4)
    pos5,ort = p.getBasePositionAndOrientation(bodyId)
    posArr1.append(pos1)
    posArr2.append(pos2)
    posArr3.append(pos3)
    posArr4.append(pos4)
    posArr5.append(pos5)

    velArr1.append(vel_x[0])
    velArr2.append(vel_x[1])
    velArr3.append(vel_x[2])
    velArr4.append(vel_x[3])
    p.stepSimulation()

  except(KeyboardInterrupt):
    posArr1 = np.array(posArr1)
    posArr2 = np.array(posArr2)
    posArr3 = np.array(posArr3)
    posArr4 = np.array(posArr4)
    posArr5 = np.array(posArr5)

    # Master array for position
    masterArr = np.zeros([posArr1.shape[0],posArr1.shape[1],5])
    masterArr[:,:,0] = posArr1
    masterArr[:,:,1] = posArr2
    masterArr[:,:,2] = posArr3
    masterArr[:,:,3] = posArr4
    masterArr[:,:,4] = posArr5

    # Master array for velocity in x direction
    masterArr2 = np.zeros([len(velArr1),4])
    masterArr2[:,0] = velArr1
    masterArr2[:,1] = velArr2
    masterArr2[:,2] = velArr3
    masterArr2[:,3] = velArr4

    colors = ['red','green','purple','blue','black']
    fig, (ax1, ax2) = plt.subplots(2)
    for i in range(5):
      ax1.plot(masterArr[:,0,i],masterArr[:,1,i],color=colors[i])
      ax1.scatter(masterArr[0,0,i],masterArr[0,1,i],color='black')
    for i in range(4):
      ax2.plot(np.linspace(1, len(velArr1), num=len(velArr1)), masterArr2[:,i],color=colors[i])
    ax1.set(xlabel='x', ylabel='y')
    ax2.set(xlabel='time', ylabel='x velocity')
    plt.show()
    raise(EOFError)
