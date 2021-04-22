import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

physicsClient = p.connect(p.GUI)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0,0,0])
p.setGravity(0,0,0)
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

pos1,ort = p.getBasePositionAndOrientation(mod1)
arm1,ort = p.getBasePositionAndOrientation(armId1)
print(pos1,arm1)
print("2")
pos1,ort = p.getBasePositionAndOrientation(mod2)
arm1,ort = p.getBasePositionAndOrientation(armId2)
print(pos1,arm1)
print("3")
pos1,ort = p.getBasePositionAndOrientation(mod3)
arm1,ort = p.getBasePositionAndOrientation(armId3)
print(pos1,arm1)
print("4")
pos1,ort = p.getBasePositionAndOrientation(mod4)
arm1,ort = p.getBasePositionAndOrientation(armId4)
print(pos1,arm1)


# run simulation
useRealTimeSimulation = 0
if (useRealTimeSimulation):
  p.setRealTimeSimulation(0)

posArr1 = []
posArr2 = []
posArr3 = []
posArr4 = []

while 1:
  try:
    p.applyExternalForce(mod1,-1,[30,0,0],[0,0,0],p.LINK_FRAME)
    p.applyExternalForce(mod2,-1,[30,0,0],[0,0,0],p.LINK_FRAME)
    p.applyExternalForce(mod3,-1,[30,0,0],[0,0,0],p.LINK_FRAME)
    p.applyExternalForce(mod4,-1,[30,0,0],[0,0,0],p.LINK_FRAME)
    pos1,ort = p.getBasePositionAndOrientation(mod1)
    pos2,ort = p.getBasePositionAndOrientation(mod2)
    pos3,ort = p.getBasePositionAndOrientation(mod3)
    pos4,ort = p.getBasePositionAndOrientation(mod4)
    posArr1.append(pos1)
    posArr2.append(pos2)
    posArr3.append(pos3)
    posArr4.append(pos4)
    p.stepSimulation()
    #print('a')
    sleep(1/240.)

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
