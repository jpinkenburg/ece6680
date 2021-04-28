import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

physicsClient = p.connect(p.DIRECT)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
arm_height = 5
arm_width = 1
arm_length = 4
body_height = 1
body_width = 3
body_length = 4

def resetSim():
  p.setGravity(0,0,0)
  planeId = p.loadURDF("plane.urdf",[0,0,0])
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

  modArr = [mod1,mod2,mod3,mod4]
  armArr = [armId1,armId2,armId3,armId4]
  partList = [planeId,bodyId,mod1,mod2,mod3,mod4,armId1,armId2,armId3,armId4]

  # run simulation
  useRealTimeSimulation = 0
  if (useRealTimeSimulation):
    p.setRealTimeSimulation(0)
    
  return modArr,armArr,partList

modArr,_,_ = resetSim()
  
#initialize position array
posArr = np.zeros([1,4,3])
for i in range(4):
  pos,_ = p.getBasePositionAndOrientation(modArr[i])
  posArr[:,i,:] = pos

#just try going in a straight line
des = posArr[0]
tstep = 1/100.

def control_prop(modArray,curLocs,desiredLocs,c=1):
  if len(curLocs.shape) == 3:
    curLocs = curLocs[0]
  for i in range(len(modArray)):
    p.applyExternalForce(modArray[i],-1,c*(desiredLocs[i]-curLocs[i]),[0,0,0],p.LINK_FRAME)
    
    
def get_positions(modArray):
  poses = np.zeros([1,len(modArray),3])
  for i in range(len(modArray)):
    pos,_ = p.getBasePositionAndOrientation(modArray[i])
    poses[:,i,:] = pos
  return poses

#also try setting a limit on how much force the motors can exert
t = 0

def make_graph(pos,title="",fname=""):
  colors = ['red','green','purple','blue']
  for i in range(4):
    plt.scatter(pos[:,i,0],pos[:,i,1],color=colors[i])
    plt.scatter(pos[0,i,0],pos[0,i,1],color=colors[i])
  plt.xlabel("World X Position")
  plt.ylabel("World Y Position")
  plt.title(title)
  if fname != "":
    plt.savefig(fname)
  plt.clf()

c=1
while 1:
  try:
    poses = get_positions(modArr)
    control_prop(modArr,poses,des,c)
    des[:,0] += tstep*50 #10 units/sec
    posArr = np.append(posArr,poses,0)
    p.stepSimulation()
    sleep(1/100.)
    t += tstep
    if t>10:
      make_graph(posArr,title="C = "+str(c),fname="pos_ctrl_"+str(c)+".png")
      p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
      modArr,_,_ = resetSim()
      t=0
      des =posArr[0,:,:]
      temp = posArr[0,:,:]
      posArr = np.zeros([1,4,3])
      posArr[0,:,:] = temp
      print(c)
      c+=1
      

  except(KeyboardInterrupt):
    masterArr = posArr
    print(posArr.shape)
    colors = ['red','green','purple','blue']
    for i in range(4):
      plt.scatter(masterArr[:,i,0],masterArr[:,i,1],color=colors[i])
      plt.scatter(masterArr[0,i,0],masterArr[0,i,1],color='black')
    plt.xlabel("world X position")
    plt.ylabel("world Y Position")
    plt.savefig("yee.png")
    plt.show()
    raise(EOFError)
