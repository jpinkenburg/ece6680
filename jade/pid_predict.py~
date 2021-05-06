import pybullet as p
from time import sleep
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt

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
armId1 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+arm_width/2+corrFac,body_width/2-arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0.2, useFaceContact=1)
armId2 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+arm_width/2+corrFac,-body_width/2+arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0.2, useFaceContact=1)
armId3 = p.loadSoftBody("arm_hori.obj", basePosition = [-arm_length-body_length/2-arm_width/2-corrFac,body_width/2-arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0.2, useFaceContact=1)
armId4 = p.loadSoftBody("arm_hori.obj", basePosition = [-arm_length-body_length/2-arm_width/2-corrFac,-body_width/2+arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0.2, useFaceContact=1)

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


# run simulation
useRealTimeSimulation = 0
if (useRealTimeSimulation):
  p.setRealTimeSimulation(0)

#initialize position array
posArr = np.zeros([1,4,3])
modArr = [mod1,mod2,mod3,mod4]
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

def control_prop_x(modArray,curLocs,desiredLocs,c=1):
  force = np.zeros([1,4,3])
  if len(curLocs.shape) == 3:
    curLocs = curLocs[0]
  for i in range(len(modArray)):
    f = [c*(desiredLocs[i,0]-curLocs[i,0]),0,0]
    p.applyExternalForce(modArray[i],-1,f,[0,0,0],p.LINK_FRAME)
    force[:,i,:] = f
  return force

def apply_forces(modArray,curLocs,desiredLocs,f,modifier=-10):
  force = np.zeros([1,4,3])
  if len(curLocs.shape) == 3:
    curLocs = curLocs[0]
  for i in range(len(modArray)):
    if modifier == -10:
      ff = f(curLocs[i,:],desiredLocs[i,:])
    else:
      ff = f(curLocs[i,:],desiredLocs[i,:],modifier)
    print(desiredLocs[i,:]-curLocs[i,:])
    p.applyExternalForce(modArray[i],-1,ff,[0,0,0],p.LINK_FRAME)
    force[:,i,:] = ff
  return force

    
def get_positions(modArray):
  poses = np.zeros([1,len(modArray),3])
  for i in range(len(modArray)):
    pos,_ = p.getBasePositionAndOrientation(modArray[i])
    poses[:,i,:] = pos
  return poses

#probably need to calculate minimum force to get some sort of meaningful object avoidance
#Try comparing PID controller to other thing

forces = np.zeros([1,4,3])
global intErr,prevErr,kp,ki,kd 

intErr= np.zeros([1,4,3])
prevErr = np.zeros([1,4,3])
kp,ki,kd = [100,0.5,0]

def pid(desPos,curPos):
  global intErr,prevErr
  if len(curPos.shape) == 3:
    curPos = curPos[0]
  err = desPos-curPos
  intErr += err*tstep
  dErr = (err-prevErr)/tstep
  prevErr = err
  f = kp*err + ki*intErr + kd*dErr
  print(f.shape)
  for i in range(4):
    f[0,i,2] = 0.
    if f[0,i,0] > 200:
      f[0,i,0] = 200
    p.applyExternalForce(modArr[i],-1,f[0,i],[0,0,0],p.LINK_FRAME)
  return f

j=0
while 1:
  try:
    poses = get_positions(modArr)
    #f=control_prop_x(modArr,poses,des,50
    pid(des,poses)
    print(poses)
    des[:,0] += tstep*50 #10 units/sec
    posArr = np.append(posArr,poses,0)
    #forces = np.append(forces,f,0)
    p.stepSimulation()
    sleep(1/100.)

  except(KeyboardInterrupt):
    masterArr = posArr
    print(posArr.shape)
    colors = ['red','green','purple','blue']
    for i in range(4):
      plt.scatter(masterArr[:,i,0],masterArr[:,i,1],color=colors[i])
      plt.scatter(masterArr[0,i,0],masterArr[0,i,1],color='black')
    plt.xlabel("world X position")
    plt.ylabel("world Y Position")
    #plt.title("
    plt.savefig("yee.png")

    #plt.figure()
    #plt.plot(np.arange(forces.shape[0]),forces[:,2,1])
    #plt.xlabel("step num")
    #plt.ylabel("Applied X force")
    
    plt.show()
    raise(EOFError)

#diagonal lines means that the robot is achieving liftoff!! :/
#goes through the plane??