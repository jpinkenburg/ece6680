import pybullet as p
# from time import sleep
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import time
import sys
import make_body_plot as m

physicsClient = p.connect(p.DIRECT)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf", [0,0,0],globalScaling=100)
p.setGravity(0,0,-10)
sF = 1.2
arm_height = 0.2*2
arm_width = 0.1/sF
arm_length = 0.4/sF
body_height = 0.1*2
body_width = .3/sF
body_length = .4/sF

# create bodies
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
k_p = 3 # Proportional constant
k_i = 0.5 # Integral constant
k_d = 0.01 # Differential constant
# goal_vel_x = [0,0,0,0] # Velocity goal of each wheel
goal_vel_x = np.array([25,25,25,25])*4/25 # Velocity goal of each wheel
goal_pos_y = np.array([1.,-1.,1.,-1.])/sF # Position goal of each wheel
goal_vel_y = np.array([1,1,1,1])*0
# goal_pos_y = [0,0,0,0] # Velocity goal of each wheel
lastControlTime = 0
# target_vel = (50,10,0)
# turns = 1
control_x_prev = np.array([0,0,0,0])
# control_y_prev = [0,0,0,0]
control_y_prev = np.array([1,-1,1,-1])*0/sF
error_x_prev = [0,0,0,0]
error_y_prev = [0,0,0,0]
time_prev = time.time()
argv = sys.argv

# For velocity plot, goal vel is 25, kp is 10, no kd ki, both x y same k no  multiplier, y vel control,run for 10s, no need different force 12 vs 34
# For bad disturbance, noise use 3std, kp 10, pos control y, force divide by 5. x P y P
# For good disturbance, noise 3std, kp 10 kd 0.01, force divide by 3, x P y PD


    

start_time = time.time()
store_data_time = time.time()

desPos = np.zeros([4,3])
for i in range(4):
  desPos[i,:] = p.getBasePositionAndOrientation(mods[i])[0]
  print(desPos[i,:])

while time.time()-start_time < 0:
  p.stepSimulation()

start_time = time.time()
save=False

vel_x = np.zeros(4)

if len(sys.argv) > 3:
  k_p = float(sys.argv[1])
  k_i = float(sys.argv[2])
  k_d = float(sys.argv[3])
  save=False
  
endTime = 20
kp = 2
ki = 0
kd = 0.1
prevErr = np.zeros(3)
interr = np.zeros(3)
p.setTimeStep(1/100.)
while time.time() - start_time < endTime:
  for i in range(4):
    curPos = p.getBasePositionAndOrientation(mods[i])[0]
    err = desPos[i,:] - curPos
    force = np.zeros(3)
    force += kp*err
    force += kd*(err-prevErr)
    interr += err
    force += ki*interr
    prevErr = err
    p.applyExternalForce(mods[i],-1,force,[0,0,0],p.LINK_FRAME)
    desPos[i,:] += np.array([0.01,0,0])*25
    time.sleep(1/100.)
    p.stepSimulation()
    vel_x[i] = p.getBaseVelocity(mods[i])[0][0]


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


np.save("limbPos.npy",masterArr)
np.save("limbVels.npy",masterArr2)
#fig,ax1,ax2 = m.make_graded_limb_plot(masterArr,masterArr2,0,20)
#fig2,ax3 = m.make_plot(posArr5,0,endTime)
m.make_graded_limb_plot(masterArr,masterArr2,0,20)
#plt.plot(masterArr[:,0,0],masterArr[:,0,0],color='blue')
if save:
  title=argv[1]+'kp_'+argv[2]+'ki_'+argv[3]+'kd.png'
  plt.savefig(title)
  raise(EOFError)
plt.show()
# raise(EOFError)
