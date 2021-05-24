import pybullet as p
# from time import sleep
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import time
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
    #print(desiredLocs[i,:]-curLocs[i,:])
    p.applyExternalForce(modArray[i],-1,ff,[0,0,0],p.LINK_FRAME)
    force[:,i,:] = ff
  return force

def get_positions(modArray):
  poses = np.zeros([1,len(modArray),3])
  for i in range(len(modArray)):
    pos,_ = p.getBasePositionAndOrientation(modArray[i])
    poses[:,i,:] = pos
  return poses

exp = lambda cur,des,scale : np.sign(des-cur)*np.exp(des-cur)
exp_x = lambda cur,des,scale : [np.sign(des[0]-cur[0])*np.exp(des[0]-cur[0]),0,0]
poly = lambda cur,des,pwr: np.power(np.sign(des-cur),pwr+1)*np.power(des-cur,pwr)/10

def startSim():
  posArr1 = []
  posArr2 = []
  posArr3 = []
  posArr4 = []
  posArr5 = []
  
  velArr1 = []
  velArr2 = []
  velArr3 = []
  velArr4 = []
  
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
  #p.changeVisualShape(armId1,-1,rgbaColor=[0,0,0,0.8])
  #p.changeVisualShape(armId2,-1,rgbaColor=[0,0,0,0.8])
  #p.changeVisualShape(armId3,-1,rgbaColor=[0,0,0,0.8])
  #p.changeVisualShape(armId4,-1,rgbaColor=[0,0,0,0.8])

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
  modList = [mod1,mod2,mod3,mod4]
  return modList,bodyId


#numLinks = p.getMeshData(armId1)
#pos,ort = p.getBasePositionAndOrientation(armId1)


# run simulation
useRealTimeSimulation = 0
if (useRealTimeSimulation):
  p.setRealTimeSimulation(0)

# Control related variables
k_p = 10 # Proportional constant
k_i = 0.5 # Integral constant
k_d = .01 # Differential constant
# goal_vel_x = [0,0,0,0] # Velocity goal of each wheel
goal_vel_x = [10,10,10,10] # Velocity goal of each wheel
goal_pos_y = [0.0833,-0.0833,0.0833,-0.0833] # Position goal of each wheel
goal_vel_y = np.array([10,10,10,10])
# goal_pos_y = [0,0,0,0] # Velocity goal of each wheel
lastControlTime = 0
# target_vel = (50,10,0)
# turns = 1
control_x_prev = [0,0,0,0]
# control_y_prev = [0,0,0,0]
control_y_prev = np.array([0.0833,-0.0833,0.0833,-0.0833])*0
error_x_prev = [0,0,0,0]
error_y_prev = [0,0,0,0]
time_prev = time.time()

# For velocity plot, goal vel is 25, kp is 10, no kd ki, both x y same k no  multiplier, y vel control,run for 10s, no need different force 12 vs 34
# For bad disturbance, noise use 3std, kp 10, pos control y, force divide by 5. x P y P
# For good disturbance, noise 3std, kp 10 kd 0.01, force divide by 3, x P y PD
des = np.zeros([4,3])
tstep = 1./100

intErr = 0.
prevErr=0
kp,ki,kd = [10,0.05,0.25]
def pid(kp,ki,kd,modArr,desPos,curPos):
  global intErr,prevErr
  if len(curPos.shape) == 3:
    curPos = curPos[0]
  err = desPos-curPos
  intErr += err*tstep
  dErr = (err-prevErr)/tstep
  prevErr = err 
  f = kp*err + ki*intErr + kd*dErr
  #print(f.shape)
  for i in range(2):
    f[i,2] = 0.
    if f[i,0] > 200:
      f[i,0] = 200
    p.applyExternalForce(modArr[i],-1,f[i],[0,0,0],p.LINK_FRAME)
  return f

def getVels(mods):
  v = np.zeros([1,4,3])
  for i in range(len(mods)):
#    print(p.getBaseVelocity(mods[i]))
    v[:,i,:] = np.array(p.getBaseVelocity(mods[i])[0])
  return v

def runSim(t,kp,ki,kd):
  modArr,bodyId = startSim()
  mods = modArr
  mod1 = modArr[0]
  mod2 = modArr[1]
  mod3 = modArr[2]
  mod4 = modArr[3]
  startTime = time.time()
  posArr = np.zeros([1,4,3])
  forces = posArr.copy()
  velArr = forces.copy()
  x=True
  add = np.zeros([4,3])
  add[:,0] += 1
  speed = 30
  des = np.zeros([4,3])
  tics = 0
  numits = 0
  endtime=9
  xvels = []
  while True:
    try:
      numits += 1
      if time.time()-startTime > 1:
        poses = get_positions(modArr)
        f=pid(kp,ki,kd,modArr,des,poses)
        #p.setJointMotorControl2(bodyId,0,controlMode=p.VELOCITY_CONTROL,targetVelocity=20)
        #f = apply_forces(modArr,poses,des,poly,1)
        des[:,0] += tstep*20 #10 units/sec
        des = poses[-1] + add*speed
        des[0,1] = 0.0833
        des[1,1] = -des[0,1]
        des[2,1] = des[0,1]
        des[3,1] = des[1,1]
        posArr = np.append(posArr,poses,0)
        velArr = np.append(velArr,getVels(modArr),0)
        xvels.append(getVels(modArr)[0,:,0])
        #forces = np.append(forces,f,0)
      p.stepSimulation()
      time.sleep(1/100.)
      tics += 1
      #print(tics)
      if tics > 1400:
        raise(KeyboardInterrupt)
      #print(des)
      
    except(KeyboardInterrupt):
      masterArr = posArr
      #print(posArr.shape)
      colors = ['red','green','purple','blue']
      labels = ['limb 1','limb 2','limb 3','limb 4']
      fig,(ax1,ax2) = plt.subplots(2)
      for i in range(4):
        ax1.plot(masterArr[1:,i,0],masterArr[1:,i,1],color=colors[i],label=labels[i])
        #ax1.scatter(masterArr[0,0,i],masterArr[0,1,i],color='black')
        ax2.plot(np.linspace(1,len(velArr),num=len(velArr))/100,velArr[:,i,0],color=colors[i],label=labels[i])
      ax1.set(xlabel='x position(m)',ylabel='y position (m)')
      ax2.set(xlabel='time (s)',ylabel='x velocity (m/s)')
      ax2.legend()
      ax1.legend()

      title = "morepics/"+str(kp)+"kP_"+str(ki)+"kI_"+str(kd)+"kD.png"
      #plt.savefig(title)
      posArr = np.transpose(posArr,(0,2,1))
      velArr = np.transpose(velArr,(0,2,1))
      #print(posArr.shape)
      #print(velArr.shape)
      #print("A")
      xvels = np.array(xvels)
      m.make_graded_limb_plot(posArr[1:],xvels[1:],0,endtime-1)
      plt.show()

      
      #plt.figure()
      #plt.plot(np.arange(forces.shape[0]),forces[:,2,1])
      #plt.xlabel("step num")
      #plt.ylabel("Applied X force")
      x=False
      p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
  return

runSim(41,1.1,0.0,0)
'''raise(RuntimeError)
runSim(25,1.1,0.05,0.25)
raise(RuntimeError)
for i in range(10):
  runSim(15,0.3,i/100,0.25)
for i in range(10):
  runSim(15,0.2+i/50,0.05,0.25)
for i in range(10):
  runSim(15,0.3,0.05,0.2+i/100)
raise(RuntimeError)
for i in range(60):
  runSim(20,i,0.5,0.01)
  runSim(10,i,0,0.01)
'''