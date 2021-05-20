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
sF = 1.2
arm_height = 0.2*2
arm_width = 0.1/sF
arm_length = 0.4/sF
body_height = 0.1*2
body_width = .3/sF
body_length = .4/sF

def startSim():
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

posArr1 = []
posArr2 = []
posArr3 = []
posArr4 = []
posArr5 = []

velArr1 = []
velArr2 = []
velArr3 = []
velArr4 = []


# Control related variables
k_p = 10 # Proportional constant
k_i = 0.5 # Integral constant
k_d = .01 # Differential constant
# goal_vel_x = [0,0,0,0] # Velocity goal of each wheel
goal_vel_x = [30,30,30,30] # Velocity goal of each wheel
goal_pos_y = [0.0833,-0.0833,0.0833,-0.0833] # Position goal of each wheel
# goal_pos_y = [0,0,0,0] # Velocity goal of each wheel
lastControlTime = 0
# target_vel = (50,10,0)
# turns = 1
control_x_prev = [0,0,0,0]
# control_y_prev = [0,0,0,0]
control_y_prev = [0.0833,-0.0833,0.0833,-0.0833]
error_x_prev = [0,0,0,0]
error_y_prev = [0,0,0,0]
time_prev = time.time()

# For velocity plot, goal vel is 25, kp is 10, no kd ki, both x y same k no  multiplier, y vel control,run for 10s, no need different force 12 vs 34
# For bad disturbance, noise use 3std, kp 10, pos control y, force divide by 5. x P y P
# For good disturbance, noise 3std, kp 10 kd 0.01, force divide by 3, x P y PD
def runSim(t,kp,ki,kd):
  modArr,bodyId = startSim()
  mods = modArr
  mod1 = modArr[0]
  mod2 = modArr[1]
  mod3 = modArr[2]
  mod4 = modArr[3]
  k_i = ki
  k_p = kp
  k_d = kd
  time_prev = time.time()
  start_time = time.time()
  store_data_time = time.time()
  for i in range(4):
    print(p.getBasePositionAndOrientation(modArr[i]))
  #raise(ArithmeticError)
  
  posArr1 = []
  posArr2 = []
  posArr3 = []
  posArr4 = []
  posArr5 = []
  
  velArr1 = []
  velArr2 = []
  velArr3 = []
  velArr4 = []
  
  q = False
  while time.time() - start_time < t:
    time.sleep(1./100.)
    try:
    #   if time.time()-start_time < 5:
    #     k_p = 0.2
    #     k_d = 0
    #     k_i = 0
    #   else:
    #     k_p = kp
    #     k_i = ki
    #     k_d = kd
      if time.time()-start_time > 1:
        
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
        #   temp = p.getBaseVelocity(mods[i])
        #   temp = temp[0][1] # Get index of y velocity
        #   pos_y.append(temp)

        # print(pos_y)
        
        # noise. Use noise of 3 for simulation
        for i in range(4):
        #   p.applyExternalForce(mods[i],-1,[np.random.normal(0, 5),np.random.normal(0, 5),0],[0,0,0],p.LINK_FRAME)
          pass
        
        # Apply force: either random noise or control
        if True:#time.time() - lastControlTime > 0.005: # Control every .1 second
          # Velocity control for x axis
          control_x = []
          for i in range(4):
            error = (vel_x[i] - goal_vel_x[i])
            P = k_p * error
            I = control_x_prev[i] + k_i * error * (time.time() - time_prev)
            D = k_d * (error - error_x_prev[i]) / (time.time() - time_prev)
            control_x.append(- P)
            if control_x[i] > 100:
              control_x[i] = 100
            elif control_x[i] < -100:
              control_x[i] = -100
              control_x_prev[i] = control_x[i]
              error_x_prev[i] = error
              #   print(control_x)

          # Position control for y axis
          control_y = []
          for i in range(4):
            error = pos_y[i] - goal_pos_y[i]
            P = k_p * error * 10
            I = control_y_prev[i] + k_i*3 * error * (time.time() - time_prev)
            D = k_d * (error - error_y_prev[i]) / (time.time() - time_prev)
            control_y.append(-P-D)
            # control_y.append(0)
            control_y_prev[i] = control_y[i]
            error_y_prev[i] = error
            #   print(control_y)
            
          time_prev = time.time()


          # Apply control as force
          for i in range(2):
            # p.applyExternalForce(mods[i],-1,[control_x[i],control_y[i],0],[0,0,0],p.LINK_FRAME)
            p.applyExternalForce(mods[i],-1,np.array([control_x[i] ,control_y[i]/1,0]),[0,-0.0,0],p.LINK_FRAME)
            p.applyExternalForce(mods[i],-1,np.array([control_x[i] ,control_y[i]/1,0]),[0,0.0,0],p.LINK_FRAME)
          for i in range(3,4):
            # p.applyExternalForce(mods[i],-1,[control_x[i],control_y[i],0],[0,0,0],p.LINK_FRAME)
            p.applyExternalForce(mods[i],-1,np.array([control_x[i]*0.1 ,control_y[i]/1,0]),[0,-0.0,0],p.LINK_FRAME)
            p.applyExternalForce(mods[i],-1,np.array([control_x[i]*0.1 ,control_y[i]/1,0]),[0,0.0,0],p.LINK_FRAME)
          

            # if i == 1 or i == 2:
            #   p.applyExternalForce(mods[i],-1,[control_x[i] - control_y[i]/5,0,0],[0,1/4,0],p.LINK_FRAME)
            #   p.applyExternalForce(mods[i],-1,[control_x[i] + control_y[i]/5,0,0],[0,-1/4,0],p.LINK_FRAME)
            # else:
            #   p.applyExternalForce(mods[i],-1,[control_x[i] + control_y[i]/5,0,0],[0,-1/4,0],p.LINK_FRAME)
            #   p.applyExternalForce(mods[i],-1,[control_x[i] - control_y[i]/5,0,0],[0,1/4,0],p.LINK_FRAME)
          pass
        
        #lastControlTime = time.time()


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
      return
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
      for i in range(4):
        ax1.plot(masterArr[:,0,i],masterArr[:,1,i],color=colors[i])
        ax1.scatter(masterArr[0,0,i],masterArr[0,1,i],color='black')
      for i in range(4):
        ax2.plot(np.linspace(1, len(velArr1), num=len(velArr1)), masterArr2[:,i],color=colors[i])
      ax1.set(xlabel='x', ylabel='y')
      ax2.set(xlabel='time', ylabel='x velocity')
      plt.savefig()
      p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
      return


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
  label = ['limb 1','limb 2','limb 3','limb 4','body']
  fig, (ax1, ax2) = plt.subplots(2)
  for i in range(5):
    test = ax1.plot(masterArr[:,0,i],masterArr[:,1,i],color=colors[i], label=label[i])
    ax1.scatter(masterArr[0,0,i],masterArr[0,1,i],color='black')
  ax1.legend()
  ax1.set_ylim([-0.5, 0.5])
  for i in range(4):
    ax2.plot(np.linspace(0, t, num=len(velArr1)), masterArr2[:,i],color=colors[i], label=label[i])
  ax1.set(xlabel='x position (m)', ylabel='y position (m)')
  ax2.set(xlabel='time (s)', ylabel='x velocity (m/s)')
  ax2.legend()
  title = str(kp)+"kp_"+str(ki)+"ki_"+str(kd)+"kd.png"
  plt.savefig(title)
  plt.show()
  p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

  print(np.sum(abs(masterArr[:,1,0] - 0.0833)))
  print(np.sum(abs(masterArr[:,1,1] + 0.0833)))
  print(np.sum(abs(masterArr[:,1,2] - 0.0833)))
  print(np.sum(abs(masterArr[:,1,3] + 0.0833)))
  avedev = np.sum(abs(masterArr[:,1,0] - 0.0833)) + np.sum(abs(masterArr[:,1,1] + 0.0833)) + np.sum(abs(masterArr[:,1,2] - 0.0833)) + np.sum(abs(masterArr[:,1,3] + 0.0833))
  avedev = avedev / 4
  print(avedev)
  print(np.sum(abs(masterArr[:,1,4])))
  print((avedev - np.sum(abs(masterArr[:,1,4])))/avedev)

  return
  # raise(EOFError)

for i in range(1):
#  runSim(30,0.2,0.2,0.5)
  runSim(40,1,0,0.01)
  #runSim(10,10,0,0.01)