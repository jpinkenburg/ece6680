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
p.setGravity(0,0,0)
arm_height = 0.2/2
arm_width = 0.1/2
arm_length = 0.4/2
body_height = 0.1/2
body_width = .3/2
body_length = .4/2

sF = 2

# create bodies
mod1 = p.loadURDF("cube_small.urdf",[body_length/2+arm_width/2+arm_length+0.14,body_width/2-arm_width/2,arm_height/2],globalScaling=1./sF)
p.changeVisualShape(mod1,-1,rgbaColor=[1,0,0,1])
mod2 = p.loadURDF("cube_small.urdf",[body_length/2+arm_width/2+arm_length+0.14,-body_width/2+arm_width/2,arm_height/2],globalScaling=1./sF)
p.changeVisualShape(mod2,-1,rgbaColor=[0,1,0,1])
mod3 = p.loadURDF("cube_small.urdf",[-arm_length-body_length/2-arm_width/2-0.14,body_width/2-arm_width/2,arm_height/2],globalScaling=1./sF)
p.changeVisualShape(mod3,-1,rgbaColor=[1,0,1,1])
mod4 = p.loadURDF("cube_small.urdf",[-arm_length-body_length/2-arm_width/2-0.14,-body_width/2+arm_width/2,arm_height/2],globalScaling=1./sF)
p.changeVisualShape(mod4,-1,rgbaColor=[0,0,1,1])

corrFac = 0.05
bodyId = p.loadURDF("body.urdf", [0,0,body_height],globalScaling=0.1/sF)
armId1 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+arm_width/2+corrFac,body_width/2-arm_width/2,arm_height/2], scale = 0.01/sF, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)
armId2 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+arm_width/2+corrFac,-body_width/2+arm_width/2,arm_height/2], scale = 0.01/sF, mass = 1., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=121, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)
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
k_p = 0 # Proportional constant
k_i = 0#.5 # Integral constant
k_d = 0#.01 # Differential constant
# goal_vel_x = [0,0,0,0] # Velocity goal of each wheel
goal_vel_x = np.array([5,5,5,5])*5 # Velocity goal of each wheel
goal_pos_y = [0.05,-0.05,0.05,-0.05] # Position goal of each wheel
# goal_pos_y = [0,0,0,0] # Velocity goal of each wheel
lastControlTime = 0
# target_vel = (50,10,0)
# turns = 1
control_x_prev = [0,0,0,0]
# control_y_prev = [0,0,0,0]
control_y_prev = [1,-1,1,-1]
error_x_prev = [0,0,0,0]
error_y_prev = [0,0,0,0]
time_prev = time.time()

# For velocity plot, goal vel is 25, kp is 10, no kd ki, both x y same k no  multiplier, y vel control,run for 10s, no need different force 12 vs 34
# For bad disturbance, noise use 3std, kp 10, pos control y, force divide by 5. x P y P
# For good disturbance, noise 3std, kp 10 kd 0.01, force divide by 3, x P y PD

start_time = time.time()
store_data_time = time.time()
modArr = [mod1,mod2,mod3,mod4]
for i in range(4):
  print(p.getBasePositionAndOrientation(modArr[i]))
while time.time() - start_time < 10:
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
      # temp = p.getBaseVelocity(mods[i])
      # temp = temp[0][1] # Get index of y velocity
      # pos_y.append(temp)

    # print(pos_y)

    # noise. Use noise of 3 for simulation
    for i in range(4):
    #    p.applyExternalForce(mods[i],-1,[np.random.normal(0, 3),np.random.normal(0, 3),0],[0,0,0],p.LINK_FRAME)
      pass

    # Apply force: either random noise or control
    if 1==1:#time.time() - lastControlTime > 0.005: # Control every .1 second
      # Velocity control for x axis
      control_x = []
      for i in range(4):
        error = vel_x[i] - goal_vel_x[i]
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
        P = k_p * error
        I = control_y_prev[i] + k_i * error * (time.time() - time_prev)
        D = k_d * (error - error_y_prev[i]) / (time.time() - time_prev)
        control_y.append(-P-D)
        # control_y.append(0)
        control_y_prev[i] = control_y[i]
        error_y_prev[i] = error
    #   print(control_y)

      time_prev = time.time()


      # Apply control as force
      for i in range(4):
        # p.applyExternalForce(mods[i],-1,[control_x[i],control_y[i],0],[0,0,0],p.LINK_FRAME)

        p.applyExternalForce(mods[i],-1,np.array([control_x[i] + control_y[i]/3,0,0])/20,[0,-1/4,0],p.WORLD_FRAME)
        p.applyExternalForce(mods[i],-1,np.array([control_x[i] - control_y[i]/3,0,0])/20,[0,1/4,0],p.WORLD_FRAME)
        

        # if i == 1 or i == 2:
        #   p.applyExternalForce(mods[i],-1,[control_x[i] - control_y[i]/5,0,0],[0,1/4,0],p.LINK_FRAME)
        #   p.applyExternalForce(mods[i],-1,[control_x[i] + control_y[i]/5,0,0],[0,-1/4,0],p.LINK_FRAME)
        # else:
        #   p.applyExternalForce(mods[i],-1,[control_x[i] + control_y[i]/5,0,0],[0,-1/4,0],p.LINK_FRAME)
        #   p.applyExternalForce(mods[i],-1,[control_x[i] - control_y[i]/5,0,0],[0,1/4,0],p.LINK_FRAME)
      
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
    for i in range(4):
      ax1.plot(masterArr[:,0,i],masterArr[:,1,i],color=colors[i])
      ax1.scatter(masterArr[0,0,i],masterArr[0,1,i],color='black')
    for i in range(4):
      ax2.plot(np.linspace(1, len(velArr1), num=len(velArr1)), masterArr2[:,i],color=colors[i])
    ax1.set(xlabel='x', ylabel='y')
    ax2.set(xlabel='time', ylabel='x velocity')
    plt.show()
    raise(EOFError)


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
label = ['limb 1','limb 2','limb 3','limb 4']
fig, (ax1, ax2) = plt.subplots(2)
for i in range(4):
  test = ax1.plot(masterArr[:,0,i],masterArr[:,1,i],color=colors[i], label=label[i])
  ax1.scatter(masterArr[0,0,i],masterArr[0,1,i],color='black')
ax1.legend()
for i in range(4):
  ax2.plot(np.linspace(0, 10, num=len(velArr1)), masterArr2[:,i],color=colors[i], label=label[i])
ax1.set(xlabel='x position (m)', ylabel='y position (m)')
ax2.set(xlabel='time (s)', ylabel='x velocity (m/s)')
ax2.legend()
plt.show()
# raise(EOFError)
