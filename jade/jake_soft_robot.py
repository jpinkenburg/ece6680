import pybullet as p
from time import sleep
import pybullet_data


physicsClient = p.connect(p.GUI)
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0,0)
planeId = p.loadURDF("plane.urdf", [0,0,0])

arm_height = 1
arm_width = 1
arm_length = 4
body_height = 1
body_width = 3
body_length = 4

# create bodies
bodyId = p.loadURDF("body.urdf", [0,0,0.5])
armId1 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+arm_width/2,body_width/2-arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 0, useBendingSprings=1, useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = 0, useFaceContact=1)
armId2 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+arm_width/2,-body_width/2+arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 0, useBendingSprings=1, useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = 0, useFaceContact=1)
armId3 = p.loadSoftBody("arm_hori.obj", basePosition = [-arm_length-body_length/2-arm_width/2,body_width/2-arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 0, useBendingSprings=1, useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = 0, useFaceContact=1)
armId4 = p.loadSoftBody("arm_hori.obj", basePosition = [-arm_length-body_length/2-arm_width/2,-body_width/2+arm_width/2,arm_height/2], scale = 0.1, mass = 1., useNeoHookean = 0, useBendingSprings=1, useMassSpring=1, springElasticStiffness=40, springDampingStiffness=.1, springDampingAllDirections = 1, useSelfCollision = 0, frictionCoeff = 0, useFaceContact=1)

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

# apply force


# run simulation
useRealTimeSimulation = 0

if (useRealTimeSimulation):
  p.setRealTimeSimulation(0)

while 1:
  if (useRealTimeSimulation):
    p.setGravity(0, 0, -10)
    sleep(0.01)  # Time in seconds.
  else:
    p.stepSimulation()
