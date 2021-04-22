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
body_length = 1
arm_width = 1
arm_height = 1
body_width = 1

mod4 = p.loadURDF("cube_small.urdf",basePosition=[-1.25,6,1],globalScaling=10)
p.changeVisualShape(mod4,-1,rgbaColor=[0,0,1,1])
armId2 = p.loadSoftBody("arm_hori.obj", basePosition = [body_length/2+arm_width/2,-body_width/2+arm_width/2,arm_height/2], scale = 0.1, mass = 100., useNeoHookean = 1, useBendingSprings=1, useMassSpring=1, springElasticStiffness=400, springDampingStiffness=1, springDampingAllDirections = 1, useSelfCollision = 1, frictionCoeff = 0, useFaceContact=1)

p.createSoftBodyAnchor(armId2,7,mod4,-1)

while 1:
    #p.applyExternalForce(mod4,-1,[0,0,0],[0,0,0],p.LINK_FRAME)
    p.stepSimulation()
    sleep(1/100.)
