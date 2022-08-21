import pybullet as p
from time import sleep
import pybullet_data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
car = p.loadURDF('./Multiagent-Reaction/multiagent_reaction/resources/simplehuman.urdf',[0,0,3])
# number_of_joints = p.getNumJoints(car)
# angle = p.addUserDebugParameter('steering',-0.5,0.5,0)
# throttle = p.addUserDebugParameter('Throttle',0,20,0)
plane = p.loadURDF('./Multiagent-Reaction/multiagent_reaction/resources/simpleplane.urdf')

sleep(3)
while True:
    p.stepSimulation()