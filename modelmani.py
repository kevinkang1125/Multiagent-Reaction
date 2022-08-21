import pybullet as p
from time import sleep
import pybullet_data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
car = p.loadURDF('./Multiagent-Reaction/multiagent_reaction/resources/simplecar.urdf',[0,0,1])
number_of_joints = p.getNumJoints(car)
angle = p.addUserDebugParameter('steering',-0.5,0.5,0)
throttle = p.addUserDebugParameter('Throttle',0,20,0)
plane = p.loadURDF('./Multiagent-Reaction/multiagent_reaction/resources/simpleplane.urdf')

sleep(3)
wheel_indices = [1,3,4,5]
hinge_indices = [0,2]

while True:
    user_angle = p.readUserDebugParameter(angle)
    user_throttle = p.readUserDebugParameter(throttle)
    for joint_index in wheel_indices:
        p.setJointMotorControl2(car,joint_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity = user_throttle
                                )
    for joint_index in hinge_indices:
        p.setJointMotorControl2(car,joint_index,
                                p.POSITION_CONTROL,
                                targetPosition = user_angle)
    p.stepSimulation()

