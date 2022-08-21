from http import client
import pybullet as p
import numpy as np
import os

class AGV:
    def __init__(self) -> None:
        self.client = client
        self.agv = p.loadURDF("./Multiagent-Reaction/multiagent_reaction/resources/simplecar.urdf",
                             basePosition=[0,0,3],
                             physicsCClientId = client
                             )
        self.steering_joints = [0,2]
        self.drive_joints = [1,3,4,5]
    
    def get_ids(self):

        return self.agv,self.client
    
    def control(self,action):
        throttle,steering_angle = action
        throttle = min(max(throttle,0),20)
        steering_angle = max(min(steering_angle,0.5),-0.5)
        #steering control
        p.setJointMotorControlArray(self.agv,self.steering_joints,
                                    controlMode = p.POSITION_CONTROL,
                                    targetPosition = [steering_angle] *2,
                                    physicsClientId = self.client)
         #drive control
        self.joints_speed = throttle
        p.setJointMotorControlArray(
            bodyUniqueId = self.agv,
            jointIndices = self.drive_joints,
            controlMode = p.VELOCITY_CONTROL,
            
        )
        