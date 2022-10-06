from http import client
import pybullet as p
import numpy as np
import os
import math

class AGV:
    def __init__(self,client):
        self.client = client
        self.agv = p.loadURDF("./Multiagent-Reaction/multiagent_reaction/resources/simplecar.urdf",
                             basePosition=[0,0,2],
                             physicsClientId = self.client
                             )
        self.steering_joints = [0,2]
        self.drive_joints = [1,3,4,5]
    
    def get_ids(self):

        return self.agv,self.client
    
    def control(self,action):
        throttle,steering_angle = action
        throttle = 10*(throttle)+10
        steering_angle = 0.8* steering_angle
        throttle = min(max(throttle,0),20)
        #steering_angle = max(min(steering_angle,0.6),-0.6)
        #steering control
        p.setJointMotorControlArray(self.agv,self.steering_joints,
                                    controlMode = p.POSITION_CONTROL,
                                    targetPositions = [steering_angle] * 2,
                                    physicsClientId = self.client)
         #drive control
        self.joints_speed = throttle
        p.setJointMotorControlArray(
            bodyUniqueId = self.agv,
            jointIndices = self.drive_joints,
            controlMode = p.VELOCITY_CONTROL,
            targetVelocities = [self.joints_speed] *4,
            physicsClientId = self.client       
        )
    
    def get_observation(self):
        pos,ang = p.getBasePositionAndOrientation(self.agv,self.client)
        ori = (np.cos(ang[2]),np.sin(ang[2]))
        #pos = pos[:2]
        #get velocity of the car
        lin_vel, ang_vel = p.getBaseVelocity(self.agv,self.client)
        vel = lin_vel[0:2]+(ang_vel[2],)
        #concatenation
        #observation = (pos+ori+vel)

        observation = (ori+vel)
        return observation
        