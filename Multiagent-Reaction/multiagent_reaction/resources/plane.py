import pybullet as p
import os


class Plane:
    def __init__(self, client):
        #f_name = os.path.join(os.path.dirname(__file__), 'simpleplane.urdf')
        self.plane=p.loadURDF(fileName='./Multiagent-Reaction/multiagent_reaction/resources/simpleplane.urdf',
                            basePosition=[0, 0, 0],
                            physicsClientId=client)
    
    def get_ids(self):
        return self.plane


