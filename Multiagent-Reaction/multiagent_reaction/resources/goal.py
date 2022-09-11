import pybullet as p
import os


class Goal:
    def __init__(self, client, base):
        #f_name = os.path.join(os.path.dirname(__file__), 'simplegoal.urdf')
        self.goal = p.loadURDF(fileName='./Multiagent-Reaction/multiagent_reaction/resources/simplegoal.urdf',
                        basePosition=[base[0], base[1], 0.1],
                        physicsClientId=client)
    
    def get_ids(self):
        
        return self.goal 


