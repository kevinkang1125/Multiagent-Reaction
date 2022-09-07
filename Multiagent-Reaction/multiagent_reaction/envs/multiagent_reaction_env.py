from http import client
import gym
import numpy as np
import pybullet as p
import math
from cmath import inf
from gym import spaces,error,utils
from gym.utils import seeding
from multiagent_reaction.resources.plane import Plane
from multiagent_reaction.resources.agv import AGV
from multiagent_reaction.resources.goal import Goal

class MultiagentReaction(gym.Env):
    metadata = {'render.modes':['human']}

    def __init__(self,render:bool =False):
        self.action_space = spaces.Box(
            low = np.array([-1,-1],dtype=np.float32),
            high = np.array([1,1],dtype=np.float32)
        )        
        pass
        
        self.observation_space = spaces.Box(
            low = np.array([-inf,-inf,-1,-1,0,0,-inf,0,-inf,-inf,0,-inf,-inf],dtype=np.float32),
            high = np.array([inf,inf,1,1,inf,inf,inf,inf,inf,inf,inf,inf,inf],dtype=np.float32)
        )
        self.np_random = seeding.np_random()
        self.viz = render
        self.client = p.connect(p.GUI if self.viz else p.DIRECT)
        
        self.car = None
        self.plane = None
        self.planeID = None
        self.carId = None
        self.target = None
        self.targetId = None
        self.done = None
        self.state = None
        self.dist_to_target = None
        self._dist_to_target = None
        self.dist_to_human = None
        self.k = 0.5
        self.timesteps = 0
        self.reset()
    

    def step(self, action):
        self.timesteps += 1
        self._dist_to_target = self.dist_to_target
        self.car.control(action)
        p.setSimulation(physiccs = self.client)
        car_obs = self.car.get_observation()
        car_pos = car_obs[:2]
        target_pos = p.getLinkState(self.targetId,0)[0]
        target_pos = target_pos[:2]
        target_pos_rel = (target_pos[0]-car_pos[0],target_pos[1]-car_pos[1])
        self.dist_to_target = math.sqrt(((car_pos[0]-target_pos[0])**2+(car_pos[1]-target_pos[1])**2))
        ## relation between agent and human
        self.dist_to_huamn = 0
        human_pos = (0,0)
        ##
        observation = car_obs+(self.dist_to_target,)+target_pos_rel+(self.dist_to_huamn,)+human_pos
        observation = np.array(observation,dtype=np.float32)
        #termination condition
        done = bool(self.dist_to_target<1)
        # reward init
        reward_reach = 0
        reward_proceed = 0
        reward_col = 0
        # reward
        if not done:
            reward_proceed=(self._dist_to_target-self.dist_to_target)*self.k
            if self.timesteps == 6000:
                reward_timeout = -1000
        elif self.dist_to_target<1:
            reward_reach = 1000
        
        reward = reward_col+reward_proceed+reward_reach+reward_timeout
        reward = np.float32(reward)
        return observation,reward,done,{}

        
    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0,0,-10)
        self.plane = Plane(self.client)
        self.planeID=self.plane.get_ids()
        self.car = AGV(self.client)
        self.carId = self.car.get_ids()
        self.done = False
        goal_x = np.random.randint(3,10)
        goal_y = np.random.randint(-10.10)
        goal = (goal_x,goal_y)
        self.target = Goal(self.client,goal)
        self.targetId = self.target.get_ids()
        car_obs = self.car.get_observation()
        car_pos = car_obs[:2]
        self.dist_to_target = math.sqrt(((car_pos[0]-goal_x)**2+(car_pos[1]-goal_y)**2))
        self.dist_to_human = 0
        human_pos = (0,0)
        full_obs = car_obs+(self.dist_to_target,)+goal+(self.dist_to_human,)+human_pos
        self.state = full_obs
        return np.array(self.state,dtype=np.float32)

        pass
    def render(self):
        pass
    
    def close(self):
        p.disconnect(self.client)
    
    def seed(self,seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return[seed]