from http import client
from threading import local
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

class MultiagentReactionEnv(gym.Env):
    metadata = {'render.modes':['human']}

    def __init__(self,render:bool =False):
        self.action_space = spaces.Box(
            low = np.array([-1,-1],dtype=np.float32),
            high = np.array([1,1],dtype=np.float32)
        )        
        
        self.observation_space = spaces.Box(
            low = np.array([-1,-1,-inf,-inf,-inf,0,-inf,-inf,0,-inf,-inf],dtype=np.float32),
            high = np.array([1,1,inf,inf,inf,inf,inf,inf,inf,inf,inf],dtype=np.float32)
        )
        #[x,y,cos,sin,vx,vy,w,dist,local_x,local_y,dist2obstacle,human_x,human_y]
        #
        self.np_random = seeding.np_random()
        self.viz = render
        self.client = p.connect(p.GUI if self.viz else p.DIRECT)
        
        p.setTimeStep=(1/50,self.client)

        self.car = None
        self.plane = None
        self.planeId = None
        self.carId = None
        self.target = None
        self.targetId = None
        self.done = None
        self.state = None
        self.dist_to_target = None
        self._dist_to_target = None
        self.dist_to_human = None
        self.k = 4
        self.timesteps = 0
        self.accu_reward = 0
        self.reward_record = []
        self.reset()
    

    def step(self, action):
        self.timesteps += 1
        self._dist_to_target = self.dist_to_target
        self.agv.control(action)
        p.stepSimulation(physicsClientId = self.client)
        car_obs = self.agv.get_observation()
        car_pos = car_obs[:4]
        target_pos = p.getBasePositionAndOrientation(self.targetId,0)[0]
        target_pos = target_pos[:2]
        tire_1_pos = p.getLinkState(self.carId,4,self.client)[0]
        tire_2_pos = p.getLinkState(self.carId,5,self.client)[0]
        rear_x = (tire_1_pos[0]+tire_2_pos[0])/2
        rear_y = (tire_1_pos[0]+tire_2_pos[0])/2
        car_center = (rear_x,rear_y)
        local_target_x = -(target_pos[1]-car_center[1])*car_pos[1]+(target_pos[0]-car_center[0])*car_pos[0]
        local_target_y =  (target_pos[1]-car_center[1])*car_pos[0]+(target_pos[0]-car_center[0])*car_pos[1]
        local_coordinate_target = (local_target_x,local_target_y)
        self.dist_to_target = math.sqrt(((car_center[0]-target_pos[0])**2+(car_center[1]-target_pos[1])**2))
        #target_pos_rel = (target_pos[0]-car_pos[0],target_pos[1]-car_pos[1]
        ## relation between agent and human
        self.dist_to_huamn = 0
        human_pos = (0,0)
        ##
        observation = car_obs+(self.dist_to_target,)+local_coordinate_target+(self.dist_to_huamn,)+human_pos
        observation = np.array(observation,dtype=np.float32)
        #termination condition
        done = bool(self.dist_to_target<1.5 or self.timesteps == 3000)
        # reward init
        reward_reach = 0.0
        reward_proceed = 0.0
        reward_col = 0.0
        reward_timeout = 0.0
        # reward
        if not done:
            reward_proceed=(self._dist_to_target-self.dist_to_target)*self.k
            self.accu_reward+=reward_proceed
        if self.dist_to_target<1.5:
            reward_reach = 100
        
        if self.timesteps == 3000:
           reward_timeout = -100
        
        reward = reward_col+reward_proceed+reward_reach+reward_timeout
        reward = np.float64(reward)
        if done:
            reward_final = reward_reach+reward_timeout+self.accu_reward
            self.reward_record.append(reward_final)
            np.savetxt('episodic_reward_3000.txt',self.reward_record)
        return observation,reward,done,{}

     
    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0,0,-10)
        self.plane = Plane(self.client)
        self.planeId=self.plane.get_ids()
        self.agv = AGV(self.client)
        self.carId,_ = self.agv.get_ids()
        self.done = False
        self.timesteps = 0
        self.accu_reward =0 
        goal_x = np.random.randint(8,20)
        goal_y = np.random.randint(-20,20)
        goal = (goal_x,goal_y)
        self.target = Goal(self.client,goal)
        self.targetId = self.target.get_ids()
        car_obs = self.agv.get_observation()
        car_pos = car_obs[:2]
        tire_1_pos = p.getLinkState(self.carId,4,self.client)[0]
        tire_2_pos = p.getLinkState(self.carId,5,self.client)[0]
        rear_x = (tire_1_pos[0]+tire_2_pos[0])/2
        rear_y = (tire_1_pos[0]+tire_2_pos[0])/2
        car_center = (rear_x,rear_y)
        target_pos = goal
        local_target_x = -(target_pos[1]-car_center[1])*car_pos[1]+(target_pos[0]-car_center[0])*car_pos[0]
        local_target_y =  (target_pos[1]-car_center[1])*car_pos[0]+(target_pos[0]-car_center[0])*car_pos[1]
        local_coordinate_target = (local_target_x,local_target_y)
        self.dist_to_target = math.sqrt(((car_center[0]-goal_x)**2+(car_center[1]-goal_y)**2))
        self.dist_to_human = 0
        human_pos = (0,0)
        full_obs = car_obs+(self.dist_to_target,)+local_coordinate_target+(self.dist_to_human,)+human_pos
        self.state = full_obs
        #print(self.state)
        return np.array(self.state,dtype=np.float32)

    def render(self):
        pass
    
    def close(self):
        p.disconnect(self.client)
    
    def seed(self,seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return[seed]