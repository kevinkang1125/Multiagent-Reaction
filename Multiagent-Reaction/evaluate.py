import gym
from stable_baselines3.common.env_checker import check_env
import multiagent_reaction
#from Truck-Trailer import truck_trailer
from stable_baselines3 import PPO
from stable_baselines3.ppo.policies import MlpPolicy
import pybullet as p

env = gym.make('MultiagentReaction-v0',render = True)
#env = env(render = False)
model = PPO.load("./log/best_model")#model name
p.resetDebugVisualizerCamera(cameraDistance = 10,cameraYaw = -90,cameraPitch = -40, cameraTargetPosition = [-0.6,-0.1,-0.15])
obs = env.reset()
for i in range (1000000):
    # obs = env.reset()
    # done = False
    # while not done:
    action,_state = model.predict(obs)
    obs,rewards,done,info = env.step(action)
    y = obs[1]
    x = obs[2]
    #p.resetDebugVisualizerCamera(cameraDistance = 5,cameraYaw = -60,cameraPitch = -30, cameraTargetPosition = [x,y,-0.15])
    #print(action)
    