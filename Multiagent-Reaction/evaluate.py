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
p.resetDebugVisualizerCamera(cameraDistance = 20,cameraYaw = -90,cameraPitch = -89, cameraTargetPosition = [8,0,-0.15])
obs = env.reset()
done = False
for i in range (10):
    obs = env.reset()
    done = False
    step = 0
    record_steer = []
    record_velocity = []
    while not done:
        action,_state = model.predict(obs)
        obs,rewards,done,info = env.step(action)
        x = action[0]
        y = action[1]
        record_steer.append(10*x+10)
        record_velocity.append(0.8*y)
    i+=1


    