import gym
from stable_baselines3.common.env_checker import check_env
import multiagent_reaction
from stable_baselines3 import PPO
from stable_baselines3.ppo.policies import MlpPolicy
env = gym.make('MultiagentReaction-v0',render = False)
check_env(env)