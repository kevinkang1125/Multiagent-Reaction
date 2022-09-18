from cmath import log
from statistics import mean
import gym
import os
from stable_baselines3.common.env_checker import check_env
import multiagent_reaction
#from Truck-Trailer import truck_trailer
from stable_baselines3 import PPO
from stable_baselines3.ppo.policies import MlpPolicy
import numpy as np
import os
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.results_plotter import load_results,ts2xy
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.callbacks import CheckpointCallback

class SaveOnBestTrainingRewardCallback(BaseCallback):
    def __init__(self, check_freq,log_dir,verbose=1):
        super(SaveOnBestTrainingRewardCallback,self).__init__(verbose)
        self.check_freq = check_freq
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir,"best_model")
        self.best_mean_reward = -np.inf
    def _init_callback(self) -> None:
        if self.save_path is not None:
            os.makedirs(self.save_path,exist_ok=True)
    
    def _on_step(self) -> bool:
        if self.n_calls % self.check_freq == 0:
            x,y = ts2xy(load_results(self.log_dir),'timesteps')
            if len(x) > 0:
                mean_reward = np.mean(y[-100:])
                
                if mean_reward>self.best_mean_reward:
                    self.best_mean_reward = mean_reward
                    self.model.save(self.save_path)
log_dir = "./log/"
os.makedirs(log_dir,exist_ok=True)
save_best_model = SaveOnBestTrainingRewardCallback(check_freq=20000,log_dir=log_dir)


checkpoint_callback= CheckpointCallback(save_freq=200000,save_path='./log/',name_prefix='rl_model')
env = gym.make('MultiagentReaction-v0',render = False)
env = Monitor(env,log_dir)
#env = env(render = False)
model = PPO(MlpPolicy,env,verbose=1,tensorboard_log="./log/",gamma=0.95)
model.learn(total_timesteps=200000,callback=checkpoint_callback,tb_log_name="pre_mature")
model.learn(total_timesteps=10000000,callback=[checkpoint_callback,save_best_model],tb_log_name="steady",reset_num_timesteps=False)
#check_env(env) 