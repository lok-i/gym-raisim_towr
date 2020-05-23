import gym
import matplotlib.pyplot as plt
import os
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2
from test_params import *



cwd = os.getcwd()
'''
model name ,target,no of epidodes,
no of steps are all defined in
test_params.py file
'''

env = gym.make('gym_raisim_towr:raisim_towr_anymal-v0',render=False,
				base_linear_target=target,no_of_steps=no_of_steps_per_epi,
				base_init_pos = base_init_pos)
env = DummyVecEnv([lambda: env]) 


model = PPO2(MlpPolicy,
			 env,
			 gamma=0.95,
			 learning_rate = 0.000001,
			 n_steps=no_of_steps_per_epi,
			 noptepochs=10,
			 nminibatches=5,
			 verbose=1,
			 tensorboard_log=cwd+'/tf_logs/')
env.reset()

model.learn(total_timesteps=no_of_episodes*no_of_steps_per_epi,
				 log_interval=1,
				 tb_log_name=agent_name)
model.save(save_path=cwd+'/models/'+agent_name+'.zip')


