import gym
import matplotlib.pyplot as plt
import os
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2
cwd = os.getcwd()


no_of_episodes = 700
no_of_steps_per_epi = 200

agent_name = 'test-('+str(no_of_episodes)+')-('+str(no_of_steps_per_epi)+')'

# independent of last z axis as it will be towr depended 
# thus only x and y matter of the final base position
target = [1,1,0.54]
env = gym.make('gym_raisim_towr:raisim_towr-v0',render=False,
				base_linear_target=target,no_of_steps=no_of_steps_per_epi,
				base_init_height = 0.54)
env = DummyVecEnv([lambda: env]) 



model = PPO2(MlpPolicy, env, n_steps=no_of_steps_per_epi, verbose=1,tensorboard_log=cwd+'/tf_logs/')
env.reset()

model.learn(total_timesteps=no_of_episodes*no_of_steps_per_epi,log_interval=1,tb_log_name=agent_name)
model.save(save_path=cwd+'/models/'+agent_name+'.zip')


