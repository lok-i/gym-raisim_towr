import gym
import matplotlib.pyplot as plt
import os
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.common.env_checker import check_env
from stable_baselines import PPO2
import time 
from test_params import *
cwd = os.getcwd()

'''
model name ,target,no of epidodes,
no of steps are all defined in
test_params.py file
'''

env = gym.make('gym_raisim_towr:raisim_towr_anymal-v0',
				render=True,
				base_linear_target=target,
				no_of_steps=no_of_steps_per_epi,
				base_init_height = base_init_height)

#check_env(env, warn=True)

#function to print traj calcuated by towr
#env.print_towr_traj()

print('action_space:',env.action_space)
print('state_space:',env.observation_space)
model = PPO2.load(cwd+'/models/'+agent_name+'.zip')

for i_episode in range(5):
	print('\n\nEPISODE_:',i_episode)
	state = env.reset()

	time.sleep(10)
	r = 0

	for t in range(no_of_steps_per_epi):
		time.sleep(0.01)
		action,_ = model.predict(state)
		
	
		state,reward,done,_ = env.step(action)
		

		
		if done == True:
			break
		r+=reward
		# print('State:\n',state)
		# print('Reward:\n',reward)
		# print('done:\n',done)
	print('Reward_sum:',r)
		
env.close()