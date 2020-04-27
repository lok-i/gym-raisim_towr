import gym
import matplotlib.pyplot as plt
import os
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2
from time import sleep

cwd = os.getcwd()
''' the inner loop should
run no_of_steps  
and towr traj will contain 
no_of_steps +1 element (including intial state)
hence querry from 1 to no_of_steps from the
towr_traj array 
'''
no_of_episodes = 700
no_of_steps_per_epi = 200

# independent of last z axis as it will be towr depended 
# thus only x and y matter of the final base position
target = [1,1,0.54]
env = gym.make('gym_raisim_towr:raisim_towr-v0',render=True,base_linear_target=target,no_of_steps=no_of_steps_per_epi,base_init_height = 0.54)

agent_name = 'test-('+str(no_of_episodes)+')-('+str(no_of_steps_per_epi)+')'

#function to print traj calcuated by towr
#env.print_towr_traj()

print('action_space:',env.action_space)
print('state_space:',env.observation_space)
model = PPO2.load(cwd+'/models/'+agent_name+'.zip')

for i_episode in range(5):
	print('\n\nEPISODE_:',i_episode)
	state = env.reset()

	sleep(2)
	r = 0
	for t in range(no_of_steps_per_epi):
		sleep(0.001)
		action = model.predict(state)[0]
		state,reward,done,_ = env.step(action)
		
		if done == True:
			break
		r+=reward
		#print('State:\n',state)
		#print('Reward:\n',reward)
		#print('done:\n',done)
	print('Reward_sum:',r)
		
env.close()