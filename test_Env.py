 
import gym

from time import sleep
import numpy as np
''' the inner loop should
run no_of_steps  
and towr traj will contain 
no_of_steps +1 element (including intial state)
hence querry from 1 to no_of_steps from the
towr_traj array 
'''

def stardize(a):
	a= (a-np.mean(a))/np.std(a)
	return a
no_of_steps = 50

# independent of last z axis as it will be towr depended 
# thus only x and y matter of the final base position
target = [1,1,0.54]


env = gym.make('gym_raisim_towr:raisim_towr-v0',
				render=True,
				base_linear_target=target,
				no_of_steps=no_of_steps,
				base_init_height = 0.54)

#function to print traj calcuated by towr
#env.print_towr_traj()

print('action_space:',env.action_space)
print('state_space:',env.observation_space)


for i_episode in range(1):
	print('\n\nEPISODE_:',i_episode)
	print(env.reset())

	sleep(1)
	for t in range(no_of_steps):
		sleep(0.01)

		action = env.action_space.sample()

		#action = [0, 1.09542,-2.3269]
		'''
		state , reward,done,{} and 

		t+1 is for query the same position as raisim 
		as intial state as well is stored in towr_traj'''
		state,reward,done,_ = env.step(action)
		# print('State:\n',state)
		# print('Reward:\n',reward)
		# print('done:\n',done)
		
env.close()

#a = [20,30000,10000]
#print(stardize(a))