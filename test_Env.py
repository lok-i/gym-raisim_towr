 
import gym

from time import sleep
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def stardize(a):
	a= (a-np.mean(a))/np.std(a)
	return a

# independent of last z axis as it will be towr depended 
# thus only x and y matter of the final base position

no_of_steps = 1000
base_init_height = 0.54
target = [10,20,0.54]

def plot_towr():
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	Base = np.array([[0,0,0]])
	EE = np.array([[0,0,0]])
	for i in range(no_of_steps):
		base = np.array(env.towr_traj[i].base_linear)
		ee  = np.array(env.towr_traj[i].ee_linear)
		Base=np.append(Base,[base],axis=0)
		EE=np.append(EE,[ee],axis=0)
	
	plt.title(label='Towr_trajectory',loc='center')
	plt.plot(Base[1:,0],Base[1:,1],Base[1:,2],label = "Base")
	plt.plot(EE[1:,0],EE[1:,1],EE[1:,2],label = "EE")
	
	
	ax.legend()
	plt.show()

env = gym.make('gym_raisim_towr:raisim_towr-v0',
				render=True,
				base_linear_target=target,
				no_of_steps=no_of_steps,
				base_init_height = base_init_height)


#function to print traj calcuated by towr
#env.print_towr_traj()

print('action_space:',env.action_space)
print('state_space:',env.observation_space)


def test_env():
	for i_episode in range(1):
		print('\n\nEPISODE_:',i_episode)
		env.reset()
		sleep(1)
		for t in range(no_of_steps):
			sleep(0.01)
			action = env.action_space.sample()
			state,reward,done,_ = env.step(action)
			print('State:\n',state)
			print('Reward:\n',reward)
			print('done:\n',done)
		
	env.close()

plot_towr()


