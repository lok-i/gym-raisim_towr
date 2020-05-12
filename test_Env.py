 
import gym

from time import sleep
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.image as mpimg
import pybullet as p
from ctypes import *


# independent of last z axis as it will be towr depended 
# thus only x and y matter of the final base position

no_of_steps = 100
base_init_height = 0.56
target = [2,0,0.42]

'''
towr trajectory plot for monoped env
'''
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
	plt.plot(EE_l1[1:,0],EE_l1[1:,1],EE_l1[1:,2],label = "EE")
	
	
	ax.legend()
	plt.show()


'''
towr trajectory plot for monoped env
'''
def plot_towr_anymal():
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	Base = np.array([[0,0,0]])
	EE_l1 = np.array([[0,0,0]])
	EE_l2 = np.array([[0,0,0]])
	EE_l3 = np.array([[0,0,0]])
	EE_l4 = np.array([[0,0,0]])

	for i in range(no_of_steps+1):
		base = np.array(env.towr_traj[i].base_linear)
        #four legs
		for j in range(4):
			ee  = np.array([env.towr_traj[i].ee_linear[0+3*j],env.towr_traj[i].ee_linear[1+3*j],env.towr_traj[i].ee_linear[2+3*j]])
			if j==0:
				EE_l1=np.append(EE_l1,[ee],axis=0)
			elif j==1:
				EE_l2=np.append(EE_l2,[ee],axis=0)
			elif j==2:
				EE_l3=np.append(EE_l3,[ee],axis=0)
			elif j==3:
				EE_l4=np.append(EE_l4,[ee],axis=0)
		Base=np.append(Base,[base],axis=0)
		#ax.clear()
	plt.title(label='Towr_trajectory-target:'+str(target),loc='center')
	plt.plot(Base[1:,0],Base[1:,1],Base[1:,2],label = "Base")
	plt.plot(EE_l1[1:,0],EE_l1[1:,1],EE_l1[1:,2],label = "EE_l1")
	plt.plot(EE_l2[1:,0],EE_l2[1:,1],EE_l2[1:,2],label = "EE_l2")
	plt.plot(EE_l3[1:,0],EE_l3[1:,1],EE_l3[1:,2],label = "EE_l3")
	plt.plot(EE_l4[1:,0],EE_l4[1:,1],EE_l4[1:,2],label = "EE_l4")
	plt.pause(0.01)
	ax.legend()	
	#plt.ioff()
	plt.show()
	
'''
this function shows the live tracking of towr predicted
co ordinates
'''
def plot_towr_anymal_animate():
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	Base = np.array([[0,0,0]])
	EE_l1 = np.array([[0,0,0]])
	EE_l2 = np.array([[0,0,0]])
	EE_l3 = np.array([[0,0,0]])
	EE_l4 = np.array([[0,0,0]])

	plt.title(label='Towr_trajectory-target:'+str(target),loc='center')
	for i in range(no_of_steps+1):
		base = np.array(env.towr_traj[i].base_linear)
        #four legs
		for j in range(4):
			ee  = np.array([env.towr_traj[i].ee_linear[0+3*j],env.towr_traj[i].ee_linear[1+3*j],env.towr_traj[i].ee_linear[2+3*j]])
			if j==0:
				EE_l1=np.append(EE_l1,[ee],axis=0)
			elif j==1:
				EE_l2=np.append(EE_l2,[ee],axis=0)
			elif j==2:
				EE_l3=np.append(EE_l3,[ee],axis=0)
			elif j==3:
				EE_l4=np.append(EE_l4,[ee],axis=0)
		Base=np.append(Base,[base],axis=0)
		ax.clear()
		plt.title(label='Towr_trajectory-target:'+str(target),loc='center')
		plt.plot(Base[1:,0],Base[1:,1],Base[1:,2],label = "Base")
		plt.plot(EE_l1[1:,0],EE_l1[1:,1],EE_l1[1:,2],label = "EE_l1")
		plt.plot(EE_l2[1:,0],EE_l2[1:,1],EE_l2[1:,2],label = "EE_l2")
		plt.plot(EE_l3[1:,0],EE_l3[1:,1],EE_l3[1:,2],label = "EE_l3")
		plt.plot(EE_l4[1:,0],EE_l4[1:,1],EE_l4[1:,2],label = "EE_l4")
		ax.legend()	
		plt.pause(0.01)
	
	
	plt.show()




env = gym.make('gym_raisim_towr:raisim_towr_anymal-v0',
				render=True,
				base_linear_target=target,
				no_of_steps=no_of_steps,
				base_init_height = base_init_height,
				gravity = True)



print('action_space:',env.action_space)
print('state_space:',env.observation_space)



def test_env():
	for i_episode in range(5):
		print('\n\nEPISODE_:',i_episode)
		state = env.reset()
		#print('State:\n',state)
		env.render()
		sleep(1)
		print('State:\n',state)
		for i in range(no_of_steps):
			sleep(0.01)
			#action = env.action_space.sample()
			action =[x * (2/np.pi) for x in env.towr_joint_angles[i]]
			#print("Action:",action)
			state,reward,done,_ = env.step(action)
			print('State:\n',state,"\n")
			print('Reward:\n',reward,"\n")
			#print('done:\n',done,"\n")
		
	env.close()

test_env()

#env.reset()
#sleep(5)
#env.render_towr_prediction()

