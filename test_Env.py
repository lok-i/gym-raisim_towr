 
import gym

from time import sleep

env = gym.make('gym_raisim_towr:raisim_towr-v0',render=True)
#observation = 
env.reset()

print('action',env.action_space)
print('state',env.observation_space)

for i_episode in range(10):
	print('NEW EPISODE')
	env.reset()
	r = 0
	for t in range(200):
		sleep(0.001)
		#action = env.action_space.sample()
		action = [0, 1.09542,-2.3269]
		#action = [0,0,0,0,0,0,0,0]
		#observation, reward, done, info =
		env.step(action)
		#r+=reward
        # if done:
        #     print("Episode finished after {} timesteps".format(t+1))
        #     print('reward: '+str(r))
        #     break
env.close()