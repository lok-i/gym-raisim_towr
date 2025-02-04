import os
import gym
import numpy as np
from gym import error, spaces, utils
from gym.utils import seeding
from ctypes import *
import random

cwd_path = os.getcwd()

def euler_to_quaternion(r):
    (roll, pitch, yaw) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [ qw, qx, qy, qz]

class Trajectory_data(Structure):
    _fields_ = [
            ('base_linear', c_float*3),
            ('base_angular',c_float*3),
            ('ee_linear', c_float*3),
            ('ee_force', c_float*3),
            ('joint_angles',c_float*3)
            
            ]


class Raisim_towrEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self,base_linear_target,base_init_height = 0.54,render=True,no_of_steps=20,gravity=True):
    # also the no of sampled from towr initially == no of steps per episode
    self.no_of_steps = no_of_steps 
    self.render_status = render
    self.base_init_height = base_init_height
    '''
    ith_step - keeps count of the index of the towr traj 
    that needs to be compared with the curret step of raisim
    simulation to calclate reward as well as give the expected 
    goal state in the state space and gets reset once the episode 
    ends ie when ithstep == no_of_steps
    '''
    self.action_space = spaces.Box(low=-1.0, high=1.0,shape=(3,))
    self.observation_space = spaces.Box(low=-1.0, high=1.0,shape=(17,))
    self.ith_step = -1

    #limits set to normalize the state space and action space
    self.joint_w_limit = 5
    self.joint_tau_limit = 10
    self.joint_a_limit = 1
    
    #data type to take care of handling arrays from the c dll's
    self.c_Float_3 = c_float*3

    #array to save the towr predicted trajectory
    self.traj_array = Trajectory_data*(self.no_of_steps+1)
    self.towr_traj = self.traj_array()

    #array to get the current raisim state from raisim.dll
    self.current_raisim_state = (c_float*16)()

    #target array to be sent to towr
    self.base_linear_target = self.c_Float_3()
    for i in range(3):
      self.base_linear_target[i] = base_linear_target[i]

    #function calls to initialize raisim.dll and towr.dll function parameters
    self._init_raisim_dll()
    self._init_towr_dll()
    
    #function calls,to calculate towr trajectory
    self.towr_dll.Trajectory(self.towr_traj,c_float(self.base_init_height),self.base_linear_target,c_int(self.no_of_steps))    
    
    #function calls,to initialize visual and world elements in raisim
    self.raisim_dll._init_ViSsetup(c_bool(gravity))
    self.raisim_dll._init_monopend(c_float(self.base_init_height))
    
  
  def _init_raisim_dll(self):
    self.raisim_dll = CDLL(cwd_path+"/gym_raisim_towr/envs/dll_rsc/build/libraisim_dll.so")
    self.raisim_dll._sim.restype = None
    self.raisim_dll._sim.argtypes = [c_float*3,c_bool]
    self.raisim_dll._rst.restype = None
    self.raisim_dll._rst.argtypes = [c_float]
    self.raisim_dll._init_monopend.restype = None
    self.raisim_dll._init_monopend.argtypes = [c_float]
    self.raisim_dll._init_ViSsetup.restype = None
    self.raisim_dll._init_ViSsetup.argtypes = [c_bool]
    self.raisim_dll.get_state.restype = None
    self.raisim_dll.get_state.argtype = [c_float*16]




  def _init_towr_dll(self):
    self.towr_dll = CDLL(cwd_path+"/gym_raisim_towr/envs/dll_rsc/build/libtowr_dll.so")
    self.towr_dll.Trajectory.restype = None
    self.towr_dll.Trajectory.argtypes = [ Trajectory_data*(self.no_of_steps +1 ) ,c_float ,c_float*3,c_int]


  def print_towr_traj(self):
    for i in range(self.no_of_steps +1 ):
      print("i :",i," time :",i*2.0/self.no_of_steps)
      print("base_linear:",self.towr_traj[i].base_linear[0],"\t",self.towr_traj[i].base_linear[1],"\t",self.towr_traj[i].base_linear[2])
      print("base_angular:",self.towr_traj[i].base_angular[0],"\t",self.towr_traj[i].base_angular[1],"\t",self.towr_traj[i].base_angular[2])
      print("ee_linear",self.towr_traj[i].ee_linear[0],"\t",self.towr_traj[i].ee_linear[1],"\t",self.towr_traj[i].ee_linear[2])
      print("ee_force:",self.towr_traj[i].ee_force[0],"\t",self.towr_traj[i].ee_force[1],"\t",self.towr_traj[i].ee_force[2])
      print("joint_angles:",self.towr_traj[i].joint_angles[0],"\t",self.towr_traj[i].joint_angles[1],"\t",self.towr_traj[i].joint_angles[2])




  def step(self, action):
    done = False
    self.ith_step +=1

    #scalling up the output to angles
    action = self.joint_a_limit*np.array(action)
    target_angle = self.c_Float_3()
    for i in range(3):
      target_angle[i] = action[i]

    #target angle - send pd targets 
      
    self.raisim_dll._sim(target_angle,self.render_status)
    #saves root_linear,root_qaut,joint_angles,joint_velocities,joint_torques from raisim
    self.raisim_dll.get_state(self.current_raisim_state)
    
    '''
    State Space to agent:-
    [base_quat[4],genralized_joint_angles[3],genralized_joint_velocities[3],genralized_joint_forces[3],
     goal_base_quat(t+1)[4]]
    '''
    state = np.zeros(17)

    target_reached = 1 if self.current_raisim_state[0] == self.target[0] and self.current_raisim_state[1] == self.target[1] else 0 
  
    if self.ith_step ==self.no_of_steps or target_reached :
      done = True

    #Clipping and Normalizing the state space and initializing
    else:
      root_quat = euler_to_quaternion(self.towr_traj[self.ith_step+1].base_angular)
      for i in range(13):
        
        state[i]=self.current_raisim_state[3+i] #skips the base co ordinates
        #0 to 3 quat alredy normalized
        if(i>3 and i<=6):
          state[i] = state[i]/(np.pi*0.5) #normalize angles
        else:
          if(i>6 and i<=9):
            if(state[i]>self.joint_w_limit): #clip_joint_velocity
              state[i] =self.joint_w_limit
            elif(state[i]<-self.joint_w_limit):
              state[i] =-self.joint_w_limit

            state[i] = state[i]/self.joint_w_limit# normalize j_w
          elif(i>9):
            if(state[i]>self.joint_tau_limit): #clip _join_tau
              state[i] =self.joint_tau_limit
            elif(state[i]<-self.joint_tau_limit):
              state[i] =-self.joint_tau_limit
            state[i] = state[i]/self.joint_tau_limit# normalize j_tau

      for i in range(3):
        #goal orienation quaternion of the next step
        state[13+i]=0#root_quat[i] #quat alredy normalized
      
  

    return state,self.calc_reward(self.ith_step),done,{}

    
  def reset(self):
    #b_h - the bot will be rest at 0,0,b_h with the default joint angles
    b_h = c_float(self.base_init_height)
    print('reset')
    #reset ithstep
    self.ith_step = -1
    self.raisim_dll._rst(b_h)
    #display the reset if required
    if(self.render_status):
      self.render()
    '''
    to return a state the state space, @ t=0
    base position angles:-
    '''
    angle_1 = 2*1.09542/np.pi
    angle_2 = 2*-2.3269/np.pi
    state,r,d,_ = self.step([0,angle_1,angle_2])
    return state
    
  def calc_reward(self, ith_step):
    #base position,base orientation and joint angles from towr is considered

    #weights for each part
    w_base_pos_quat = 0 #0.15
    w_j_a           = 0 #0.5
    w_balance       = -1

    #making them np arrays
    towr_pos = np.array(self.towr_traj[ith_step].base_linear)
    towr_quat = np.array(euler_to_quaternion(self.towr_traj[ith_step].base_angular))
    towr_j_a  = np.array(self.towr_traj[ith_step].joint_angles) 
    
    raisim_pos = np.array([self.current_raisim_state[0],self.current_raisim_state[1],self.current_raisim_state[2]])
    raisim_quat = np.array([self.current_raisim_state[3],self.current_raisim_state[4],self.current_raisim_state[5],self.current_raisim_state[6]])
    raisim_j_a  = np.array([self.current_raisim_state[7],self.current_raisim_state[8],self.current_raisim_state[9]])
    
    #calculating the mse for each part 
    pos_diff_mse = np.mean(np.square(np.subtract(towr_pos,raisim_pos)))
    quat_diff_mse = np.mean(np.square(np.subtract(towr_quat,raisim_quat)))
    j_a_mse       = np.mean(np.square(np.subtract(towr_j_a,raisim_j_a)))
    level = -1
    if raisim_pos[2]<0.5:
        level = 1
    

    #final rewar:-
    reward =w_base_pos_quat*np.exp(-20*pos_diff_mse + -10*quat_diff_mse)+w_j_a * np.exp(-5*j_a_mse) + w_balance*level

    # print("\n\ntowr_pos_:",towr_pos,'\t',"raisim_pos_:",raisim_pos,'\n')
    # print("towr_quat_:",towr_quat,'\t',"raisim_quat_:",raisim_quat,'\n')
    # print("pose_mse_:",pos_diff_mse,'\t',"quat_mse_:",quat_diff_mse,"\n")
    # print("reward_:",reward)

    return reward


  def close(self):
    print("close")
    self.raisim_dll._close()

    
  def render(self):
    #to render a single frame
    self.raisim_dll._render()




    ''' 
    r(t) = w_base_pos * r_base_pos(t) + 
         w_base_lin_vel * r_base_lin_vel(t) + 
         w_end_eff_co *  r_end_eff_co(t) + 
         w_base_rpy *r_base_rpy(t) +
         w_base_angulat_vel * r_base_angulat_vel
      
    '''