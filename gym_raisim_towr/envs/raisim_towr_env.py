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
            ('ee_force', c_float*3)
            
            ]


class Raisim_towrEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self,base_linear_target,base_init_height = 0.54,render=True,no_of_steps=20,gravity=True):
    self.no_of_steps = no_of_steps # also the no of sampled from towr initially == no of steps per episode
    self.render_status = render
    self.base_init_height = base_init_height

    self.action_space = spaces.Box(low=-1.57, high=1.57,shape=(3,))

    #cant set low , high as the diff of position is present
    self.observation_space = spaces.Box(low=-2, high=2, shape=(16,))
    self.ith_step = -1
    
    self.c_Float_3 = c_float*3
    self.traj_array = Trajectory_data*(self.no_of_steps+1)
    
    self.towr_traj = self.traj_array()
    self.current_raisim_state = (c_float*16)()
    self.base_linear_target = self.c_Float_3()
    for i in range(3):
      self.base_linear_target[i] = base_linear_target[i]


    self._init_raisim_dll()
    self._init_towr_dll()
    
    self.towr_dll.Trajectory(self.towr_traj,c_float(self.base_init_height),self.base_linear_target,c_int(self.no_of_steps))    
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



  def step(self, action):
    done = False
    self.ith_step +=1
    target_angle = self.c_Float_3()
    for i in range(3):
      target_angle[i] = action[i]
    #target angle _ pd targets   
    self.raisim_dll._sim(target_angle,self.render_status)
    self.raisim_dll.get_state(self.current_raisim_state)
    
    # goal - curren base position
    #print('current_raisim:(',np.array(self.current_raisim_state),")\ntowr[",self.ith_step,"](",np.array(self.towr_traj[self.ith_step].base_linear),')')
    for i in range(3):
      self.current_raisim_state[i] = self.towr_traj[self.ith_step].base_linear[i] - self.current_raisim_state[i]

    #print("\nState sent out:",np.array(self.current_raisim_state),"\n\n")
    '''
    State:-{goal_base_pos[3] - base_pos[3],base_quat[4],genralized_joint_angles[3],genralized_joint_velocities[3],genralized_joint_forces[3]}
    '''
    if self.ith_step ==self.no_of_steps:
      done = True
    return np.array(self.current_raisim_state),self.calc_reward(self.ith_step),done,{}

    
  def reset(self):
    b_h = c_float(self.base_init_height)
    print('reset')
    self.ith_step = -1


    self.raisim_dll._rst(b_h)
    if(self.render_status):
      self.render()
    state,r,d,_ = self.step([0,1.09542,-2.3269])#base position angles
    return state
    
  def calc_reward(self, ith_step):
    #only base pose and orientation is considered

    w_base_pos_quat = 0.15
    towr_pose = np.array(self.towr_traj[ith_step].base_linear)
    towr_quat = np.array(euler_to_quaternion(self.towr_traj[ith_step].base_angular)) 
    
    raisim_pose = np.array([self.current_raisim_state[0],self.current_raisim_state[1],self.current_raisim_state[2]])
    raisim_quat = np.array([self.current_raisim_state[3],self.current_raisim_state[4],self.current_raisim_state[5],self.current_raisim_state[6]])
    
    pose_diff_mse = np.mean(np.square(np.subtract(towr_pose,raisim_pose)))
    quat_diff_mse = np.mean(np.square(np.subtract(towr_quat,raisim_quat)))
  
    reward =w_base_pos_quat*np.exp(-20*pose_diff_mse + -10*quat_diff_mse)

    # print("\n\ntowr_pose_:",towr_pose,'\t',"raisim_pose_:",raisim_pose,'\n')
    # print("towr_quat_:",towr_quat,'\t',"raisim_quat_:",raisim_quat,'\n')
    # print("pose_mse_:",pose_diff_mse,'\t',"quat_mse_:",quat_diff_mse,"\n")
    # print("reward_:",reward)

    return reward


  def close(self):
    print("close")
    self.raisim_dll._close()

    
  def render(self):
    self.raisim_dll._render()




    ''' 
    r(t) = w_base_pos * r_base_pos(t) + 
         w_base_lin_vel * r_base_lin_vel(t) + 
         w_end_eff_co *  r_end_eff_co(t) + 
         w_base_rpy *r_base_rpy(t) +
         w_base_angulat_vel * r_base_angulat_vel
      
    '''