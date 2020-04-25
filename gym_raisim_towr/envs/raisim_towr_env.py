import os
import gym
import numpy as np
from gym import error, spaces, utils
from gym.utils import seeding
from ctypes import *
import random

cwd_path = os.getcwd()

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
    self.no_of_steps = no_of_steps # also the no of sampled from towr initially
    self.render_status = render
    self.base_init_height = base_init_height

    self.action_space = spaces.Box(low=-1.57, high=1.57,shape=(3,))
    self.observation_space = spaces.Box(low=-2, high=2, shape=(6,))
    
    self.c_Float_3 = c_float*3
    self.traj_array = Trajectory_data*(self.no_of_steps+1)
    
    self.towr_traj = self.traj_array()
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
  	
  	target_angle = self.c_Float_3()
  	for i in range(3):
  		target_angle[i] = action[i]
  	self.raisim_dll._sim(target_angle,self.render_status)
    
  def reset(self):
    b_h = c_float(self.base_init_height)
    print('reset')
    self.raisim_dll._rst(b_h)
    
  #def render(self, mode='human'):
    
  def close(self):
    print("close")
    self.raisim_dll._close();
    

