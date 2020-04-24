import os
import gym
import numpy as np
from gym import error, spaces, utils
from gym.utils import seeding
from ctypes import *
import random

cwd_path = os.getcwd()


class Raisim_towrEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self,render=True,no_of_steps=20,gravity=True):
    self.no_of_steps = no_of_steps
    self.render_status = render
    self.base_init_height = 0.54
    self.action_space = spaces.Box(low=-1.57, high=1.57,shape=(3,))
    self.observation_space = spaces.Box(low=-2, high=2, shape=(6,))
     
    self._init_raisim_dll()
    
    self.raisim_dll._init_ViSsetup(c_bool(gravity))
    self.raisim_dll._init_monopend(c_float(self.base_init_height))
    
  
  def _init_raisim_dll(self):
    self.raisim_dll = CDLL(cwd_path+"/gym_raisim_towr/envs/dll_rsc/build/libraisim_dll.so")
    self.Float_3 = c_float*3
    self.raisim_dll._sim.restype = None
    self.raisim_dll._sim.argtypes = [c_float*3,c_bool]
    self.raisim_dll._rst.restype = None
    self.raisim_dll._rst.argtypes = [c_float]
    self.raisim_dll._init_monopend.restype = None
    self.raisim_dll._init_monopend.argtypes = [c_float]
    self.raisim_dll._init_ViSsetup.restype = None
    self.raisim_dll._init_ViSsetup.argtypes = [c_bool]



  def step(self, action):
  	
  	target = self.Float_3()
  	for i in range(3):
  		target[i] = action[i]
  	self.raisim_dll._sim(target,self.render_status)
    
  def reset(self):
    b_h = c_float(self.base_init_height)
    print('reset')
    self.raisim_dll._rst(b_h)
    
  #def render(self, mode='human'):
    
  def close(self):
    print("close")
    self.raisim_dll._close();
    



# self.raisim_dll._init_ViSsetup(c_bool(True))
# self.raisim_dll._init_monopend(c_float(0.54))
# target = Float_3()
# target[0] = 0
# target[1] = 1.09542
# target[2] = -2.3269


# for i in range(500):
#   self.raisim_dll._sim(target,True)
  
#   if i%70==0 :
#     self.raisim_dll._rst(0.54)
#   for i in range(3):
#     target[i] = 1.57*random.random()
#   