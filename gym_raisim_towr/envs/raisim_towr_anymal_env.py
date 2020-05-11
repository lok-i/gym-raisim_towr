import os
import gym
import numpy as np
from gym import error, spaces, utils
from gym.utils import seeding
from ctypes import *
import random
import pybullet as p
cwd_path = os.getcwd()
'''
* in raisim the real value is in the first and py bullet its in the last
* the end effector co - ordinates should be wrt to worl frame
'''
def euler_to_quaternion(r):
    (roll, pitch, yaw) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [ qw, qx, qy, qz]


p.connect(p.DIRECT)
file_path = cwd_path +"/gym_raisim_towr/envs/dll_rsc/rsc/anymal/anymal.urdf"
p.setGravity(0,0,-10)
geo_anymal = p.loadURDF(file_path)


'''
* ik function with an error of +- 0.14 radians
* since pybullet has a recursice IK solver
  we r running stepSimulation 20 times to get 
  the angles in the final target ee location.
  (values 20 was consistent to reach the
  target location any where inside the kinematic box of towr)
'''

def pybullet_ik(base_pos,base_quat,ee1,ee2,ee3,ee4):
    base_pos = list(base_pos)
    base_quat = [base_quat[1],base_quat[2],base_quat[3],base_quat[0]]
    angles_12=[]
    for i in range(20):
        p.resetBasePositionAndOrientation(geo_anymal, base_pos,base_quat)
        leg_LF = p.calculateInverseKinematics(geo_anymal,5 ,ee1)
        leg_RF = p.calculateInverseKinematics(geo_anymal,10,ee2)
        leg_LH = p.calculateInverseKinematics(geo_anymal,15,ee3)
        leg_RH = p.calculateInverseKinematics(geo_anymal,20,ee4)
        p.stepSimulation()
        angles_12 =leg_LF[0:3]+leg_RF[3:6]+leg_LH[6:9]+leg_RH[9:12]
    return(angles_12)

class Trajectory_data(Structure):
    _fields_ = [
            ('base_linear', c_float*3),
            ('base_angular',c_float*3),
            ('ee_linear', c_float*12),
            ('ee_force', c_float*12),
           
            
            ]


class Raisim_towr_anymalEnv(gym.Env):
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
    self.action_space = spaces.Box(low=-1.0, high=1.0,shape=(12,))
    self.observation_space = spaces.Box(low=-1.0, high=1.0,shape=(44,))
    self.ith_step = -1

    #limits set to normalize the state space and action space
    self.joint_w_limit = 100
    self.joint_tau_limit = 100
    # joint limits according to the kinematic box in towr
    self.joint_angle_ul =[ 0.1331395 ,1.68423158,-0.38000404,
                           0.49791087,1.68423119,-0.38000466,
                           0.13313961,-0.12118592,1.67963936, 
                           0.49791083,-0.12118621,1.67963958]
   
    self.joint_angle_ll =[-0.49791066,0.12118728,-1.67963886,
                          -0.13313934,0.12118650,-1.67963938,
                          -0.4979109,-1.68423165,0.38000445, 
                          -0.13313932,-1.68423096,0.3800044 ]

    
    
    #data type to take care of handling arrays from the c dll's
    self.c_Float_3 = c_float*3

    #array to save the towr predicted trajectory
    self.traj_array = Trajectory_data*(self.no_of_steps+1)
    self.towr_traj = self.traj_array()

    #array to get the current raisim state from raisim.dll
    self.current_raisim_state = (c_float*43)() 

    #target array to be sent to towr
    self.base_linear_target = self.c_Float_3()
    for i in range(3):
      self.base_linear_target[i] = base_linear_target[i]

    #function calls to initialize raisim.dll and towr.dll function parameters
    self._init_raisim_dll()
    self._init_towr_dll()
    
    #function calls,to calculate towr trajectory
    self.towr_dll.Trajectory(self.towr_traj,c_float(self.base_init_height),self.base_linear_target,c_int(self.no_of_steps))    
    #list to keep the joint angles for the corresponding towr predicted ee_cordinates
    self.towr_joint_angles = []
    for i in range(len(self.towr_traj)):
        towr_quat = euler_to_quaternion(self.towr_traj[i].base_angular)
        self.towr_joint_angles.append(list(pybullet_ik(self.towr_traj[i].base_linear, towr_quat,
                                                       self.towr_traj[i].ee_linear[0:3],
                                                       self.towr_traj[i].ee_linear[3:6],
                                                       self.towr_traj[i].ee_linear[6:9],
                                                       self.towr_traj[i].ee_linear[9:12])))

  
    #function calls,to initialize visual and world elements in raisim
    self.raisim_dll._init_ViSsetup(c_bool(gravity))
    self.raisim_dll._init_anymal(c_float(self.base_init_height))


    
  



  
  def _init_raisim_dll(self):
    self.raisim_dll = CDLL(cwd_path+"/gym_raisim_towr/envs/dll_rsc/build/libraisim_anymal_dll.so")
    self.raisim_dll._sim.restype = None
    self.raisim_dll._sim.argtypes = [c_float*12,c_bool]# 12 angles
    self.raisim_dll._rst.restype = None
    self.raisim_dll._rst.argtypes = [c_float]
    self.raisim_dll._init_anymal.restype = None
    self.raisim_dll._init_anymal.argtypes = [c_float]
    self.raisim_dll._init_ViSsetup.restype = None
    self.raisim_dll._init_ViSsetup.argtypes = [c_bool]
    self.raisim_dll.get_state.restype = None
    self.raisim_dll.get_state.argtype = [c_float*43]

 



  def _init_towr_dll(self):
    self.towr_dll = CDLL(cwd_path+"/gym_raisim_towr/envs/dll_rsc/build/libtowr_anymal_dll.so")
    self.towr_dll.Trajectory.restype = None
    self.towr_dll.Trajectory.argtypes = [ Trajectory_data*(self.no_of_steps +1 ) ,c_float ,c_float*3,c_int]

#to print the entire towr traj
  def print_towr_traj(self):
    for i in range(self.no_of_steps +1 ):
      print("i :",i," time :",i*2.0/self.no_of_steps)
      print("base_linear:",self.towr_traj[i].base_linear[0],"\t",self.towr_traj[i].base_linear[1],"\t",self.towr_traj[i].base_linear[2])
      print("base_angular:",self.towr_traj[i].base_angular[0],"\t",self.towr_traj[i].base_angular[1],"\t",self.towr_traj[i].base_angular[2])
      for j in range(4):
        print("leg_no:",j)
        print("ee_linear_leg:",self.towr_traj[i].ee_linear[0+3*j],"\t",self.towr_traj[i].ee_linear[1+3*j],"\t",self.towr_traj[i].ee_linear[2+3*j])
        print("ee_force_leg:", self.towr_traj[i].ee_force[0+3*j],"\t",self.towr_traj[i].ee_force[1+3*j],"\t",self.towr_traj[i].ee_force[2+3*j])


  def scale_action_angle(self,input_array,action_to_angle = True):
    output = [0,0,0,
              0,0,0,
              0,0,0,
              0,0,0]

    for i in range(12):
        ll_plus_ul  = self.joint_angle_ll[i]+self.joint_angle_ul[i]
        ll_minus_ul = self.joint_angle_ll[i]-self.joint_angle_ul[i]
        
        if(action_to_angle):
            #output - angle,input - action
            output[i] = (ll_plus_ul - (ll_minus_ul*input_array[i]))*0.5
    
        else:
            #output - action,input - angle
            output[i] = (ll_plus_ul - (2.0*input_array[i]))/ll_minus_ul
    
    return output

  def step(self, action):
    #action is 4x3 angles
    done = False
    self.ith_step +=1
    

    #convert action to angles
    action = self.scale_action_angle(action,action_to_angle = True)
    
    target_angle = (c_float*12)()
    

    for i in range(12):
      target_angle[i] = action[i]
    
    # every action update happens after 5 raisim_simulation steps
    # cant find the optimal value though
    for i in range(5):
    #target angle - send pd targets 
        self.raisim_dll._sim(target_angle,self.render_status)
    #saves root_linear,root_qaut,joint_angles,joint_velocities,joint_torques from raisim
        self.raisim_dll.get_state(self.current_raisim_state)
    '''
    State Space to agent:-
    [base_quat[4],genralized_joint_angles[12],genralized_joint_velocities[12],genralized_joint_forces[12],
     goal_base_quat(t+1)[4]]
    '''
    state = np.zeros(44)
  
    if self.ith_step ==self.no_of_steps:
      done = True

    #Clipping and Normalizing the state space and initializing
    else:
      towr_base_quat = euler_to_quaternion(self.towr_traj[self.ith_step+1].base_angular)
      
      #current_base_quaternions
      state[0:4] =  self.current_raisim_state[3:7]
      #current_joint_angles
      state[4:16] = self.scale_action_angle(self.current_raisim_state[7:19],action_to_angle =False)
      #current_joint_wel_and_torques
      for i in range(24):
        state[i+16]=self.current_raisim_state[i+19] #skips the base co ordinates
        if(i<12):
            if(state[i]>self.joint_w_limit): #clip_joint_velocity
                state[i] =self.joint_w_limit
            elif(state[i]<-self.joint_w_limit):
                state[i] =-self.joint_w_limit
                state[i] = state[i]/self.joint_w_limit# normalize j_w
        else:
            if(state[i]>self.joint_tau_limit): #clip _join_tau
              state[i] =self.joint_tau_limit
            elif(state[i]<-self.joint_tau_limit):
              state[i] =-self.joint_tau_limit
            state[i] = state[i]/self.joint_tau_limit# normalize j_tau
      #next_goal_base_quaternions
      state[40:44]=towr_base_quat #quat alredy normalized


    return state ,self.calc_reward(self.ith_step),done,{}

    
  def reset(self):
    #b_h - the bot will be rest at 0,0,b_h with the default stance joint angles
    b_h = c_float(self.base_init_height)
    print('reset')
    
    #reset ithstep
    self.ith_step = -1
    self.raisim_dll._rst(b_h)

    #display the reset if required
    if(self.render_status):
      self.render()

   
    nominal_stance_angles = [-0.13535572325766684, 0.9743811894650634, -1.332805236697449, 
                              0.1353557332372563, 0.9743812194285772, -1.3328052761745393, 
                             -0.13535573056942238, -0.9743812218456691, 1.3328052791030047, 
                              0.13535572529984888, -0.9743811832125653, 1.3328052297175719]

    nominal_stance_action= self.scale_action_angle(nominal_stance_angles,action_to_angle = False)

    
    state,r,d,_ = self.step(nominal_stance_action)
    return state
    
  def calc_reward(self, ith_step):
    #weights for each part
    w_base_pos_quat = 0.76923 
    w_joint_angles  = 0.23076
    #w_base_height   = -1
    
    #making towr data as np arrays
    towr_pos = np.array(self.towr_traj[ith_step].base_linear)
    towr_quat = np.array(euler_to_quaternion(self.towr_traj[ith_step].base_angular))
    towr_joint_angles = np.array(self.towr_joint_angles[ith_step])
    
    #making raisim data as np arrays
    raisim_pos = np.array(self.current_raisim_state[0:3])
    raisim_quat = np.array(self.current_raisim_state[3:7])
    raisim_joint_angles = np.array(self.current_raisim_state[7:19])
    #raisim_base_z = raisim_pos[2] 

    #calculating the mse for each part 
    base_pos_diff_mse = np.mean(np.square(np.subtract(towr_pos,raisim_pos)))
    base_quat_diff_mse = np.mean(np.square(np.subtract(towr_quat,raisim_quat)))
    joint_angle_diff_mse = np.square(np.subtract(towr_joint_angles,raisim_joint_angles))
    joint_angle_diff_mse = np.mean(joint_angle_diff_mse[0:3])+np.mean(joint_angle_diff_mse[3:6])+np.mean(joint_angle_diff_mse[6:9])+np.mean(joint_angle_diff_mse[9:12])
    #height_penalty = -1 if raisim_base_z < 0.52 else 0 
    #final reward:-        
    
    reward = w_base_pos_quat*np.exp(-20*base_pos_diff_mse + -10*base_quat_diff_mse)+ w_joint_angles* np.exp(-5*joint_angle_diff_mse)
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