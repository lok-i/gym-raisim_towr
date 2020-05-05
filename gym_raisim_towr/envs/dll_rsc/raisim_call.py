from ctypes import *
import random
import numpy as np
raisim_dll = CDLL("./build/libraisim_anymal_dll.so")
#raisim_dll.Trajectory()
#print(raisim_dll)
base_init_height = 0.54

no_of_samples = 10

float_array_3 = c_float*3
class Trajectory_data(Structure):
    _fields_ = [
            ('base_linear', c_float*3),
            ('base_angular',c_float*3),
            ('ee_linear', c_float*3),
            ('ee_force', c_float*3)
            
            ]


raisim_dll._init_ViSsetup(c_bool(True))
raisim_dll._init_anymal(c_float(base_init_height))

Float_12 = c_float*12
target = Float_12()

raisim_dll._sim.restype = None
raisim_dll._sim.argtypes = [c_float*12,c_bool]

raisim_dll._rst.restype = None

raisim_dll._rst.argtypes = [c_float]

raisim_dll.get_state.restype = None

raisim_dll.get_state.argtype = [c_float*43]

state = (c_float*43)()

for i in range(12):
    target[i] = 0#1.57*random.random()

for i in range(500):
    raisim_dll._sim(target,True)
    raisim_dll.get_state(state)
    for i in range(43):
        print('state_[',i,'] :',state[i])  
    if i%70==0 :
        raisim_dll._rst(0.54)
    for i in range(12):
        target[i] = 0 #1.57*random.random()
	
# def euler_to_quaternion(r):
#     (yaw, pitch, roll) = (r[0], r[1], r[2])
#     qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
#     qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
#     qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
#     return [qx, qy, qz, qw]


# r = [0,0,0]
# print(euler_to_quaternion(r))