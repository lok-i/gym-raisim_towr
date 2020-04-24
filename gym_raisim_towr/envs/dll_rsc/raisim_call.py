from ctypes import *
import random
raisim_dll = CDLL("./build/libraisim_dll.so")
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
raisim_dll._init_monopend(c_float(base_init_height))

Float_3 = c_float*3
target = Float_3()
raisim_dll._sim.restype = None
raisim_dll._sim.argtypes = [c_float*3,c_bool]

raisim_dll._rst.restype = None

raisim_dll._rst.argtypes = [c_float]
target[0] = 0
target[1] = 1.09542
target[2] = -2.3269

for i in range(500):
    raisim_dll._sim(target,True)
    
    if i%70==0 :
        raisim_dll._rst(0.54)
    for i in range(3):
        target[i] = 1.57*random.random()
	
