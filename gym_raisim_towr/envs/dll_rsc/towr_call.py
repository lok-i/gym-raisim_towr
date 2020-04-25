from ctypes import *

lib = CDLL("./build/libtowr_dll.so")
#lib.Trajectory()
#print(lib)


no_of_samples = 10

float_array_3 = c_float*3


class Trajectory_data(Structure):
    _fields_ = [
            ('base_linear', c_float*3),
            ('base_angular',c_float*3),
            ('ee_linear', c_float*3),
            ('ee_force', c_float*3)
            
            ]





#t ={ 0.i,0.8,0.7}

def print_traj(a,arr_size,no_of_samples):
	time = 0.00
	for i in range(arr_size):
		print("i :",i," time :",i*2.0/no_of_samples)
		print("base_linear:",a[i].base_linear[0],"\t",a[i].base_linear[1],"\t",a[i].base_linear[2])
		print("base_angular:",a[i].base_angular[0],"\t",a[i].base_angular[1],"\t",a[i].base_angular[2])
		print("ee_linear",a[i].ee_linear[0],"\t",a[i].ee_linear[1],"\t",a[i].ee_linear[2])
		print("ee_force:",a[i].ee_force[0],"\t",a[i].ee_force[1],"\t",a[i].ee_force[2])






def cal_towr_code():
	target = float_array_3()
	target[0] =1
	target[1] =1
	target[2] =0.54

	

	base_initial_height = c_float(0.54)
	print("Enter no_of_samples:")
	no_of_samples = int(input())
	arr_size = no_of_samples+1
	lib.Trajectory.restype = None
	lib.Trajectory.argtypes = [ Trajectory_data*(arr_size) ,c_float ,c_float*3,c_int]
	print("\nTarget:")
	for i in range(3):
		print(target[i])
    
	traj_array = Trajectory_data*(arr_size)
	Result_traj = traj_array()

	#for i in range(no_of_samples):
	#	print("i+1 :",i+1," 2/(i+i) :",2.0/(i+1)," (i)*2.0/(i+1) :",(i)*2.0/(i+1))
	lib.Trajectory(Result_traj,base_initial_height,target,no_of_samples)
	print_traj(Result_traj,arr_size,no_of_samples)



cal_towr_code()