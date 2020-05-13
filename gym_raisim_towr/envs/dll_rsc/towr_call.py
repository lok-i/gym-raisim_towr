'''
file to test the functions
in towr dll created
'''



from ctypes import *

lib = CDLL("./build/libtowr_anymal_dll.so")
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

class Trajectory_data2(Structure):
    _fields_ = [
            ('base_linear', c_float*3),
            ('base_angular',c_float*3),
            ('ee_linear', c_float*12),
            ('ee_force', c_float*12)
            
            ]





#t ={ 0.i,0.8,0.7}

def print_traj(a,arr_size,no_of_samples):
	time = 0.00
	for i in range(arr_size):
		print("\n\ni :",i," time :",i*2.0/no_of_samples)
		print("base_linear:",a[i].base_linear[0],"\t",a[i].base_linear[1],"\t",a[i].base_linear[2])
		print("base_angular:",a[i].base_angular[0],"\t",a[i].base_angular[1],"\t",a[i].base_angular[2])
		print("ee_linear",a[i].ee_linear[0],"\t",a[i].ee_linear[1],"\t",a[i].ee_linear[2])
		print("ee_force:",a[i].ee_force[0],"\t",a[i].ee_force[1],"\t",a[i].ee_force[2])

def print_traj2(a,arr_size,no_of_samples):
	time = 0.00
	for i in range(arr_size):
		print("\n\ni :",i," time :",i*2.0/no_of_samples)
		print("base_linear:",a[i].base_linear[0],"\t",a[i].base_linear[1],"\t",a[i].base_linear[2])
		print("base_angular:",a[i].base_angular[0],"\t",a[i].base_angular[1],"\t",a[i].base_angular[2])
		for j in range(4):
			print("leg_no:",j)
			print("ee_linear_leg:",a[i].ee_linear[0+3*j],"\t",a[i].ee_linear[1+3*j],"\t",a[i].ee_linear[2+3*j])
			print("ee_force_leg:", a[i].ee_force[0+3*j],"\t",a[i].ee_force[1+3*j],"\t",a[i].ee_force[2+3*j])






def cal_towr_code():
	target = float_array_3()
	target[0] =4
	target[1] =1
	target[2] =0.54

	

	base_initial_height = c_float(0.54)
	print("Enter no_of_samples:")
	no_of_samples = int(input())
	arr_size = no_of_samples+1
	lib.Trajectory.restype = None
	lib.Trajectory.argtypes = [ Trajectory_data2*(arr_size) ,c_float ,c_float*3,c_int]
	print("\nTarget:")
	for i in range(3):
		print(target[i])
    
	traj_array = Trajectory_data2*(arr_size)
	Result_traj = traj_array()

	#for i in range(no_of_samples):
	#	print("i+1 :",i+1," 2/(i+i) :",2.0/(i+1)," (i)*2.0/(i+1) :",(i)*2.0/(i+1))
	lib.Trajectory(Result_traj,base_initial_height,target,no_of_samples)
	print_traj2(Result_traj,arr_size,no_of_samples)



cal_towr_code()