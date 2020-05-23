import pybullet as p
import os
import time
import numpy as np
import pybullet_data

from ctypes import *

lib = CDLL("./build/libtowr_anymal_dll.so")

file_path = "./rsc/anymal/"


class Trajectory_data2(Structure):
    _fields_ = [
            ('base_linear', c_float*3),
            ('base_angular',c_float*3),
            ('ee_linear', c_float*12),
            ('ee_force', c_float*12)
            
            ]





#p.resetBasePositionAndOrientation(anymal, [0, 0, 0.6], quat)

def print_traj2(a,no_of_samples):
	time = 0.00
	for i in range(no_of_samples+1):
		print("\n\ni :",i," time :",i*2.0/no_of_samples)
		print("base_linear:",a[i].base_linear[0],"\t",a[i].base_linear[1],"\t",a[i].base_linear[2])
		print("base_angular:",a[i].base_angular[0],"\t",a[i].base_angular[1],"\t",a[i].base_angular[2])
		for j in range(4):
			print("leg_no:",j)
			print("\tee_linear_leg:",a[i].ee_linear[0+3*j],"\t",a[i].ee_linear[1+3*j],"\t",a[i].ee_linear[2+3*j])
			#print("\tJoint_angles:",towr_joint_angles[i][0+3*j:3+3*j])
			print("\tee_force_leg:", a[i].ee_force[0+3*j],"\t",a[i].ee_force[1+3*j],"\t",a[i].ee_force[2+3*j])
			#print("\tJoint_Torques:",towr_joint_torques[i][0+3*j:3+3*j])


def cal_towr_code(initial_pos,target_pos):
	target = float_array_3()
	initial= float_array_3()
	for i in range(3):
		target[i] = target_pos[i]
		initial[i] = initial_pos[i]
	

	arr_size = no_of_samples+1
	lib.Trajectory.restype = None
	lib.Trajectory.argtypes = [ Trajectory_data2*(arr_size) ,c_float*3 ,c_float*3,c_int]
	print("\nTarget:")
	for i in range(3):
		print(target[i])
    
	traj_array = Trajectory_data2*(arr_size)
	Result_traj = traj_array()

	#for i in range(no_of_samples):
	#	print("i+1 :",i+1," 2/(i+i) :",2.0/(i+1)," (i)*2.0/(i+1) :",(i)*2.0/(i+1))
	lib.Trajectory(Result_traj,initial,target,no_of_samples)
	#print_traj2(Result_traj,arr_size,no_of_samples)
	return(Result_traj)


def set_anymal(anymal,base_pos,base_quat,ee1,ee2,ee3,ee4):
	angles = []
	for i in range(30):
		p.resetBasePositionAndOrientation(anymal,base_pos,base_quat)

		leg_LF = p.calculateInverseKinematics(anymal,5 ,ee1)#eight_vertx_LF[j])
		leg_RF = p.calculateInverseKinematics(anymal,10,ee2)#eight_vertx_RF[j])
		leg_LH = p.calculateInverseKinematics(anymal,15,ee3)#eight_vertx_LH[j])
		leg_RH = p.calculateInverseKinematics(anymal,20,ee4)#eight_vertx_RH[j])
		p.setJointMotorControlArray(bodyUniqueId=anymal,
	                                jointIndices = [1,2,3],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_LF[0:3],
	                                 )
		p.setJointMotorControlArray(bodyUniqueId=anymal,
	                                jointIndices = [6,7,8],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_RF[3:6],
	                                 )
		p.setJointMotorControlArray(bodyUniqueId=anymal,
	                                jointIndices = [11,12,13],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_LH[6:9],
	                                 )
		p.setJointMotorControlArray(bodyUniqueId=anymal,
	                                jointIndices = [16,17,18],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_RH[9:12],
	                                 )
		p.stepSimulation()
		angles = leg_LF[0:3]+leg_RF[3:6]+leg_LH[6:9]+leg_RH[9:12]
	return angles

def draw_frame(point):
	p.addUserDebugLine(point,[point[0]+0.1,point[1],point[2]],[1,0,0],5)
	p.addUserDebugLine(point,[point[0],point[1]+0.1,point[2]],[0,1,0],5)
	p.addUserDebugLine(point,[point[0],point[1],point[2]+0.1],[0,0,1],5)

def euler_to_quaternion(r):
    (roll, pitch, yaw) = (r[0], r[1], r[2])
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz,qw]

def ik_leg(anymal,leg_index,ee):
    angles=[]
    for i in range(20):
        angles = p.calculateInverseKinematics(anymal,(leg_index+1)*5 ,ee)
        p.stepSimulation()
    if(leg_index == 0):
    	return(angles[0:3])
    elif(leg_index == 1):
    	return(angles[3:6])
    elif(leg_index == 2):
    	return(angles[6:9])
    elif(leg_index == 3):
    	return(angles[9:12])

def find_force(anymal,leg_index,f_tip,angles):
	linear_jacob = p.calculateJacobian(anymal,(leg_index+1)*5,
		                               [0,0,0],angles,
		                               [0,0,0,0,0,0,0,0,0,0,0,0],
		                               [0,0,0,0,0,0,0,0,0,0,0,0])[0]
	
	# 3 x 3
	Jacob_without_base = np.array([linear_jacob[0][6+3*leg_index:9+3*leg_index],
		                           linear_jacob[1][6+3*leg_index:9+3*leg_index],
		                           linear_jacob[2][6+3*leg_index:9+3*leg_index]])
				#print("Sliced:\n",Jacob_without_base)
				#print("Transpose:\n",np.transpose(Jacob_without_base))
	f_tip = np.array(f_tip)
	# tau = J^T.f_tip
	torque_calculated = np.dot(np.transpose(Jacob_without_base),f_tip)
	return(torque_calculated)
	
				
		#towr_traj[i].ee_force[0+3*j:3+3*j])
		
no_of_samples = 100
float_array_3 = c_float*3
init_base_pos = [0,0,0.5]

def run_towr_tracking(target_pos):
	current_base_pos = init_base_pos
	p.connect(p.GUI)
	p.resetSimulation()
	#log = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4,fileName="./demo_log.mp4")
	plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0) #5,10,15,20
	p.setGravity(0,0,-10)
	anymal = p.loadURDF(file_path+"anymal.urdf",[0,2,0.5])
	vis_anymal = p.loadURDF(file_path+"anymal_vis.urdf",[0,0,0.5])

	set_anymal(vis_anymal,[0,0,0.5],[0,0,0,1], [0.34,0.19,init_base_pos[2]-0.42],
											   [0.34,-0.19,init_base_pos[2]-0.42],
    										   [-0.34,0.19,init_base_pos[2]-0.42],
    										   [-0.34,-0.19,init_base_pos[2]-0.42])

	set_anymal(anymal,[0,2,0.5],[0,0,0,1], [0.34,2+0.19,init_base_pos[2]-0.42],
											   [0.34,2-0.19,init_base_pos[2]-0.42],
    										   [-0.34,2+0.19,init_base_pos[2]-0.42],
    										   [-0.34,2-0.19,init_base_pos[2]-0.42])
	reached_or_out = True
	dummy = 0
	while(reached_or_out):
		draw_frame(current_base_pos)
		start = time.time()
		Result_traj = cal_towr_code(current_base_pos,target_pos)
		end = time.time()
		#print_traj2(Result_traj,no_of_samples)
		print("Trajectory_calculation_time:",end - start)
		for towr_sample_index in range(no_of_samples+1):
			towr_quat = euler_to_quaternion(Result_traj[towr_sample_index].base_angular)
			#p.resetBasePositionAndOrientation(vis_anymal,Result_traj[towr_sample_index].base_linear ,towr_quat)
			angles = set_anymal(vis_anymal,Result_traj[towr_sample_index].base_linear ,towr_quat,
				                           Result_traj[towr_sample_index].ee_linear[0:3],
				                           Result_traj[towr_sample_index].ee_linear[3:6],
										   Result_traj[towr_sample_index].ee_linear[6:9],
				                           Result_traj[towr_sample_index].ee_linear[9:12])

										
			p.stepSimulation()
			time.sleep(0.01)
			for i in range(4):

				if(i == 0):
					current_leg_angles = angles[0:3]
				elif(i == 1):
					current_leg_angles = angles[3:6]
				elif(i == 2):
					current_leg_angles = angles[6:9]
				elif(i == 3):
					current_leg_angles = angles[9:12]
				
				f_tip = Result_traj[towr_sample_index].ee_force[0+3*i:3+3*i]
				torques = find_force(vis_anymal,i,f_tip,angles)
				
				#if(len(p.getContactPoints(plane,vis_anymal,-1,5*(i+1)))==0):
				#print("POSITION_CONTROL")
				p.setJointMotorControlArray(bodyUniqueId=anymal,
	                                			jointIndices = [1+5*i,2+5*i,3+5*i],
	                                			controlMode=p.POSITION_CONTROL,
	                                			targetPositions=current_leg_angles)
				# else:
	 		# 		print("TORQUE_CONTROL")
	 		# 		p.setJointMotorControlArray(bodyUniqueId=anymal,
	   #                              			jointIndices = [1+5*i,2+5*i,3+5*i],
	   #                              			controlMode=p.TORQUE_CONTROL,
	   #                              			forces=torques)
	 			


	    
		current_base_anymal = p.getBasePositionAndOrientation(anymal)[0]
		target_anymal = [target_pos[0],target_pos[1]-2,target_pos[2]]
		if(target_anymal[0]<current_base_anymal[0]):
			reached_or_out = False
			#current_base_pos = [2,current_base_anymal[1]-2,current_base_anymal[2]]
		else:
			current_base_pos = [current_base_anymal[0],current_base_anymal[1]-2,current_base_anymal[2]]

run_towr_tracking([2,0,0.54])
#p.stopStateLogging(log)
