import pybullet as p
import os
import time
import numpy as np
import pybullet_data

from ctypes import *

lib = CDLL("./build/libtowr_anymal_dll.so")

file_path = "./rsc/anymal/anymal.urdf"


class Trajectory_data2(Structure):
    _fields_ = [
            ('base_linear', c_float*3),
            ('base_angular',c_float*3),
            ('ee_linear', c_float*12),
            ('ee_force', c_float*12)
            
            ]


no_of_samples = 10
float_array_3 = c_float*3
init_base_pos = [0,0,0.5]


#p.resetBasePositionAndOrientation(anymal, [0, 0, 0.6], quat)

def print_traj2(a,towr_joint_angles,towr_joint_torques,no_of_samples):
	time = 0.00
	for i in range(no_of_samples+1):
		print("\n\ni :",i," time :",i*2.0/no_of_samples)
		print("base_linear:",a[i].base_linear[0],"\t",a[i].base_linear[1],"\t",a[i].base_linear[2])
		print("base_angular:",a[i].base_angular[0],"\t",a[i].base_angular[1],"\t",a[i].base_angular[2])
		for j in range(4):
			print("leg_no:",j)
			print("\tee_linear_leg:",a[i].ee_linear[0+3*j],"\t",a[i].ee_linear[1+3*j],"\t",a[i].ee_linear[2+3*j])
			print("\tJoint_angles:",towr_joint_angles[i][0+3*j:3+3*j])
			print("\tee_force_leg:", a[i].ee_force[0+3*j],"\t",a[i].ee_force[1+3*j],"\t",a[i].ee_force[2+3*j])
			print("\tJoint_Torques:",towr_joint_torques[i][0+3*j:3+3*j])


def cal_towr_code(initial_pos,target_pos):
	target = float_array_3()
	target[0] = target_pos[0]
	target[1] = target_pos[1]
	target[2] = target_pos[2]

	

	base_initial_height = c_float(initial_pos[2])

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
	#print_traj2(Result_traj,arr_size,no_of_samples)
	return(Result_traj)


def init_anymal(geo_anymal):
	p.resetBasePositionAndOrientation(geo_anymal, init_base_pos,[0,0,0,1])

	for i in range(30):
		leg_LF = p.calculateInverseKinematics(geo_anymal,5 ,[0.34,0.19,init_base_pos[2]-0.42])#eight_vertx_LF[j])
		leg_RF = p.calculateInverseKinematics(geo_anymal,10,[0.34,-0.19,init_base_pos[2]-0.42])#eight_vertx_RF[j])
		leg_LH = p.calculateInverseKinematics(geo_anymal,15,[-0.34,0.19,init_base_pos[2]-0.42])#eight_vertx_LH[j])
		leg_RH = p.calculateInverseKinematics(geo_anymal,20,[-0.34,-0.19,init_base_pos[2]-0.42])#eight_vertx_RH[j])
		p.setJointMotorControlArray(bodyUniqueId=geo_anymal,
	                                jointIndices = [1,2,3],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_LF[0:3],
	                                 )
		p.setJointMotorControlArray(bodyUniqueId=geo_anymal,
	                                jointIndices = [6,7,8],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_RF[3:6],
	                                 )
		p.setJointMotorControlArray(bodyUniqueId=geo_anymal,
	                                jointIndices = [11,12,13],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_LH[6:9],
	                                 )
		p.setJointMotorControlArray(bodyUniqueId=geo_anymal,
	                                jointIndices = [16,17,18],
	                                controlMode=p.POSITION_CONTROL,
	                                targetPositions=leg_RH[9:12],
	                                 )
		p.stepSimulation()

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

def pybullet_ik(geo_anymal,base_pos,base_quat,ee1,ee2,ee3,ee4):
    base_pos = list(base_pos)
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

def pre_calculate_towr_ik_angles(geo_anymal,towr_traj):
	towr_joint_angles = []
	for i in range(no_of_samples+1):
		towr_quat = euler_to_quaternion(towr_traj[i].base_angular)
		towr_joint_angles.append(list(pybullet_ik(geo_anymal,towr_traj[i].base_linear, towr_quat,
                                              	towr_traj[i].ee_linear[0:3],
                                              	towr_traj[i].ee_linear[3:6],
                                              	towr_traj[i].ee_linear[6:9],
                                              	towr_traj[i].ee_linear[9:12])))
	return(towr_joint_angles)

def pre_calculate_towr_torques_from_ftip(geo_anymal,towr_traj,towr_joint_angles):
	towr_joint_torques = []
	for i in range(no_of_samples+1):
		#time.sleep(1)
		towr_quat = euler_to_quaternion(towr_traj[i].base_angular)
		p.resetBasePositionAndOrientation(geo_anymal,towr_traj[i].base_linear,towr_quat)
		torque_per_pose = []
		for j in range(4):
				
				#link_state = p.getLinkState(geo_anymal,5+5*j)
				#draw_frame(link_state[0])
				#might be worng as the legs are in air, but Jacob still takes dezired angles
				linear_jacob = p.calculateJacobian(geo_anymal,5+5*j,[0,0,0],towr_joint_angles[i],[0,0,0,0,0,0,0,0,0,0,0,0],[0,0,0,0,0,0,0,0,0,0,0,0])[0]
	
				#print("\nJacob_of_leg:",i+1,"\n",linear_jacob[i])
				#3 x 3

				Jacob_without_base = np.array([linear_jacob[0][6+3*j:9+3*j],
		                           			   linear_jacob[1][6+3*j:9+3*j],
		                           		       linear_jacob[2][6+3*j:9+3*j]])
				#print("Sliced:\n",Jacob_without_base)
				#print("Transpose:\n",np.transpose(Jacob_without_base))
				ftip = np.array(towr_traj[i].ee_force[0+3*j:3+3*j])
				# tau = J^T.f_tip
				torque_calculated = np.dot(np.transpose(Jacob_without_base),ftip)
				torque_per_pose.append(list(torque_calculated))

		towr_joint_torques.append(torque_per_pose[0]+torque_per_pose[1]+torque_per_pose[2]+torque_per_pose[3])
	#print(towr_joint_torques)
	return towr_joint_torques

def run_towr_tracking(target_pos):
	current_base_pos = init_base_pos
	Simulation_state = -1
	first = False
	while(True):
		
		Result_traj = cal_towr_code(current_base_pos,target_pos)
		#print_traj2(Result_traj,no_of_samples+1,no_of_samples)
		
		# compute new trjectory from current position
		p.connect(p.DIRECT)
		p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0) #5,10,15,20,
		p.setGravity(0,0,-10)
		geo_anymal = p.loadURDF(file_path)
		init_anymal(geo_anymal)
		towr_joint_angles  = pre_calculate_towr_ik_angles(geo_anymal,Result_traj)
		towr_joint_torques = pre_calculate_towr_torques_from_ftip(geo_anymal,Result_traj,towr_joint_angles)
		p.disconnect()

		p.connect(p.GUI)
		if(first):
			p.restoreState(Simulation_state)
		first = True
		#print_traj2(Result_traj,towr_joint_angles,towr_joint_torques,no_of_samples)
		p.resetSimulation()
		plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0) #5,10,15,20
		p.setGravity(0,0,-10)
		geo_anymal = p.loadURDF(file_path)
		init_anymal(geo_anymal)
		t = 0

		for towr_sample_index in range(no_of_samples+1):
			p.stepSimulation()
			time.sleep(0.01)
			for i in range(4):
	 			
	 			link_state = p.getLinkState(geo_anymal,5+5*i)
	 			draw_frame(link_state[0])
	 			
	 			#print("use FORCE_CONTROL")
	 			print("collision info of leg",i+1,"\n",p.getContactPoints(plane,geo_anymal,-1,5*5+i))
	 			if(len(p.getContactPoints(plane,geo_anymal,-1,5*5+i))==0):
	 				print("Leg",i+1,"  use POSITION_CONTROL")
	 				# p.setJointMotorControlArray(bodyUniqueId=geo_anymal,
	     #                            		jointIndices = [1+5*i,2+5*i,3+5*i],
	     #                            		controlMode=p.POSITION_CONTROL,
	     #                            		targetPositions=towr_joint_angles[towr_sample_index][0+3*i:3+3*i])
	 			else:
	 				print("torque_applied_to_leg:",i+1,":",towr_joint_torques[towr_sample_index][0+3*i:3+3*i])
	 				# p.setJointMotorControlArray(bodyUniqueId=geo_anymal,
	     #                            		jointIndices = [1+5*i,2+5*i,3+5*i],
	     #                            		controlMode=p.TORQUE_CONTROL,
	     #                            		forces=100*np.array(towr_joint_torques[towr_sample_index][0+3*i:3+3*i])
	    
		current_base = p.getBasePositionAndOrientation(geo_anymal)
		Simulation_state = p.saveState()
		p.disconnect()		


run_towr_tracking([2,0,0.54])
# p.connect(p.GUI)
# plane = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0) #5,10,15,20,
# p.setGravity(0,0,-10)
# geo_anymal = p.loadURDF(file_path)

# p.setJointMotorControlArray(bodyUniqueId=geo_anymal,
# 	                                jointIndices = [1,2,3],
# 	                                controlMode=p.POSITION_CONTROL,
# 	                                targetPositions=[0,0,0],
# 	                                 )
# p.setJointMotorControlArray(bodyUniqueId=geo_anymal,
# 	                                jointIndices = [6,7,8],
# 	                                controlMode=p.POSITION_CONTROL,
# 	                                targetPositions=[-0.57,0,0],
# 	                                 )
# p.setJointMotorControlArray(bodyUniqueId=geo_anymal,
# 	                                jointIndices = [11,12,13],
# 	                                controlMode=p.POSITION_CONTROL,
# 	                                targetPositions=[1.57,0,0],
# 	                                 )
# p.setJointMotorControlArray(bodyUniqueId=geo_anymal,
# 	                                jointIndices = [16,17,18],
# 	                                controlMode=p.POSITION_CONTROL,
# 	                                targetPositions=[-0.57,0,0],
# 	                                 )
# for i in range(100):
# 	time.sleep(0.1)
# 	p.resetBasePositionAndOrientation(geo_anymal, [0,0,1-i*0.01],[0,0,0,1])
# 	p.stepSimulation()
# 	if(len(p.getContactPoints(plane,geo_anymal,-1,5))==0):
# 		print("leg 1 No_contact")
# 	else:
# 		print("leg 1 In_contact")
# 	if(len(p.getContactPoints(plane,geo_anymal,-1,10))==0):
# 		print("leg 2 No_contact")
# 	else:
# 		print("leg 2 In_contact")
# 	if(len(p.getContactPoints(plane,geo_anymal,-1,15))==0):
# 		print("leg 3 No_contact")
# 	else:
# 		print("leg 3 In_contact")
# 	if(len(p.getContactPoints(plane,geo_anymal,-1,20))==0):
# 		print("leg 4 No_contact")
# 	else:
# 		print("leg 4 In_contact")

	#print("i:",i,"\n",p.getContactPoints(plane,geo_anymal,-1,5))
	
#init_anymal(geo_anymal)