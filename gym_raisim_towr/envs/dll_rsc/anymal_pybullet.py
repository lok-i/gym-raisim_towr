'''
test file for checking functions
and values in realtion to 
pybullet 
'''




import pybullet as p
import os
import time
import numpy as np
import pybullet_data
file_path = "./rsc/anymal/anymal.urdf"


p.connect(p.GUI)


p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0) #5,10,15,20,
p.setGravity(0,0,-10)
geo_anymal = p.loadURDF(file_path)
quat = p.getQuaternionFromEuler([0,0,0])
#p.resetBasePositionAndOrientation(anymal, [0, 0, 0.6], quat)


def save_towr_angles(base_pos,base_quat,ee1,ee2,ee3,ee4):
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



base_pos = [0,0,0.56]

#p.resetBasePostionAndOrientation(bot2, [0, 0, 1], [0, 0, 0, 0.707])
# num_of_joints = p.getNumJoints(anymal);
# print("num_of_joints",p.getNumJoints(anymal))
# for i in range(p.getNumJoints(anymal)):
# 	print("\nlink_info",i,":",p.getJointInfo(anymal,i))# b = [12,3,90,6,7]
# print("b\n",b,"\n",type(b[1:3]),"\n",b[1:3])
# print('anymal_dof',)

eight_vertx_LF = np.array([[0.19,0.09,base_pos[2]-0.42-0.1],
						[0.19,0.29,base_pos[2]-0.42-0.1],
						[0.19,0.29,base_pos[2]-0.42+0.1],
						[0.19,0.09,base_pos[2]-0.42+0.1],

						[0.49,0.09,base_pos[2]-0.42-0.1],
						[0.49,0.29,base_pos[2]-0.42-0.1],
						[0.49,0.29,base_pos[2]-0.42+0.1],
						[0.49,0.09,base_pos[2]-0.42+0.1],])
eight_vertx_RF = np.array([[0.19,-0.09,base_pos[2]-0.42-0.1],
						   [0.19,-0.29,base_pos[2]-0.42-0.1],
						   [0.19,-0.29,base_pos[2]-0.42+0.1],
						   [0.19,-0.09,base_pos[2]-0.42+0.1],

						   [0.49,-0.09,base_pos[2]-0.42-0.1],
						   [0.49,-0.29,base_pos[2]-0.42-0.1],
						   [0.49,-0.29,base_pos[2]-0.42+0.1],
						   [0.49,-0.09,base_pos[2]-0.42+0.1],])

eight_vertx_LH = np.array([[-0.19,0.09,base_pos[2]-0.42-0.1],
						   [-0.19,0.29,base_pos[2]-0.42-0.1],
						   [-0.19,0.29,base_pos[2]-0.42+0.1],
						   [-0.19,0.09,base_pos[2]-0.42+0.1],

						   [-0.49,0.09,base_pos[2]-0.42-0.1],
						   [-0.49,0.29,base_pos[2]-0.42-0.1],
						   [-0.49,0.29,base_pos[2]-0.42+0.1],
						   [-0.49,0.09,base_pos[2]-0.42+0.1],])

eight_vertx_RH = np.array([[-0.19,-0.09,base_pos[2]-0.42-0.1],
						   [-0.19,-0.29,base_pos[2]-0.42-0.1],
						   [-0.19,-0.29,base_pos[2]-0.42+0.1],
						   [-0.19,-0.09,base_pos[2]-0.42+0.1],

						   [-0.49,-0.09,base_pos[2]-0.42-0.1],
						   [-0.49,-0.29,base_pos[2]-0.42-0.1],
						   [-0.49,-0.29,base_pos[2]-0.42+0.1],
						   [-0.49,-0.09,base_pos[2]-0.42+0.1],])




'''
x and y define the leg selected

x = 1, y = 1	-	LF
x = 1, y =-1	-	RF
x =-1, y = 1	-	LH
x =-1, y =-1	-	RH
'''

def kinematic_dabba(x,y):

	p.addUserDebugLine([x*0.19,y*0.09,base_pos[2]-0.42-0.1],[x*0.49,y*0.09,base_pos[2]-0.42-0.1],[1,0,0],3)
	p.addUserDebugLine([x*0.49,y*0.09,base_pos[2]-0.42-0.1],[x*0.49,y*0.29,base_pos[2]-0.42-0.1],[1,0,0],3)
	
	p.addUserDebugLine([x*0.19,y*0.29,base_pos[2]-0.42-0.1],[x*0.49,y*0.29,base_pos[2]-0.42-0.1],[1,0,0],3)
	p.addUserDebugLine([x*0.19,y*0.09,base_pos[2]-0.42-0.1],[x*0.19,y*0.29,base_pos[2]-0.42-0.1],[1,0,0],3)

	p.addUserDebugLine([x*0.19,y*0.09,base_pos[2]-0.42+0.1],[x*0.49,y*0.09,base_pos[2]-0.42+0.1],[1,0,0],3)
	p.addUserDebugLine([x*0.49,y*0.09,base_pos[2]-0.42+0.1],[x*0.49,y*0.29,base_pos[2]-0.42+0.1],[1,0,0],3)
		
	p.addUserDebugLine([x*0.19,y*0.29,base_pos[2]-0.42+0.1],[x*0.49,y*0.29,base_pos[2]-0.42+0.1],[1,0,0],3)
	p.addUserDebugLine([x*0.19,y*0.09,base_pos[2]-0.42+0.1],[x*0.19,y*0.29,base_pos[2]-0.42+0.1],[1,0,0],3)
		
	p.addUserDebugLine([x*0.49,y*0.09,base_pos[2]-0.42-0.1],[x*0.49,y*0.09,base_pos[2]-0.42+0.1],[1,0,0],3)
	p.addUserDebugLine([x*0.19,y*0.09,base_pos[2]-0.42-0.1],[x*0.19,y*0.09,base_pos[2]-0.42+0.1],[1,0,0],3)
	p.addUserDebugLine([x*0.49,y*0.29,base_pos[2]-0.42-0.1],[x*0.49,y*0.29,base_pos[2]-0.42+0.1],[1,0,0],3)
	p.addUserDebugLine([x*0.19,y*0.29,base_pos[2]-0.42-0.1],[x*0.19,y*0.29,base_pos[2]-0.42+0.1],[1,0,0],3)


t=0

max_angles = np.full(12,-np.inf)
min_angles = np.full(12,np.inf)

#print(max_angles)
#print(min_angles)
max_diff = -np.inf


while(True):
	kinematic_dabba(1,1)
	kinematic_dabba(1,-1)
	kinematic_dabba(-1,1) 
	kinematic_dabba(-1,-1)  
	for j in range(8):
		#print("vertex:",j+1,',',eight_vertx_LF[j])
		vertex_angles = []
		for i in range (300):
			
			p.resetBasePositionAndOrientation(geo_anymal, base_pos, quat)

			leg_LF = p.calculateInverseKinematics(geo_anymal,5 ,eight_vertx_LF[j])
			leg_RF = p.calculateInverseKinematics(geo_anymal,10,eight_vertx_RF[j])
			leg_LH = p.calculateInverseKinematics(geo_anymal,15,eight_vertx_LH[j])
			leg_RH = p.calculateInverseKinematics(geo_anymal,20,eight_vertx_RH[j])
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
			
			vertex_angles=leg_LF[0:3]+leg_RF[3:6]+leg_LH[6:9]+leg_RH[9:12]
			p.stepSimulation()
			time.sleep(0.01)
		#print("\nLF_RF_LH_RH_angles:",vertex_angles)
		angles_12 = np.array(save_towr_angles(base_pos,[1,0,0,0],eight_vertx_LF[j],
			                                                             eight_vertx_RF[j],
			                                                             eight_vertx_LH[j],
			                                                             eight_vertx_RH[j]))
		#print("\nsave_actuator_angles:\n",)
		for i in range(12):
			angles_12[i] = angles_12[i] - vertex_angles[i]
		print("\nAngle_diff:\n",angles_12)
		if(max_diff<np.amax(np.absolute(angles_12))):
			max_diff = np.amax(np.absolute(angles_12))
print("after_80_trials_maz_diff:",max_diff)


		# for i in range (12):
		# 	if(max_angles[i] < vertex_angles[i]):
		# 		max_angles[i] = vertex_angles[i]
		# 	if(min_angles[i] > vertex_angles[i]):
		# 		min_angles[i] = vertex_angles[i]