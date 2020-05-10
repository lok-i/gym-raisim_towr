import pybullet as p
import os
import time
import pybullet_data
file_path = "./rsc/anymal/anymal.urdf"


p.connect(p.GUI)


p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0) #5,10,15,20,
p.setGravity(0,0,0)
anymal = p.loadURDF(file_path)
quat = p.getQuaternionFromEuler([90,0,0])
print(quat)
p.resetBasePositionAndOrientation(anymal, [0, 0, 0.6], quat)
#p.resetBasePositionAndOrientation(bot2, [0, 0, 1], [0, 0, 0, 0.707])
# num_of_joints = p.getNumJoints(anymal);
# print("num_of_joints",p.getNumJoints(anymal))
for i in range(p.getNumJoints(anymal)):
	print("\nlink_info",i,":",p.getJointInfo(anymal,i))
b = [1,2,3,90,6,7]
print("b\n",b,"\n",type(b[1:3]),"\n",b[1:3])
print('anymal_dof',)
while(True):
	#p.resetBasePositionAndOrientation(anymal, [0, 0, 0.6], quat)
	leg1 = p.calculateInverseKinematics(anymal,5,[0,0,0]) 
	leg2 = p.calculateInverseKinematics(anymal,10,[0,0,0]) 
	leg3 = p.calculateInverseKinematics(anymal,15,[0,0,0]) 
	leg4 = p.calculateInverseKinematics(anymal,20,[0,0,0])
	
	# for i in range(3):
	# 	p.setJointMotorControl2(bodyIndex=anymal,
 #                                jointIndex=i+1,
 #                                controlMode=p.POSITION_CONTROL,
 #                                targetPosition=leg1[i],
 #                                targetVelocity=0,
 #                                force=500)
		
	# 	p.setJointMotorControl2(bodyIndex=anymal,
 #                                jointIndex=i+6,
 #                                controlMode=p.POSITION_CONTROL,
 #                                targetPosition=leg2[i+3],
 #                                targetVelocity=0,
 #                                force=500)
		
	# 	p.setJointMotorControl2(bodyIndex=anymal,
 #                                jointIndex=i+11,
 #                                controlMode=p.POSITION_CONTROL,
 #                                targetPosition=leg3[i+6],
 #                                targetVelocity=0,
 #                                force=500)
		
	# 	p.setJointMotorControl2(bodyIndex=anymal,
 #                                jointIndex=i+16,
 #                                controlMode=p.POSITION_CONTROL,
 #                                targetPosition=leg4[i+9],
 #                                targetVelocity=0,
 #                                force=500)
		


	#print(leg1,"\n",leg2,"\n",leg3,"\n",leg4)
	time.sleep(0.01)
	p.stepSimulation()

