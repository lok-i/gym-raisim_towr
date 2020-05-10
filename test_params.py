no_of_episodes = 5000 
no_of_steps_per_epi = 100
# independent of last z axis as it will be towr depended 
# thus only x and y matter of the final base position
target = [2,0,0.54]
base_init_height = 0.56
agent_name = 'Test-7'

'''
10/5/20
-------
TESTS FOR ANGLE LIMITS
Fixed Params:
-------------
*no_of_episodes = 5000 
*no_of_steps_per_epi = 100
*target = [2,0,0.54]
*base_init_height = 0.56
*Hyper params according to levine
*1 action update per 4 simulation steps
*Reward Fucnction = Base pos + quat exponential term alone
*State Space :
[base_quat[4],genralized_joint_angles[12],
genralized_joint_velocities[12],genralized_joint_forces[12],
goal_base_quat(t+1)[4]]
*Action Space :
12 joint angles ,3 per leg (pd targets)

Test param-fine tuning: Test -1
-----------------------
Test - 1 : 
joint limit = 0.39 
Test - 2 :
joint limit = 0.6

TESTS FOR NEW REWARD FUNCTION
Fixed Params:
-------------
*no_of_episodes = 5000 
*no_of_steps_per_epi = 100
*target = [2,0,0.54]
*base_init_height = 0.56
*Hyper params according to levine
*1 action update per 4 simulation steps

*State Space :
[base_quat[4],genralized_joint_angles[12],
genralized_joint_velocities[12],genralized_joint_forces[12],
goal_base_quat(t+1)[4]]
*Action Space :
12 joint angles ,3 per leg (pd targets)

* joint limit = 0.39 
Test param-fine tuning:
-----------------------
Test - 3: learning curve looks gud
*Reward Fucnction =  0.76923(Base pos + quat weighted exponential) 
                    +0.23076(joint angles weighted exponential)
Test - 4: no big change
*Reward Fucnction =  0.76923(Base pos + quat weighted exponential) 
                    +0.23076(joint angles weighted exponential) - 1(if base height < 0.4)

Test - 5:
*Reward Function = (0.76923*raisim_pos[0] + 0.23076*raisim_pos[2])/(raisim_pos[0]+raisim_pos[2])

Test - 6:
*Reward Function = (0.76923*raisim_pos[0] + 0.23076*raisim_pos[2])/(raisim_pos[0]+raisim_pos[2]) - penalty

TESTS FOR STATE SPACE
Fixed Params:
-------------
*no_of_episodes = 5000 
*no_of_steps_per_epi = 100
*target = [2,0,0.54]
*base_init_height = 0.56
*Hyper params according to levine
*1 action update per 4 simulation steps


*Action Space :
12 joint angles ,3 per leg (pd targets)
*Reward Fucnction =  0.76923(Base pos + quat weighted exponential) 
                    +0.23076(joint angles weighted exponential)
* joint limit = 0.39 
Test param-fine tuning:
-----------------------
Test - 7:
*State Space : camparable to test 3 , test 3 id slightly better though
[base_quat[4],genralized_joint_angles[12],
genralized_joint_velocities[12],genralized_joint_forces[12],
goal_base_quat(t+1)[4],goal_joint_angles(t+1)[12]]

TESTS FOR after fixing abduction motors
Fixed Params:
-------------
*no_of_episodes = 5000 
*no_of_steps_per_epi = 100
*target = [2,0,0.54]
*base_init_height = 0.56
*Hyper params according to levine
*1 action update per 4 simulation steps


*Action Space :
12 joint angles ,3 per leg (pd targets)
*Reward Fucnction =  0.76923(Base pos + quat weighted exponential) 
                    +0.23076(joint angles weighted exponential)

* joint limit = 0.39 
Test param-fine tuning:
-----------------------
Test - 8:





'''