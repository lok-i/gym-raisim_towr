no_of_episodes = 5000 
no_of_steps_per_epi = 200
# independent of last z axis as it will be towr depended 
# thus only x and y matter of the final base position
target = [2,0,0.54]
base_init_pos = [0,0,0.54]
agent_name = 'Test-18'

'''
14/5/20

Test - 13
* scale_angles_action used
* rf ,scaling fixed
15/5/20

Test - 14
* with 5 sim update per learning action
* done wen fallen down
* -1 reward for falling down
self.joint_angle_ul =[ 0.1331395 ,1.68423158,-0.38000404,
                           0.49791087,1.68423119,-0.38000466,
                           0.13313961,-0.12118592,1.67963936, 
                           0.49791083,-0.12118621,1.67963958]
   
self.joint_angle_ll =[-0.49791066,0.12118728,-1.67963886,
                          -0.13313934,0.12118650,-1.67963938,
                          -0.4979109,-1.68423165,0.38000445, 
                          -0.13313932,-1.68423096,0.3800044 ]

Test - 15

new limits:-

    self.joint_angle_ul =[ 0.1331395 ,1,-0.38000404,
                           0.49791087,1,-0.38000466,
                           0.13313961,-0.12118592,1.4, 
                           0.49791083,-0.12118621,1.4]
   
    self.joint_angle_ll =[-0.49791066,0.12118728,-1.4,
                          -0.13313934,0.12118650,-1.4,
                          -0.4979109,-1,0.38000445, 
                          -0.13313932,-1,0.3800044 ]
Test - 16

new limits:-

    self.joint_angle_ul =[ 0.1331395 ,1,-0.8,
                           0.49791087,1,-0.8,
                           0.13313961,-0.5,1.4, 
                           0.49791083,-0.5,1.4]
   
    self.joint_angle_ll =[-0.49791066,0.5,-1.4,
                          -0.13313934,0.5,-1.4,
                          -0.4979109,-1,0.8, 
                          -0.13313932,-1,0.8 ]
Test - 17 
* decreased learning rate

Test -18
* abducntion removed from consideration in 
->action space
->reward function

'''