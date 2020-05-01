no_of_episodes = 1000
no_of_steps_per_epi = 300

agent_name = 'agent_angle_restricted_('+str(no_of_episodes)+')-('+str(no_of_steps_per_epi)+')'

# independent of last z axis as it will be towr depended 
# thus only x and y matter of the final base position
target = [3,0,0.54]