from gym.envs.registration import register

register(
    id='raisim_towr-v0',
    entry_point='gym_raisim_towr.envs:Raisim_towrEnv',
)

register(
    id='raisim_towr_anymal-v0',
    entry_point='gym_raisim_towr.envs:Raisim_towr_anymalEnv',
)
