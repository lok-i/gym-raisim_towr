from gym.envs.registration import register

register(
    id='raisim_towr-v0',
    entry_point='gym_raisim_towr.envs:Raisim_towrEnv',
)
