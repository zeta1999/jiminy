from gym.envs.registration import register

register(
    id='jiminy-cartpole-v0',
    entry_point='gym_jiminy.envs:JiminyCartPoleEnv',
)