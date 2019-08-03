from gym.envs.registration import register

register(
    id='jiminy-cartpole-v0',
    entry_point='gym_jiminy.envs:JiminyCartPoleEnv',
)

register(
    id='jiminy-acrobot-v0',
    entry_point='gym_jiminy.envs:JiminyAcrobotEnv',
)