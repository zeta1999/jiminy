import os
import time

import gym

from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO2

import jiminy_py
from gym_jiminy.unit import SubprocVecEnvLock

# multiprocess environment
nb_cpu = 4
env = SubprocVecEnvLock([lambda: gym.make("gym_jiminy:jiminy-cartpole-v0") for _ in range(nb_cpu)])

env.remotes[0].send(('get_attr','dt'))
dt = env.remotes[0].recv()

tensorboard_data_path = os.path.dirname(os.path.realpath(__file__))
print("Tensorboard log path: " + tensorboard_data_path)
model = PPO2(MlpPolicy, env, tensorboard_log=tensorboard_data_path) # Use verbose=1 instead of tensorboard_log for stdout monitoring
model.learn(total_timesteps=1000000)

# model.save("ppo2_cartpole")
# del model
# model = PPO2.load("ppo2_cartpole")

# Enjoy trained agent
t_end = 20 # duration of the simulations in seconds
obs = env.reset()
for _ in range(int(t_end/dt)):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render('rgb_array')
    time.sleep(dt)

env.close()
