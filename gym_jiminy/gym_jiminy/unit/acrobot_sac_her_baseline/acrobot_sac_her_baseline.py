import os
import time

import gym

from stable_baselines.sac.policies import FeedForwardPolicy # Using 'sac' policies instead of the 'common' ones
from stable_baselines import SAC, HER

import jiminy_py

# Create a single-process environment
env = gym.make("gym_jiminy:jiminy-acrobot-v0")

### Create the model or load one

# Define a custom MLP policy with two hidden layers of size 128 and 64
class CustomPolicy(FeedForwardPolicy):
    __module__ = None # Necessary to avoid having to specify the policy when loading a model
    def __init__(self, *args, **_kwargs):
        super(CustomPolicy, self).__init__(*args, **_kwargs,
                                           layers=[128, 64],
                                           feature_extraction="mlp")

# Create 4 artificial transitions per real transition
n_sampled_goal = 4

# Set the Tensorboard path
tensorboard_data_path = os.path.dirname(os.path.realpath(__file__))

# Create the 'model' according to the chosen algorithm
model = HER(CustomPolicy, env, SAC,
            n_sampled_goal=n_sampled_goal,
            goal_selection_strategy='future', buffer_size=int(1e6),
            learning_rate=1e-3, gamma=0.95, batch_size=256,
            tensorboard_log=tensorboard_data_path)

# Load a model if desired
# model = HER.load("ppo2_cartpole", env=env, policy=CustomPolicy)

### Run the learning process
model.learn(total_timesteps=400000,
            log_interval=1,
            reset_num_timesteps=False)

# Save the model if desired
# model.save("ppo2_cartpole")

### Enjoy a trained agent

# duration of the simulations in seconds
t_end = 20

# Run the simulation in real-time
episode_reward = 0
obs = env.reset()
for _ in range(int(t_end/env.dt)):
    action, _states = model.predict(obs)
    obs, reward, done, info = env.step(action)
    env.render()
    time.sleep(env.dt)

    episode_reward += reward
    if done or info.get('is_success', False):
        print("Reward:", episode_reward,
              "Success:", info.get('is_success', False))
        episode_reward = 0.0
        obs = env.reset()
