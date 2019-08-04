import os
import numpy as np

from gym import core, spaces
from gym.utils import seeding

import jiminy
from jiminy_py import engine_asynchronous
from gym_jiminy.common import RenderOutMock


class RobotJiminyEnv(core.Env):
    """
    Base class for Jiminy actors in a Scene.
    These environments create single-player scenes and behave like normal Gym environments.
    """

    metadata = {
        'render.modes': ['human']
    }

    def __init__(self, robot_name, engine_py, dt):
        ####################### Configure the learning environment #############################

        self.robot_name = robot_name
        self.engine_py = engine_py
        self.dt = dt

        motors_position_idx = self.engine_py._engine.model.motors_position_idx
        joint_lower_position_limit = self.engine_py._engine.model.position_lower_limit
        joint_upper_position_limit = self.engine_py._engine.model.position_upper_limit
        joint_velocity_limit = self.engine_py._engine.model.velocity_limit

        action_low = joint_lower_position_limit[motors_position_idx].A1
        action_high = joint_upper_position_limit[motors_position_idx].A1
        self.action_space = spaces.Box(action_low, action_high, dtype=np.float64)

        obs_low = np.concatenate((joint_lower_position_limit.A1, -joint_velocity_limit.A1))
        obs_high = np.concatenate((joint_upper_position_limit.A1, joint_velocity_limit.A1))
        self.observation_space = spaces.Box(obs_low, obs_high, dtype=np.float64)

        self.obs_random_low = -0.1 * np.ones(self.observation_space.shape)
        self.obs_random_high = 0.1 * np.ones(self.observation_space.shape)
        self.state = None
        self.viewer = None
        self.steps_beyond_done = None

        self.seed()

        ####################### Enforce some options of the engine #############################

        engine_options = self.engine_py.get_engine_options()

        engine_options["stepper"]["iterMax"] = -1 #Infinite number of iteration
        engine_options["stepper"]["sensorsUpdatePeriod"] = self.dt
        engine_options["stepper"]["controllerUpdatePeriod"] = self.dt

        self.engine_py.set_engine_options(engine_options)

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        self.engine_py.seed(seed)
        self.state = self.engine_py.state
        return [seed]

    def reset(self):
        self.state = self.np_random.uniform(low=self.obs_random_low, high=self.obs_random_high)
        self.engine_py.reset(np.expand_dims(self.state, axis=-1))
        self.steps_beyond_done = None
        return self.state

    def render(self, lock=None, mode='human'):
        self.engine_py.render(lock)
        if (self.viewer is None):
            self.viewer = self.engine_py._client
        return RenderOutMock()

    def close(self):
        if (self.viewer is not None):
            self.engine_py.close()


class RobotJiminyGoalEnv(RobotJiminyEnv, core.GoalEnv):
    """
    Base class for Jiminy actors in a Scene.
    These environments create single-player scenes and behave like normal Gym goal-environments.
    """

    pass