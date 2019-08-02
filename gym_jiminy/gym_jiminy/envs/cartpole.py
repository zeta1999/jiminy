"""
Classic cart-pole system simulated using Jiminy Engine
"""

import os
import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np

import jiminy
from jiminy_py import *


class RenderMockOut:
    def __init__(self):
        self.mock = np.array([[[]]])

    def __array__(self):
        return self.mock

    def __iter__(self):
        return iter(self.mock)

    def __len__(self):
        return len(self.mock)

    def __getitem__(self, key):
        return self.mock.__getitem__(key)


class JiminyCartPoleEnv(gym.Env):
    """
    Description:
        A pole is attached by an un-actuated joint to a cart.
        The goal is to prevent the pendulum from falling over by increasing and reducing the cart's velocity.
    Observation:
        Type: Box(4)
        Num	Observation                Min         Max
        0	Cart Position             -1.5         1.5
        1	Cart Velocity             -Inf         Inf
        2	Pole Angle                -50 deg      50 deg
        3	Pole Velocity At Tip      -Inf         Inf

    Actions:
        Type: Discrete(2)
        Num	Action
        0	Push cart to the left
        1	Push cart to the right

        Note: The amount the velocity that is reduced or increased is not fixed, it depends on the angle the pole is pointing.
              This is because the center of gravity of the pole increases the amount of energy needed to move the cart underneath it.
    Reward:
        Reward is 1 for every step taken, including the termination step.
    Starting State:
        All observations are assigned a uniform random value in [-0.05..0.05]
    Episode Termination:
        Pole Angle is more than 25 degrees
        Cart Position is more than 75 cm
        Episode length is greater than 200
    Solved Requirements:
        Considered solved when the average reward is greater than or equal to 195.0 over 100 consecutive trials.
    """

    metadata = {
        'render.modes': ['human']
    }

    def __init__(self):
        ############################# Initialize the simulation #################################

        urdf_path = os.path.join(os.environ["HOME"], "wdc_workspace/src/jiminy/data/cartpole/cartpole.urdf")
        contacts = []
        motors = ["slider_to_cart"]
        self.model = jiminy.model()
        self.model.initialize(urdf_path, contacts, motors, False)
        self.model.add_encoder_sensor("slider", "slider_to_cart")
        self.model.add_encoder_sensor("pole", "cart_to_pole")
        self.engine_py = engine_asynchronous(self.model)

        ########################### Configuration the simulation ################################

        self.dt = 2.0e-3 # The time step of the 'step' method

        model_options = self.model.get_model_options()
        sensors_options = self.model.get_sensors_options()
        engine_options = self.engine_py.get_engine_options()
        ctrl_options = self.engine_py.get_controller_options()

        model_options["telemetry"]["enableImuSensors"] = False
        engine_options["telemetry"]["enableConfiguration"] = False
        engine_options["telemetry"]["enableVelocity"] = False
        engine_options["telemetry"]["enableAcceleration"] = False
        engine_options["telemetry"]["enableCommand"] = False
        engine_options["telemetry"]["enableEnergy"] = False

        engine_options["stepper"]["solver"] = "runge_kutta_dopri5" # ["runge_kutta_dopri5", "explicit_euler"]
        engine_options["stepper"]["iterMax"] = -1 #Infinite number of iteration
        engine_options["stepper"]["sensorsUpdatePeriod"] = self.dt
        engine_options["stepper"]["controllerUpdatePeriod"] = self.dt

        self.model.set_model_options(model_options)
        self.model.set_sensors_options(sensors_options)
        self.engine_py.set_engine_options(engine_options)
        self.engine_py.set_controller_options(ctrl_options)

        ########################### Configuration the learning ################################

        # Force mag of the action
        self.force_mag = 200.0

        # Angle at which to fail the episode
        self.theta_threshold_radians = 25 * math.pi / 180
        self.x_threshold = 0.75

        # Angle limit set to 2 * theta_threshold_radians so failing observation is still within bounds
        high = np.array([self.x_threshold * 2,
                         np.finfo(np.float32).max,
                         self.theta_threshold_radians * 2,
                         np.finfo(np.float32).max])

        self.action_space = spaces.Discrete(2) # action can be either 0 or 1
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self.seed()
        self.viewer = None

        self.steps_beyond_done = None

    def seed(self, seed=None):
        self.engine_py.seed(seed)
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def reset(self):
        state = self.np_random.uniform(low=-1, high=1, size=(4,1)) * np.array([[0.5, 0.15, 0.1, 0.1]]).T
        self.engine_py.reset(state)
        self.steps_beyond_done = None
        return self.engine_py.state

    def render(self, lock=None, mode='human'):
        self.engine_py.render(lock)
        if (self.viewer is None):
            self.viewer = self.engine_py._client
        return RenderMockOut()

    def close(self):
        self.engine_py.close()

    def step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))

         # Bypass check and use direct assignment to max out the performances
        # self.engine_py.step(np.array([[action * self.force_mag]]))
        if action == 1:
            self.engine_py._action[0] = self.force_mag
        else:
            self.engine_py._action[0] = -self.force_mag
        self.engine_py.step()
        x, theta, x_dot, theta_dot = self.engine_py.state

        done =  x < -self.x_threshold \
                or x > self.x_threshold \
                or theta < -self.theta_threshold_radians \
                or theta > self.theta_threshold_radians

        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            # Pole just fell!
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warn("You are calling 'step()' even though this environment has already returned done = True. You should always call 'reset()' once you receive 'done = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0

        return self.engine_py.state, reward, done, {}
