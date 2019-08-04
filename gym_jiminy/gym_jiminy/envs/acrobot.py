"""
Classic acrobot system simulated using Jiminy Engine
"""

import os
from math import sin, cos, pi
import numpy as np

from gym import core, spaces, logger
from gym.utils import seeding

import jiminy
from jiminy_py import engine_asynchronous
from gym_jiminy.common import RobotJiminyGoalEnv


class JiminyAcrobotEnv(RobotJiminyGoalEnv):
    """
    Acrobot is a 2-link pendulum with only the second joint actuated.
    Initially, both links point downwards. The goal is to swing the
    end-effector at a height at least the length of one link above the base.
    Both links can swing freely and can pass by each other, i.e., they don't
    collide when they have the same angle.

    **STATE:**
    The state consists of the sin() and cos() of the two rotational joint
    angles and the joint angular velocities :
    [cos(theta1) sin(theta1) cos(theta2) sin(theta2) thetaDot1 thetaDot2].
    For the first link, an angle of 0 corresponds to the link pointing downwards.
    The angle of the second link is relative to the angle of the first link.
    An angle of 0 corresponds to having the same angle between the two links.
    A state of [1, 0, 1, 0, ..., ...] means that both links point downwards.

    **ACTIONS:**
    The action is either applying +1, 0 or -1 torque on the joint between
    the two pendulum links.

    **REFERENCE:**
    .. seealso::
        R. Sutton: Generalization in Reinforcement Learning:
        Successful Examples Using Sparse Coarse Coding (NIPS 1996)
    .. seealso::
        R. Sutton and A. G. Barto:
        Reinforcement learning: An introduction.
        Cambridge: MIT press, 1998.
    """

    metadata = {
        'render.modes': ['human'],
    }

    def __init__(self):
        ############################### Configure the learning #################################

        # The time step of the 'step' method
        self.dt = 2.0e-3

        self.MAX_VEL_1 = 4 * pi # <= Should implement velocity bounds in Jiminy
        self.MAX_VEL_2 = 9 * pi

        self.AVAIL_TORQUE = [-1.0, 0.0, +1.0]

        self.torque_noise_max = 0.0

        # Force mag of the action
        self.torque_mag = 40.0

        high = np.array([1.0, 1.0, 1.0, 1.0, self.MAX_VEL_1, self.MAX_VEL_2])

        # self.action_space = spaces.Discrete(3)
        # self.observation_space = spaces.Dict(dict(
        #     desired_goal=,
        #     achieved_goal=,
        #     observation=spaces.Box(low=-high, high=high, dtype=np.float64),
        # ))

        ############################# Initialize the simulation ################################

        cur_dir = os.path.dirname(os.path.realpath(__file__))
        urdf_path = os.path.join(cur_dir, "../../../data/double_pendulum/double_pendulum.urdf")
        motors = ["SecondPendulumJoint"]
        self.model = jiminy.model()
        self.model.initialize(urdf_path, motors=motors)
        self.model.add_encoder_sensor(joint_name="PendulumJoint")
        self.model.add_encoder_sensor(joint_name="SecondPendulumJoint")
        self.engine_py = engine_asynchronous(self.model)

        ############################# Configure the simulation #################################

        model_options = self.model.get_model_options()
        sensors_options = self.model.get_sensors_options()
        engine_options = self.engine_py.get_engine_options()
        ctrl_options = self.engine_py.get_controller_options()

        model_options["telemetry"]["enableEncoderSensors"] = False
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

        ################################### Miscellaneous ######################################

        self._tipIdx = self.engine_py._engine.model.pinocchio_model.getFrameId("SecondPendulumMass")
        self._tipPosZMax = self.engine_py._engine.model.pinocchio_data.oMf[self._tipIdx].translation.A1[2]

        self.state = None
        self.viewer = None
        self.steps_beyond_done = None

        self.seed()

    def reset(self):
        stateOpenAI = self.np_random.uniform(low=-1, high=1, size=(4,)) * np.array([0.2, 0.2, 1.0, 1.0])
        alpha1, alpha2, alpha1_dot, alpha2_dot  = stateOpenAI
        theta1 = alpha1 - pi
        theta2 = alpha2 - pi
        theta1_dot = alpha1_dot
        theta2_dot = alpha2_dot
        stateJiminy = np.array([[theta1, theta2, theta1_dot, theta2_dot]]).T
        self.engine_py.reset(stateJiminy)
        self.steps_beyond_done = None
        self.state = self.engine_py.state
        return self._get_ob()

    def step(self, a):
        torque = self.AVAIL_TORQUE[a]

        # Add noise to the force action
        if self.torque_noise_max > 0:
            torque += self.np_random.uniform(-self.torque_noise_max, self.torque_noise_max)

        # Bypass 'self.engine_py.step' method and use direct assignment to max out the performances
        self.engine_py._action[0] = torque * self.torque_mag
        self.engine_py.step(dt_desired=self.dt)
        self.state = self.engine_py.state

        # Check the terminal condition and compute reward
        terminal = self._terminal()
        if not terminal:
            reward = -1.0
        elif self.steps_beyond_done is None:
            self.steps_beyond_done = 0
            reward = 0.0
        else:
            if self.steps_beyond_done == 0:
                logger.warn("You are calling 'step()' even though this environment has already returned terminal = True. You should always call 'reset()' once you receive 'terminal = True' -- any further steps are undefined behavior.")
            self.steps_beyond_done += 1
            reward = 0.0

        # Penalize the reward and terminate if the velocity to exceed the limits
        _, _, theta1_dot, theta2_dot  = self.state
        if theta1_dot < -self.MAX_VEL_1 or self.MAX_VEL_1 < theta1_dot \
        or theta2_dot < -self.MAX_VEL_2 or self.MAX_VEL_2 < theta2_dot:
            reward += -1e4
            terminal = True

        return self._get_ob(), reward, terminal, {}

    def _get_ob(self):
        theta1, theta2, theta1_dot, theta2_dot  = self.state
        theta1_dot = min(max(theta1_dot, -self.MAX_VEL_1), self.MAX_VEL_1)
        theta2_dot = min(max(theta2_dot, -self.MAX_VEL_2), self.MAX_VEL_2)
        return np.array([cos(theta1 + pi),
                         sin(theta1 + pi),
                         cos(theta2 + pi),
                         sin(theta2 + pi),
                         theta1_dot,
                         theta2_dot])

    def _terminal(self):
        tipPosZ = self.engine_py._engine.model.pinocchio_data.oMf[self._tipIdx].translation.A1[2]
        return tipPosZ > 0.9*self._tipPosZMax
