import os
from collections import OrderedDict
import tempfile

import numpy as np

import jiminy
import jiminy_py

import libpinocchio_pywrap as pin
from pinocchio.robot_wrapper import RobotWrapper

class engine_asynchronous(object):
    def __init__(self, model):
        # Make sure that the sensors have already been added to the model !

        self._sensors_types = model.get_sensors_options().keys()
        self._state = np.zeros((model.nx, 1))
        self._observation = OrderedDict((sensor_type,[]) for sensor_type in self._sensors_types)
        self._action = np.zeros((len(model.motors_names), 1))

        # Instantiate the Jiminy controller
        self._controller = jiminy.controller_functor(
            self._send_command, self._internal_dynamics, len(self._sensors_types))
        self._controller.initialize(model)

        # Instantiate the Jiminy engine
        self._engine = jiminy.engine()
        self._engine.initialize(model, self._controller) # model and controller are pass-by-reference
        self.reset()

        self.is_gepetto_available = False
        self._client = None
        self._viewer_proc = None
        self._id = None
        self._rb = None

    def _send_command(self, t, q, v, *args):
        for k, sensor_type in enumerate(self._observation):
            self._observation[sensor_type] = args[k]
        uCommand = args[-1]
        uCommand[:] = self._action

    def _internal_dynamics(self, t, q, v, *args):
        pass

    def seed(self, seed=None):
        if (seed is not None):
            engine_options = self._engine.get_options()
            engine_options["stepper"]["randomSeed"] = seed
            self._engine.set_options(engine_options)

    def reset(self, x0=None):
        self._state = None
        if (x0 is None):
            x0 = np.zeros((self._engine.model.nx, 1))
        if (int(self._engine.reset(x0)) != 1):
            raise ValueError("Reset of engine failed.")

    def step(self, action_next=None):
        if (action_next is not None):
            self.action = action_next
        self._state = None
        return self._engine.step()

    def render(self, lock=None):
        if (lock is not None):
            lock.acquire()
        try:
            # Instantiate the Gepetto model and viewer client if necessary
            if (not self.is_gepetto_available):
                self._client, self._viewer_proc = jiminy_py.get_gepetto_client(True)
                self._id = next(tempfile._get_candidate_names())
                self._rb = RobotWrapper()
                collision_model = pin.buildGeomFromUrdf(self._engine.model.pinocchio_model,
                                                        self._engine.model.urdf_path, [],
                                                        pin.GeometryType.COLLISION)
                visual_model = pin.buildGeomFromUrdf(self._engine.model.pinocchio_model,
                                                    self._engine.model.urdf_path, [],
                                                    pin.GeometryType.VISUAL)
                self._rb.__init__(model=self._engine.model.pinocchio_model,
                                collision_model=collision_model,
                                visual_model=visual_model)
                self.is_gepetto_available = True

            # Load model in gepetto viewer if needed
            if not self._id in self._client.gui.getSceneList():
                self._client.gui.createSceneWithFloor(self._id)
                window_id = self._client.gui.createWindow("jiminy")
                self._client.gui.addSceneToWindow(self._id, window_id)
                self._client.gui.createGroup(self._id + '/' + self._id)
                self._client.gui.addLandmark(self._id + '/' + self._id, 0.1)

                self._rb.initDisplay("jiminy", self._id, loadModel=False)
                self._rb.loadDisplayModel(self._id + '/' + "robot")

            # Update viewer
            jiminy_py.update_gepetto_viewer(self._rb, self._engine.model.pinocchio_data,
                                            self._client, True)
        finally:
            if (lock is not None):
                lock.release()

    def close(self):
        if (self._viewer_proc is not None):
            os.killpg(os.getpgid(self._viewer_proc.pid), signal.SIGTERM)
            self._viewer_proc = None

    @property
    def state(self):
        if (self._state is None):
            # Get x by value, then convert the matrix column into an actual 1D array by reference
            self._state = self._engine.stepper_state.x.A1
        return self._state

    @property
    def observation(self):
        return self._observation

    @property
    def action(self):
        return self._action

    @action.setter
    def action(self, action_next):
        if (not isinstance(action_next, (np.ndarray, np.generic))
        or action_next.size != len(self._action) ):
            raise ValueError("The action must be a numpy array with the right dimension.")
        self._action[:] = action_next

    def get_engine_options(self):
        return self._engine.get_options()

    def set_engine_options(self, options):
        self._engine.set_options(options)

    def get_controller_options(self):
        return self._controller.get_options()

    def set_controller_options(self, options):
        self._controller.set_options(options)
