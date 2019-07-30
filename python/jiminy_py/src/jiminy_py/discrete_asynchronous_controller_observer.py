from collections import OrderedDict

import numpy as np

class discrete_asynchronous_controller_observer:
    def __init__(self, model):
        # Make sure that the sensors have already been added to the model !

        self.sensors_types = model.get_sensors_options().keys()
        self.state = np.zeros((len(model.motors_names),1))
        self.observation = OrderedDict((sensor_type,[]) for sensor_type in self.sensors_types)
        self.action = np.zeros((len(model.motors_names),1))

        self._controller = jiminy.controller_functor(
            self._send_command, self._internal_dynamics, len(self.sensors_types))
        self._controller.initialize(model)

    def update_action(self, action_next):
        self.action[:] = action_next

    def _send_command(self, t, q, v, *args):
        self.state = np.concatenate((q, v))
        for k, sensor_type in enumerate(self.observation):
            self.observation[sensor_type] = args[k]
        uCommand = args[-1]
        uCommand[:] = self.action

    def _internal_dynamics(self, t, q, v, *args):
        pass