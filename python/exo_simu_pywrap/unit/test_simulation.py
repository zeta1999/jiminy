import time
import numpy as np
import matplotlib.pyplot as plt
from numba import jit # Use to precompile Python code

import pinocchio as pnc
from pinocchio.robot_wrapper import RobotWrapper

from wdc_dynamicsteller import DynamicsTeller, rootjoint
from wdc.math_py.neural_network import *
from wdc.effort_assessment.plotting_tools import *

import exo_simu
from exo_simu_py import *

##########################################################################################

urdf_path = "/home/builder/.simulation/atalante_with_patient/atalante_with_patient.urdf"
urdf_mesh_path = "/home/builder/.simulation"
neural_network_path = "/home/builder/wdc_workspace/src/wandercode/test_data/data/trajectories/generic_walk_network_v5-4.json"
traj_features = {"steplength": 16, 
                 "duration": 0.95,
                 "stairheight": 0.0}

##########################################################################################

# Get the patient information
patient_info = get_patient_info(urdf_path)

# Load the generic neural network
network = load_specialized_walk_network(neural_network_path, patient_info)[0]
x = np.array([OrderedDict(sorted(traj_features.items(), key=lambda t: INPUT_ORDER_NN[t[0]])).values()]).T
pred = np.ravel(np.asarray(network.eval(x)))

# Extract the trajectory data from the prediction vector
trajectory_data = extract_state_from_neural_network_prediction(urdf_path, pred)

# Estimate the free-flyer
retrieve_freeflyer(trajectory_data)

# Compute the ground reaction forces and motor torques
compute_efforts_with_rnea(trajectory_data)
  
########################################################################################


class pid_feedforward:
    def __init__(self, trajectory_data_ref, Kp=None, Kd=None):
        if Kp is None:
            self.Kp = np.array([[41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0,
                                 41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0]]).T
        if Kd is None:
            self.Kd = np.array([[500.0, 160.0, 120.0, 270.0, 15.0, 20.0, 
                                 500.0, 160.0, 120.0, 270.0, 15.0, 20.0]]).T
        
        self.state_machine = dict()
        evolution_robot = trajectory_data_ref['evolution_robot']
        support_foot_ref = evolution_robot[0].support_foot
        state_ref = self.state_machine[support_foot_ref] = dict()
        state_ref['t'] = np.array([s.t for s in evolution_robot])
        state_ref['q'] = np.asarray(np.concatenate(
            [s.q[THOT_TO_SIMU_JOINT_MASK_POSITION] for s in evolution_robot], axis=1))
        state_ref['dq'] = np.asarray(np.concatenate(
            [s.v[THOT_TO_SIMU_JOINT_MASK_VELOCITY] for s in evolution_robot], axis=1))
        state_ref['ddq'] = np.asarray(np.concatenate(
            [s.a[THOT_TO_SIMU_JOINT_MASK_VELOCITY] for s in evolution_robot], axis=1))
        state_ref['u'] = np.asarray( np.concatenate(
            [s.tau.items()[0][1][THOT_TO_SIMU_JOINT_MASK_VELOCITY] for s in evolution_robot], axis=1))
        support_foot_next = [elem for elem in SUPPORT_FOOT_ENUM if support_foot_ref not in elem][0]
        state_next = self.state_machine[support_foot_next] = dict()
        state_next['t'] = state_ref['t']
        state_next['q'] = RELABELING_JOINT_MATRIX.dot(state_ref['q'])
        state_next['dq'] = RELABELING_JOINT_MATRIX.dot(state_ref['dq'])
        state_next['ddq'] = RELABELING_JOINT_MATRIX.dot(state_ref['ddq'])
        state_next['u'] = RELABELING_JOINT_MATRIX.dot(state_ref['u'])

        self.time_offset = 0
        self.current_state = support_foot_ref

    def state_machine_switch_next(self, t_cur):
        self.current_state = [elem for elem in SUPPORT_FOOT_ENUM if self.current_state not in elem][0]
        self.time_offset = t_cur
        print("switching to next state: %s" % self.current_state)

    @staticmethod
    @jit(nopython=True)
    def _linear_interp(ratio, value_min, value_max):
        return np.atleast_2d((1 - ratio) * value_min + ratio * value_max).T

    # TODO: ODE Problem while going back and forth from the current state to the next because of adaptive step
    # TODO: Find a way to use numba jit with this method (or the entire class)
    def compute_command(self, t_cur, x, optoforces, IMUs):
        # Switch to next state if necessary and get information about the current state
        traj_ref = self.state_machine[self.current_state]
        t_rel = t_cur - self.time_offset
        if t_rel > traj_ref['t'][-1]:
            self.state_machine_switch_next(t_cur)
            traj_ref = self.state_machine[self.current_state]
            t_rel = t_cur - self.time_offset
        
        # Determine the relative time in the current state
        t_ind_min = get_closest_left_ind(traj_ref['t'], t_rel)
        t_ind_max = t_ind_min + 1
        ratio = (t_rel - traj_ref['t'][t_ind_min])/(traj_ref['t'][t_ind_max] - traj_ref['t'][t_ind_min])

        # Compute the value of the reference trajectory at that time
        q_ref = self._linear_interp(ratio, traj_ref['q'][:,t_ind_min], traj_ref['q'][:,t_ind_max])
        dq_ref = self._linear_interp(ratio, traj_ref['dq'][:,t_ind_min], traj_ref['dq'][:,t_ind_max])
        #ddq_ref = self._linear_interp(ratio, traj_ref['ddq'][:,t_ind_min], traj_ref['ddq'][:,t_ind_max])
        u_ref = self._linear_interp(ratio, traj_ref['u'][:,t_ind_min], traj_ref['u'][:,t_ind_max])

        # Compute PID torques
        u_pid = - (self.Kp * (np.asarray(x)[7:19] - q_ref) + self.Kd * (np.asarray(x)[25:37] - dq_ref))

        return u_ref + u_pid


########################################################################################

simulator = exo_simu.ExoSimulator()
model_options = simulator.get_model_options()
simu_options = simulator.get_simulation_options()

#model_options["gravity"][2] = 0
simu_options["tolRel"] = 1.0e-5
simu_options["tolAbs"] = 1.0e-4
simu_options["logController"] = False
simu_options["logOptoforces"] = False
simu_options["logIMUs"] = False

simulator.set_urdf_path(urdf_path)
simulator.set_model_options(model_options)
simulator.set_simulation_options(simu_options)

def callback(t, x):
    return True #bool(t <= trajectory_data['evolution_robot'][-1].t)

x0 = get_initial_state_simulation(trajectory_data)
t0 = 0.0
tf = 3.0
dt = 1e-3

controller = pid_feedforward(trajectory_data)
simulator.simulate(x0,t0,0.1,dt,controller.compute_command,callback) # Force compile Python controller for a fair benchmark
start = time.time()
simulator.simulate(x0,t0,tf,dt,controller.compute_command,callback)
end = time.time()
print("Simulation time: %03.0fms" %((end - start)*1.0e3))

log = np.array(simulator.get_log())
print(log.shape)
trajectory_data_log = extract_state_from_simulation_log(urdf_path, log)

# Display the simulation trajectory
display_robot(trajectory_data_log, speed_ratio=0.5)

# Display the reference trajectory
display_robot(trajectory_data, speed_ratio=0.5, scene_name="ref")