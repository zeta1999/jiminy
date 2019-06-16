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

# %load_ext autoreload
# %autoreload 2

################################## User variables #######################################

urdf_path = "/home/builder/.simulation/atalante_with_patient/atalante_with_patient.urdf"
urdf_mesh_path = "/home/builder/.simulation"
neural_network_path = "/home/builder/wdc_workspace/src/wandercode/test_data/data/trajectories/generic_walk_network_v5-4.json"
traj_features = {"steplength": 16, 
                 "duration": 0.95,
                 "stairheight": 0.0}

############# Compute the reference trajectory using the neural network #################

# Get the patient information
patient_info = get_patient_info(urdf_path)

# Load the generic neural network
network = load_specialized_walk_network(neural_network_path, patient_info)[0]
x = np.array([OrderedDict(sorted(traj_features.items(), key=lambda t: INPUT_ORDER_NN[t[0]])).values()]).T
pred = np.ravel(np.asarray(network.eval(x)))

# Compute the trajectory data from the prediction vector
trajectory_data = extract_state_from_neural_network_prediction(urdf_path, pred)
  
################################ Simulate the system ####################################

simulator = exo_simu.ExoSimulator()
model_options = simulator.get_model_options()
simu_options = simulator.get_simulation_options()

# model_options["gravity"][2] = 0
simu_options["tolRel"] = 1.0e-5
simu_options["tolAbs"] = 1.0e-4
simu_options["logController"] = False
simu_options["logOptoforces"] = False
simu_options["logIMUs"] = False

simulator.set_urdf_path(urdf_path)
simulator.set_model_options(model_options)
simulator.set_simulation_options(simu_options)

def callback(t, x):
    return bool(x[2] > 0) 

x0 = get_initial_state_simulation(trajectory_data)
t0 = 0.0
tf = 5.0
dt = 1e-3

controller = pid_feedforward(trajectory_data)
simulator.simulate(x0,t0,0.1,dt,controller.compute_command,callback) # Force compile Python controller for a fair benchmark
start = time.time()
simulator.simulate(x0,t0,tf,dt,controller.compute_command,callback)
end = time.time()
print("Simulation time: %03.0fms" %((end - start)*1.0e3))

################################## Display the results ##################################

log = np.array(simulator.get_log())
print(log.shape)
trajectory_data_log = extract_state_from_simulation_log(urdf_path, log)

# Display the simulation trajectory
display_robot(trajectory_data_log, speed_ratio=0.5)

# Display the reference trajectory
# display_robot(trajectory_data, speed_ratio=0.5, scene_name="ref")
# delete_scenes_viewer('ref')

# plot_kinematics(trajectory_data_log, get_n_steps(trajectory_data, int(tf//trajectory_data['evolution_robot'][-1].t)))