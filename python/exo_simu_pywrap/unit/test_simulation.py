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
traj_features = {"steplength":  16, 
                 "duration":    0.95,
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

simulator = exo_simu.simulator()
model_options = simulator.get_model_options()
simu_options = simulator.get_simulation_options()

# simu_options["world"]["gravity"][2] = 0

simu_options["stepper"]["tolRel"] = 1.0e-5
simu_options["stepper"]["tolAbs"] = 1.0e-4
simu_options["stepper"]["sensorsUpdatePeriod"] = 0.0
simu_options["stepper"]["controllerUpdatePeriod"] = 0.0

simu_options['contacts']['stiffness'] = 1.0e6
simu_options['contacts']['damping'] = 2000.0
simu_options['contacts']['dryFrictionVelEps'] = 0.01
simu_options['contacts']['frictionDry'] = 5.0
simu_options['contacts']['frictionViscous'] = 5.0
simu_options['contacts']['transitionEps'] = 0.001

simulator.init(urdf_path)
simulator.set_model_options(model_options)
simulator.set_simulation_options(simu_options)

def callback(t, x):
    return bool(x[2] > 0) 

x0 = get_initial_state_simulation(trajectory_data)
tf = 3.0

Kp = np.array([[20000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,
                20000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0]]).T
Kd = np.array([[250.0, 150.0, 100.0, 100.0, 150.0, 100.0, 
                250.0, 150.0, 100.0, 100.0, 150.0, 100.0]]).T
controller = pid_feedforward(trajectory_data, Kp, Kd)
simulator.simulate(x0, 5e-2, controller.compute_command, callback) # Force compile Python controller for a fair benchmark
controller.reset()
start = time.time()
simulator.simulate(x0, tf, controller.compute_command, callback)
end = time.time()
print("Simulation time: %03.0fms" %((end - start)*1.0e3))

################################## Display the results ##################################

log = np.asarray(simulator.get_log()[1])
print('%i log points' % log.shape[0])
trajectory_data_log = extract_state_from_simulation_log(urdf_path, log)

nb_steps = int(trajectory_data_log['evolution_robot'][-1].t/trajectory_data['evolution_robot'][-1].t)
trajectory_data_ref = get_n_steps(trajectory_data, nb_steps)

# Display the simulation trajectory and the reference
# play_trajectories([trajectory_data_ref, trajectory_data_log], xyz_offset=[None, None], urdf_rgba=[(1.0,0.0,0.0,0.5), None], speed_ratio=0.5)

# plot_kinematics(trajectory_data_log, trajectory_data_ref)