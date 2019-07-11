import time
import numpy as np
import matplotlib.pyplot as plt
from numba import jit # Use to precompile Python code

import pinocchio as pnc
from pinocchio.robot_wrapper import RobotWrapper

from wdc_dynamicsteller import DynamicsTeller, rootjoint
from wdc.math_py.neural_network import *
from wdc.effort_assessment.plotting_tools import *
from logviewer.logfile import LogFile

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
ctrl_options = simulator.get_controller_options()

ctrl_options["telemetry"]["logController"] = False
model_options["telemetry"]["logForceSensors"] = True
model_options["telemetry"]["logImuSensors"] = False
model_options["telemetry"]["logEncoderSensors"] = False
simu_options["telemetry"]["logConfiguration"] = True
simu_options["telemetry"]["logVelocity"] = True
simu_options["telemetry"]["logAcceleration"] = True
simu_options["telemetry"]["logCommand"] = True

# simu_options["world"]["gravity"][2] = 0

simu_options["stepper"]["tolRel"] = 1.0e-5
simu_options["stepper"]["tolAbs"] = 1.0e-4
simu_options["stepper"]["sensorsUpdatePeriod"] = 0.0
simu_options["stepper"]["controllerUpdatePeriod"] = 0.0
simu_options["stepper"]["randomSeed"] = 0

simu_options['contacts']['stiffness'] = 1.0e6
simu_options['contacts']['damping'] = 2000.0
simu_options['contacts']['dryFrictionVelEps'] = 0.01
simu_options['contacts']['frictionDry'] = 5.0
simu_options['contacts']['frictionViscous'] = 5.0
simu_options['contacts']['transitionEps'] = 0.001

model_options['ForceSensor']['noiseStd'] = 50.0

simulator.init(urdf_path)
simulator.set_model_options(model_options)
simulator.set_simulation_options(simu_options)
simulator.set_controller_options(ctrl_options)

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

log_info, log_data = simulator.get_log()
log_info = list(log_info)
log_data = np.asarray(log_data)
log_constants = log_info[1:log_info.index('StartColumns')]
log_header = log_info[(log_info.index('StartColumns')+1):-1]

print('%i log points' % log_data.shape[0])
print(log_constants)
trajectory_data_log = extract_state_from_simulation_log(urdf_path, log_header, log_data)

nb_steps = int(trajectory_data_log['evolution_robot'][-1].t/trajectory_data['evolution_robot'][-1].t)
trajectory_data_ref = get_n_steps(trajectory_data, nb_steps)

# Display the simulation trajectory and the reference
# play_trajectories([trajectory_data_ref, trajectory_data_log], xyz_offset=[None, None], urdf_rgba=[(1.0,0.0,0.0,0.5), None], speed_ratio=0.5)

# Display some kinematics data
# plot_kinematics(trajectory_data_log, trajectory_data_ref)

# Save the log in TSV
# simulator.write_log("/tmp/blackbox/log.data", True)
# log = LogFile("/tmp/blackbox/log.data")
# plt.plot(log.parser.get_data("Global.Time"),log.parser.get_data("ForceSensor.LeftExternalHeel.F_z"))
# plt.show()