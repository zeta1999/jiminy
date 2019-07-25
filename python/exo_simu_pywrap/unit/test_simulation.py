import time
from math import *
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

################################## User parameters #######################################

urdf_path = "/home/builder/.simulation/atalante_with_patient/atalante_with_patient.urdf"
urdf_mesh_path = "/home/builder/.simulation"
neural_network_path = "/home/builder/wdc_workspace/src/wandercode/test_data/data/trajectories/generic_walk_network_v5-4.json"
traj_features = {"steplength":  16,
                 "duration":    0.95,
                 "stairheight": 0.0}

############### Compute the reference trajectory using a neural network #################

# Get the patient information
patient_info = get_patient_info(urdf_path)

# Load the generic neural network
network = load_specialized_walk_network(neural_network_path, patient_info)[0]
x = np.array([OrderedDict(sorted(traj_features.items(), key=lambda t: INPUT_ORDER_NN[t[0]])).values()]).T
pred = np.ravel(np.asarray(network.eval(x)))

# Compute the trajectory data from the prediction vector
trajectory_data = extract_state_from_neural_network_prediction(urdf_path, pred)

########################### Configuration the simulation ################################

simulator = exo_simu.simulator()
simulator.init(urdf_path)

model_options = simulator.get_model_options()
sensors_options = simulator.get_sensors_options()
simu_options = simulator.get_engine_options()
ctrl_options = simulator.get_controller_options()

ctrl_options["telemetryEnable"] = True
model_options["telemetry"]["enableForceSensors"] = True
model_options["telemetry"]["enableImuSensors"] = True
model_options["telemetry"]["enableEncoderSensors"] = False
simu_options["telemetry"]["enableConfiguration"] = True
simu_options["telemetry"]["enableVelocity"] = True
simu_options["telemetry"]["enableAcceleration"] = True
simu_options["telemetry"]["enableCommand"] = True
simu_options["telemetry"]["enableEnergy"] = True

# simu_options["world"]["gravity"][2] = 0

simu_options["stepper"]["solver"] = "runge_kutta_dopri5" # ["runge_kutta_dopri5", "explicit_euler"]
simu_options["stepper"]["tolRel"] = 1.0e-5
simu_options["stepper"]["tolAbs"] = 1.0e-4
simu_options["stepper"]["dtMax"] = 2.0e-3 # 2.0e-4 for "explicit_euler", 3.0e-3 for "runge_kutta_dopri5"
simu_options["stepper"]["iterMax"] = 100000
simu_options["stepper"]["sensorsUpdatePeriod"] = 0.0
simu_options["stepper"]["controllerUpdatePeriod"] = 0.0
simu_options["stepper"]["randomSeed"] = 0

simu_options['contacts']['stiffness'] = 1.0e6
simu_options['contacts']['damping'] = 2000.0
simu_options['contacts']['dryFrictionVelEps'] = 0.01
simu_options['contacts']['frictionDry'] = 5.0
simu_options['contacts']['frictionViscous'] = 5.0
simu_options['contacts']['transitionEps'] = 0.001

model_options["dynamics"]["inertiaBodiesBiasStd"] = 0.0
model_options["dynamics"]["massBodiesBiasStd"] = 0.0
model_options["dynamics"]["centerOfMassPositionBodiesBiasStd"] = 0.0
model_options["dynamics"]["relativePositionBodiesBiasStd"] = 0.0
model_options["dynamics"]["enableFlexibleModel"] = True
model_options["dynamics"]["flexibleJointsNames"] = ["RightTransverseHipJoint"]
model_options["dynamics"]["flexibleJointsStiffness"] = [np.array([[1.0e5, 1.0e5, 1.0e5]]).T]
model_options["dynamics"]["flexibleJointsDamping"] = [np.array([[1.0e1, 1.0e1, 1.0e1]]).T]

# for sensorOptions in sensors_options['ImuSensor'].values():
#     sensorOptions['rawData'] = True
#     sensorOptions['noiseStd'] = [5.0e-2, 4.0e-2, 0.0, 0.0, 0.0, 0.0]
# sensors_options['ImuSensor'][sensors_options['ImuSensor'].keys()[0]]['bias'] = [-8.0e-2, +9.0e-2, 0.0, 0.0, 0.0, 0.0]
# sensors_options['ImuSensor'][sensors_options['ImuSensor'].keys()[0]]['delay'] =  2.0e-3
# sensors_options['ImuSensor'][sensors_options['ImuSensor'].keys()[0]]['delayInterpolationOrder'] = 0

simulator.set_model_options(model_options)
simulator.set_sensors_options(sensors_options)
simulator.set_engine_options(simu_options)
simulator.set_controller_options(ctrl_options)

################################ Run the simulation #####################################

Kp = np.array([20000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,
               20000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0])
Kd = np.array([250.0, 150.0, 100.0, 100.0, 150.0, 100.0,
               250.0, 150.0, 100.0, 100.0, 150.0, 100.0])
controller = pid_feedforward(simulator, trajectory_data, Kp, Kd)

def callback(t, x, out):
    out[0] = x[2] > 0 # [0] is needed to avoid assignment (here it copies the left value into the right reference instead)

x0 = get_initial_state_simulation(trajectory_data)
tf = 3.0

# simulator.register_force_impulse("PelvisLink", 1.5, 10.0e-3, np.array([[1.0e3, 0.0, 0.0]]).T)
# simulator.register_force_impulse("PelvisLink", 2.2, 20.0e-3, np.array([[0.0, 1.0e3, 0.0]]).T)
# def forceFct(t, x, out):
#     out[0] = 1.0e2 * sin(2 * pi * (t / 0.5))
#     out[1] = 1.0e2 * cos(2 * pi * (t / 0.5))
# simulator.register_force_profile("PelvisLink", forceFct)

controller.reset()
simulator.run(x0, 5e-2, controller.compute_command, callback) # Force compile Python controller for a fair benchmark

controller.reset()
start = time.time()
simulator.run(x0, tf, controller.compute_command, callback)
end = time.time()
print("Simulation time: %03.0fms" %((end - start)*1.0e3))

############################### Extract the results #####################################

log_info, log_data = simulator.get_log()
log_info = list(log_info)
log_data = np.asarray(log_data)
log_constants = log_info[1:log_info.index('StartColumns')]
log_header = log_info[(log_info.index('StartColumns')+1):-1]

print('%i log points' % log_data.shape[0])
print(log_constants)
pinocchio_model = simulator.get_pinocchio_model()
trajectory_data_log = extract_state_from_simulation_log(urdf_path, log_header, log_data, pinocchio_model)

nb_steps = int(trajectory_data_log['evolution_robot'][-1].t/trajectory_data['evolution_robot'][-1].t)
trajectory_data_ref = get_n_steps(trajectory_data, nb_steps)

############################## Display the results ######################################

# Plot some data using standard tools only
# plt.plot(log_data[:,["Global.Time" in field for field in log_header]],
#          log_data[:,[sensors_options['ImuSensor'].keys()[0] in field for field in log_header]])
# plt.show()

# Display the simulation trajectory and the reference
# play_trajectories([trajectory_data_ref, trajectory_data_log],
#                   xyz_offset=[None, None],
#                   urdf_rgba=[(1.0,0.0,0.0,0.5), None],
#                   speed_ratio=0.5)

# Display some kinematics data
# plot_kinematics(trajectory_data_log, trajectory_data_ref)

# Save the log in TSV
# simulator.write_log("/tmp/blackbox/log.data", True)
# log = LogFile("/tmp/blackbox/log.data")

# Plot some data using logviewer
# plt.plot(log.parser.get_data("Global.Time"), log.parser.get_data("PelvisIMU.Gyroy"))
# plt.show()