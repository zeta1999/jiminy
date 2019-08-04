import os
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

import wdc_jiminy
from wdc_jiminy_py import *

################################## User parameters #######################################

wdc_root_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
urdf_path = os.path.join(wdc_root_dir, "data/atalante_with_patient/atalante_with_patient.urdf")
neural_network_path = os.path.join(wdc_root_dir, "data/atalante_with_patient/generic_walk_network_v5-4.json")
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

############################# Initialize the simulation #################################

# Instantiate the model
model = wdc_jiminy.exo_model()
model.initialize(urdf_path)

# Instantiate the controller
controller = wdc_jiminy.exo_controller()

Kp = np.array([20000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,
               20000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0])
Kd = np.array([225.0, 135.0, 100.0, 100.0, 135.0, 100.0,
               225.0, 135.0, 100.0, 100.0, 135.0, 100.0])
pid_controller_py = pid_feedforward(model, controller, trajectory_data, Kp, Kd)

controller.initialize(model, pid_controller_py.compute_command)

# Instantiate the engine
engine = jiminy.engine()

def callback(t, x, out):
    out[0] = x[2] > 0 # [0] is needed to avoid assignment (here it copies the left value into the right reference instead)

engine.initialize(model, controller, callback)

########################### Configuration the simulation ################################

model_options = model.get_model_options()
sensors_options = model.get_sensors_options()
engine_options = engine.get_options()
ctrl_options = controller.get_options()

ctrl_options["telemetryEnable"] = True
model_options["telemetry"]["enableForceSensors"] = True
model_options["telemetry"]["enableImuSensors"] = True
model_options["telemetry"]["enableEncoderSensors"] = False
engine_options["telemetry"]["enableConfiguration"] = True
engine_options["telemetry"]["enableVelocity"] = True
engine_options["telemetry"]["enableAcceleration"] = True
engine_options["telemetry"]["enableCommand"] = True
engine_options["telemetry"]["enableEnergy"] = True

# engine_options["world"]["gravity"][2] = 0

engine_options["stepper"]["odeSolver"] = "runge_kutta_dopri5" # ["runge_kutta_dopri5", "explicit_euler"]
engine_options["stepper"]["tolRel"] = 1.0e-5
engine_options["stepper"]["tolAbs"] = 1.0e-4
engine_options["stepper"]["dtMax"] = 2.0e-3 # 2.0e-4 for "explicit_euler", 3.0e-3 for "runge_kutta_dopri5"
engine_options["stepper"]["iterMax"] = 100000
engine_options["stepper"]["sensorsUpdatePeriod"] = 0.0
engine_options["stepper"]["controllerUpdatePeriod"] = 0.0
engine_options["stepper"]["randomSeed"] = 0

engine_options['contacts']['stiffness'] = 1.0e6
engine_options['contacts']['damping'] = 2000.0
engine_options['contacts']['dryFrictionVelEps'] = 0.01
engine_options['contacts']['frictionDry'] = 5.0
engine_options['contacts']['frictionViscous'] = 5.0
engine_options['contacts']['transitionEps'] = 0.001

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

model.set_model_options(model_options)
model.set_sensors_options(sensors_options)
engine.set_options(engine_options)
controller.set_options(ctrl_options)

################################ Run the simulation #####################################

x0 = get_initial_state_simulation(trajectory_data)
tf = 3.0

# engine.register_force_impulse("PelvisLink", 1.5, 10.0e-3, np.array([[1.0e3, 0.0, 0.0]]).T)
# engine.register_force_impulse("PelvisLink", 2.2, 20.0e-3, np.array([[0.0, 1.0e3, 0.0]]).T)
# def forceFct(t, x, out):
#     out[0] = 1.0e2 * sin(2 * pi * (t / 0.5))
#     out[1] = 1.0e2 * cos(2 * pi * (t / 0.5))
# engine.register_force_profile("PelvisLink", forceFct)

pid_controller_py.reset()
start = time.time()
engine.simulate(x0, tf)
end = time.time()
print("Simulation time: %03.0fms" %((end - start)*1.0e3))

############################### Extract the results #####################################

log_info, log_data = engine.get_log()
log_info = list(log_info)
log_data = np.asarray(log_data)
log_constants = log_info[1:log_info.index('StartColumns')]
log_header = log_info[(log_info.index('StartColumns')+1):-1]

print('%i log points' % log_data.shape[0])
print(log_constants)
trajectory_data_log = extract_state_from_simulation_log(log_header, log_data, urdf_path, model.pinocchio_model, True)

nb_steps = int(trajectory_data_log['evolution_robot'][-1].t/trajectory_data['evolution_robot'][-1].t)
trajectory_data_ref = get_n_steps(trajectory_data, nb_steps)

# Save the log in TSV
# engine.write_log("/tmp/blackbox/log.data", True)

############################## Display the results ######################################

# Plot some data using standard tools only
# plt.plot(log_data[:,log_header.index('Global.Time')],
#          log_data[:,[sensors_options['ImuSensor'].keys()[0] in field for field in log_header]])
# plt.show()

# Display the simulation trajectory and the reference
# play_trajectories([trajectory_data_ref, trajectory_data_log],
#                   xyz_offset=[None, None],
#                   urdf_rgba=[(1.0,0.0,0.0,0.5), None],
#                   speed_ratio=0.5)

# Display some kinematics data
# plot_kinematics(trajectory_data_log, trajectory_data_ref)

# Load the TSV log file
# log = LogFile("/tmp/blackbox/log.data")

# Plot some data using logviewer
# plt.plot(log.parser.get_data("Global.Time"), log.parser.get_data("PelvisIMU.Gyroy"))
# plt.show()