import time
import xmltodict
import numpy as np
import matplotlib.pyplot as plt
from numba import jit # Use to precompile Python code

import exo_simu

import pinocchio as pnc
from pinocchio.robot_wrapper import RobotWrapper
from wdc_dynamicsteller import DynamicsTeller, rootjoint

from wdc.math_py.neural_network import *

from exo_simu_py import *

EXO_THIGH_RPY = [-7/180*np.pi, 0, 0]
EXO_DEFAULT_THIGH_SETTINGS_MM = 420
JOINT_ORDER = ["RightFrontalHipJoint",
               "RightTransverseHipJoint",
               "RightSagittalHipJoint",
               "RightSagittalKneeJoint",
               "RightSagittalAnkleJoint",
               "RightHenkeAnkleJoint",
               "LeftFrontalHipJoint",
               "LeftTransverseHipJoint",
               "LeftSagittalHipJoint",
               "LeftSagittalKneeJoint",
               "LeftSagittalAnkleJoint",
               "LeftHenkeAnkleJoint"]

urdf_path = "/home/builder/.simulation/atalante_with_patient/atalante_with_patient.urdf"
urdf_mesh_path = "/home/builder/.simulation"
neural_network_path = "/home/builder/wdc_workspace/src/wandercode/test_data/data/trajectories/generic_walk_network_v5-4.json"

##########################################################################################

# Get patient information and robot settings from the URDF file
patient_info = {}
with open(urdf_path,'r') as urdf_file:
    urdf_data = xmltodict.parse(urdf_file.read())
patient_info['patient_height'] = float(urdf_data['robot']['patient_info']['patient_height'])
patient_info["patient_weight"] = float(urdf_data['robot']['patient_info']['patient_mass'])

thigh_coord = np.fromstring([joint for joint in urdf_data['robot']['joint'] 
                             if joint['@name'] == 'RightSagittalKneeJoint'][0]['origin']['@xyz'], dtype=float, sep=' ')
shank_coord = np.fromstring([joint for joint in urdf_data['robot']['joint'] 
                             if joint['@name'] == 'RightSagittalAnkleJoint'][0]['origin']['@xyz'], dtype=float, sep=' ')
thigh_adjust = np.sign(thigh_coord[2])*thigh_coord - np.array([0,0,np.cos(0*EXO_THIGH_RPY[0])*EXO_DEFAULT_THIGH_SETTINGS_MM*1e-3])
patient_info["thigh_setting"] = 1e3 * (EXO_DEFAULT_THIGH_SETTINGS_MM*1e-3 + np.sign(thigh_adjust[2])*np.linalg.norm(thigh_adjust[2]))
patient_info["shank_setting"] = - 1e3 * shank_coord[2]

## Initialize RobotWrapper and DynamicsTeller
rbdt = DynamicsTeller.make(urdf_path, rootjoint.FREEFLYER)
# rb = RobotWrapper()
# rb.initFromURDF(urdf_path, urdf_mesh_path, pnc.JointModelFreeFlyer())
rb = pnc.RobotWrapper()
rb.initFromURDF(urdf_path, urdf_mesh_path, root_joint=pnc.JointModelFreeFlyer())

# Load the generic neural network
network = load_specialized_walk_network(neural_network_path, patient_info)[0]
support_foot = 'RightSole'
input_order = {"steplength": 1, "duration": 2, "stairheight": 3}
x = np.array([[16,0.95,0.0]]).T
pred = np.ravel(np.asarray(network.eval(x)))
t = np.linspace(0,pred[1],200)
q = np.reshape(pred[42:], (12,-1))

# Post-Processing: Filtering and numerical derivation
q = np.reshape(pred[42:], (12,-1))
params = dict()
params['mixing_ratio_1'] = 0.04
params['mixing_ratio_2'] = 0.04
params['smoothness'] = [0.0,0.0,0.0]
params['smoothness'][0]  = 2e-9
params['smoothness'][1]  = 2e-9
params['smoothness'][2]  = 2e-9
relabel = np.zeros((12,12))
relabel[6:,:6] = relabel[:6,6:] = np.diag([-1,-1,1,1,1,-1])
q = smoothing_filter(t, q, t, relabel=relabel, params=params)
t_tmp  = (t[:-1] + t[1:])/2
dq  = np.diff(q,n=1,axis=1) / np.diff(t)
dq = smoothing_filter(t_tmp, dq, t)
t_tmp = (t[:-1] + t[1:])/2
ddq = np.diff(dq,n=1,axis=1) / np.diff(t)
ddq = smoothing_filter(t_tmp, ddq, t)

# Compute the free-flyer and feedforward torques

## Reorder the joints
qe = np.zeros((rbdt.getModelPositionSize(), q.shape[1]))
dqe = np.zeros((rbdt.getModelVelocitySize(), q.shape[1]))
ddqe = np.zeros((rbdt.getModelVelocitySize(), q.shape[1]))
for i in range(len(JOINT_ORDER)):
    qe[rbdt.getJointIdInPosition(JOINT_ORDER[i])] = q[i]
    dqe[rbdt.getJointIdInVelocity(JOINT_ORDER[i])] = dq[i]
    ddqe[rbdt.getJointIdInVelocity(JOINT_ORDER[i])] = ddq[i]

# from wdc.walk_py.wb_converter import WbConverter
# wb_converter = WbConverter(rbdt)
# model_positions  = np.zeros((rbdt.getModelPositionSize(), q.shape[1]))
# model_velocities = np.zeros((rbdt.getModelVelocitySize(), q.shape[1]))
# model_accelerations = np.zeros((rbdt.getModelVelocitySize(), q.shape[1]))
# for i in range(q.shape[1]):
#     mp, mv = wb_converter.joints_to_model(support_foot, se3.SE3.Identity(), qe[qe_mask][:, [i]], dqe[qe_mask][:, [i]])
#     ma = wb_converter.joints_to_model_acceleration(support_foot, se3.SE3.Identity(), qe[qe_mask][:, [i]], dqe[qe_mask][:, [i]], ddqe[qe_mask][:, [i]])

#     model_positions[:, [i]] = mp
#     model_velocities[:, [i]] = mv
#     model_accelerations[:, [i]] = ma

u = np.zeros((12,len(t)))
f_ext = np.zeros((6,len(t)))
wrench = np.zeros((12,len(t)))
qe_mask = np.concatenate((np.arange(7,13),np.arange(14,20)))
dqe_mask = np.concatenate((np.arange(6,12),np.arange(13,19)))
root_joint_idx = rb.model.getJointId("root_joint")
for i in range(q.shape[1]):
    qe_i, dqe_i, ddqe_i, u_i, f_ext_i, wrench_i = \
        qe[:,[i]], dqe[:,[i]], ddqe[:,[i]], u[:,[i]], f_ext[:,[i]], wrench[:,[i]]

    ## Compute freeflyer using support foot as reference frame.
    qe_i, dqe_i, ddqe_i = rbdt.computeFreeflyerStateFromFixedBody(support_foot, qe_i, dqe_i, ddqe_i)

    ## Compute the efforts at each time step
    ### Apply a first run of rnea without explicit external forces
    pnc.computeJointJacobians(rb.model, rb.data, qe_i)
    pnc.rnea(rb.model, rb.data, qe_i, dqe_i, ddqe_i)
    ### Initialize vector of exterior forces to 0
    fs_i = pnc.StdVec_Force()
    fs_i.extend([pnc.Force(np.matrix([0.0, 0, 0, 0, 0, 0]).T) for _ in range(len(rb.model.names))])
    ### Compute the force at the henke level
    support_foot_idx = rb.model.frames[rb.model.getBodyId(support_foot)].parent
    fs_i[support_foot_idx] = rb.data.oMi[support_foot_idx].actInv(rb.data.oMi[root_joint_idx]).act(rb.data.f[root_joint_idx])
    ### Recompute the efforts with RNEA and the correct external force
    u_i = pnc.rnea(rb.model, rb.data, qe_i, dqe_i, ddqe_i, fs_i)[dqe_mask]
    ### Get the external forces applied on the support foot
    f_ext_tmp = fs_i[support_foot_idx]
    ha_M_s = rb.model.frames[rb.model.getBodyId(support_foot)].placement
    f_ext_i = ha_M_s.actInv(f_ext_tmp).vector

    qe[:,[i]], dqe[:,[i]], ddqe[:,[i]], u[:,[i]], f_ext[:,[i]], wrench[:,[i]] = \
        qe_i, dqe_i, ddqe_i, u_i, f_ext_i, wrench_i

# zmp = np.vstack((-f_ext[4,:] / f_ext[2,:], f_ext[3,:] / f_ext[2,:]))

# plt.plot(t, f_ext.T) ; 
# plt.legend(['1','2','3','4','5','6'])
# plt.show()

# print(pnc.rnea(rb.model, rb.data, qe_i, dqe_i, ddqe_i))
# print(pnc.rnea(rb.model, rb.data, qe_i, dqe_i, ddqe_i, fs_i))
# plt.plot(t, u.T) ; 
# plt.legend([item for i,item in enumerate(rb.model.names) if i in np.concatenate((np.arange(2,8),np.arange(9,15)))])
# plt.show()
  
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

Kp = np.array([[41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0,
                41000.0, 16000.0, 16000.0, 32000.0, 4500.0, 3500.0]]).T
Kd = np.array([[500.0, 160.0, 120.0, 270.0, 15.0, 20.0, 
                500.0, 160.0, 120.0, 270.0, 15.0, 20.0]]).T
@jit(nopython=True)
def controller(t_cur, x, optoforces, IMUs):
    t_ind_min = get_closest_left_ind(t, t_cur)
    t_ind_max = t_ind_min + 1
    ratio = (t_cur - t[t_ind_min])/(t[t_ind_max] - t[t_ind_min])
    q_ref = np.atleast_2d((1 - ratio) * qe[qe_mask,t_ind_min] + ratio * qe[qe_mask,t_ind_max]).T
    dq_ref = np.atleast_2d((1 - ratio) * dqe[dqe_mask,t_ind_min] + ratio * dqe[dqe_mask,t_ind_max]).T
    # ddq_ref = np.atleast_2d((1 - ratio) * ddqe[dqe_mask,t_ind_min] + ratio * ddqe[dqe_mask,t_ind_max]).T
    u_ref = np.atleast_2d((1 - ratio) * u[:,t_ind_min] + ratio * u[:,t_ind_max]).T

    return - (Kp * (np.asarray(x)[7:19] - q_ref) + Kd * (np.asarray(x)[25:37] - dq_ref)) + u_ref

def callback(t, x):
    return (t <= 2.5)

x0 = np.concatenate((qe[0:3,0],np.array([1]),qe[3:6,0]/qe[6,0],qe[qe_mask,0],
                     dqe[0:6,0],dqe[dqe_mask,0]))[:,np.newaxis]
x0[2] -= 0.02
t0 = 0.0
tf = 3.0
dt = 1e-3

simulator.simulate(x0,t0,0.1,dt,controller,callback) # Force compile Python controller for a fair benchmark
start = time.time()
simulator.simulate(x0,t0,tf,dt,controller,callback)
end = time.time()
print("Simulation time: %03.0fms" %((end - start)*1.0e3))

log = np.array(simulator.get_log())
print(log.shape)
log_data = visualize(log,speed_ratio=0.5)

rb.initDisplay()
rb.loadDisplayModel("world/wdc_robot")
init_time = time.time()
for i in range(len(t)):
    rb.display(qe[:,[i]])
    t_simu = time.time() - init_time
    if t_simu < t[i]:
        time.sleep(t[i] - t_simu)

# plt.figure()
# plt.plot(log[:,0],log[:,3], '*-')
# plt.show()