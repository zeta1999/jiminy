#!/usr/bin/env python

import os
import time
from copy import copy
import json
from collections import OrderedDict
import xmltodict
import numpy as np
from scipy.interpolate import UnivariateSpline

from pinocchio.utils import *
from gepetto.corbaserver import Client

from wdc_dynamicsteller import DynamicsTeller, rootjoint
from wdc.effort_assessment.state_extracting import State, retrieve_freeflyer
from wdc.effort_assessment.compute_efforts import compute_efforts_with_rnea
from wdc.log_analysis_py import WDC_MESH_PATH, WDC_URDF_PATH


EXO_THIGH_RPY = [-7/180*np.pi, 0, 0]
EXO_DEFAULT_THIGH_SETTINGS_MM = 420
JOINT_ORDER_EXO_SIMU = ['LeftFrontalHipJoint',
                        'LeftTransverseHipJoint',
                        'LeftSagittalHipJoint',
                        'LeftSagittalKneeJoint',
                        'LeftSagittalAnkleJoint',
                        'LeftHenkeAnkleJoint',
                        'RightFrontalHipJoint',
                        'RightTransverseHipJoint',
                        'RightSagittalHipJoint',
                        'RightSagittalKneeJoint',
                        'RightSagittalAnkleJoint',
                        'RightHenkeAnkleJoint']
JOINT_ORDER_NN = ["RightFrontalHipJoint",
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
SUPPORT_FOOT_NN = 'RightSole'
SUPPORT_FOOT_ENUM = ['RightSole', 'LeftSole']
INPUT_ORDER_NN = {"steplength": 1, "duration": 2, "stairheight": 3}
THOT_TO_SIMU_JOINT_MASK_POSITION = np.concatenate((np.arange(7,13),np.arange(14,20)))
THOT_TO_SIMU_JOINT_MASK_VELOCITY = np.concatenate((np.arange(6,12),np.arange(13,19)))
RELABELING_JOINT_MATRIX = np.zeros((12,12))
RELABELING_JOINT_MATRIX[6:,:6] = RELABELING_JOINT_MATRIX[:6,6:] = np.diag([-1,-1,1,1,1,-1])

## @brief Reorder the elements position and velocity from json to dynamics_teller order
##
## @param position list of joint position
## @param velocity list of joint velocity
## @param json_order list giving the joint order in json file
## @param dynamics_teller DynamicsTeller target
def _reorderJoint(dynamics_teller, joint_order, position, velocity=None, acceleration=None):
    q = np.zeros((dynamics_teller.getModelPositionSize(), position.shape[1]))
    if velocity is not None:
        v = np.zeros((dynamics_teller.getModelVelocitySize(), position.shape[1]))
        if acceleration is not None:
            a = np.zeros((dynamics_teller.getModelVelocitySize(), position.shape[1]))
    for i in range(len(joint_order)):
        q[dynamics_teller.getJointIdInPosition(joint_order[i])] = position[i]
        if velocity is not None:
            v[dynamics_teller.getJointIdInVelocity(joint_order[i])] = velocity[i]
            if acceleration is not None:
                a[dynamics_teller.getJointIdInVelocity(joint_order[i])] = acceleration[i]
    if velocity is None:
        return q
    if acceleration is None:       
        return q, v
    return q, v, a

def smoothing_filter(time_in,val_in,time_out=None,relabel=None,params=None):
    if time_out is None:
        time_out = time_in
    if params is None:
        params = dict()
        params['mixing_ratio_1'] = 0.12
        params['mixing_ratio_2'] = 0.04
        params['smoothness'] = [0.0,0.0,0.0]
        params['smoothness'][0]  = 5e-4
        params['smoothness'][1]  = 5e-4
        params['smoothness'][2]  = 5e-4

    if relabel is None:
        mix_fit    = [None,None,None]
        mix_fit[0] = lambda t: 0.5*(1+np.sin(1/params['mixing_ratio_1']*((t-time_in[0])/(time_in[-1]-time_in[0]))*np.pi-np.pi/2))
        mix_fit[1] = lambda t: 0.5*(1+np.sin(1/params['mixing_ratio_2']*((t-(1-params['mixing_ratio_2'])*time_in[-1])/(time_in[-1]-time_in[0]))*np.pi+np.pi/2))
        mix_fit[2] = lambda t: 1
        
        val_fit = []
        for jj in range(val_in.shape[0]):
            val_fit_jj = []
            for kk in range(len(params['smoothness'])):
                val_fit_jj.append(UnivariateSpline(time_in, val_in[jj], s=params['smoothness'][kk]))
            val_fit.append(val_fit_jj)
        
        time_out_mixing = [None, None, None]
        time_out_mixing_ind = [None, None, None]
        time_out_mixing_ind[0] = time_out < time_out[-1]*params['mixing_ratio_1']
        time_out_mixing[0] = time_out[time_out_mixing_ind[0]]
        time_out_mixing_ind[1] = time_out > time_out[-1]*(1-params['mixing_ratio_2'])
        time_out_mixing[1] = time_out[time_out_mixing_ind[1]]
        time_out_mixing_ind[2] = np.logical_and(np.logical_not(time_out_mixing_ind[0]), np.logical_not(time_out_mixing_ind[1]))
        time_out_mixing[2] = time_out[time_out_mixing_ind[2]]
        
        val_out = np.zeros((val_in.shape[0],len(time_out)))
        for jj in range(val_in.shape[0]):
            for kk in range(len(time_out_mixing)):
                val_out[jj,time_out_mixing_ind[kk]] = \
                   (1 - mix_fit[kk](time_out_mixing[kk])) * val_fit[jj][kk](time_out_mixing[kk]) + \
                        mix_fit[kk](time_out_mixing[kk])  * val_fit[jj][-1](time_out_mixing[kk])
    else:
        time_tmp   = np.concatenate([time_in[:-1]-time_in[-1],time_in,time_in[1:]+time_in[-1]])
        val_in_tmp = np.concatenate([relabel.dot(val_in[:,:-1]),val_in,relabel.dot(val_in[:,1:])], axis=1)
        val_out = np.zeros((val_in.shape[0],len(time_out)))
        for jj in range(val_in_tmp.shape[0]):
            f = UnivariateSpline(time_tmp, val_in_tmp[jj], s=params['smoothness'][-1])
            val_out[jj] = f(time_out)

    return val_out

def get_patient_info(urdf_path):
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

    return patient_info

def extract_state_from_neural_network_prediction(urdf_path, pred):
    # Extract time and joint angle evolution from prediction
    t = np.linspace(0,pred[1],200)
    q = np.reshape(pred[42:], (12,-1))

    # Post-processing: Filtering and numerical derivation
    params = dict()
    params['mixing_ratio_1'] = 0.04
    params['mixing_ratio_2'] = 0.04
    params['smoothness'] = [0.0,0.0,0.0]
    params['smoothness'][0]  = 2e-9
    params['smoothness'][1]  = 2e-9
    params['smoothness'][2]  = 2e-9

    q = smoothing_filter(t, q, t, relabel=RELABELING_JOINT_MATRIX, params=params)
    t_tmp  = (t[:-1] + t[1:])/2
    dq  = np.diff(q,n=1,axis=1) / np.diff(t)
    dq = smoothing_filter(t_tmp, dq, t)
    t_tmp = (t[:-1] + t[1:])/2
    ddq = np.diff(dq,n=1,axis=1) / np.diff(t)
    ddq = smoothing_filter(t_tmp, ddq, t)

    ## Reorder the joints
    rbdt = DynamicsTeller.make(urdf_path, rootjoint.FREEFLYER)
    qe, dqe, ddqe = _reorderJoint(rbdt, JOINT_ORDER_NN, q, dq, ddq)
    
    # Create state sequence
    evolution_robot = []
    for i in range(len(t)):
        evolution_robot.append(State(qe[:,[i]], dqe[:,[i]], ddqe[:,[i]], t[i], support_foot=SUPPORT_FOOT_NN))

    trajectory_data = {"evolution_robot": evolution_robot, "urdf": urdf_path}

    # Estimate the free-flyer
    retrieve_freeflyer(trajectory_data)

    # Compute the ground reaction forces and motor torques
    compute_efforts_with_rnea(trajectory_data, verbose=False)

    return trajectory_data

def extract_state_from_simulation_log(urdf_path, log_data):
    # Extract time, joint positions and velocities evolution from log.
    # Note that the quaternion angular velocity vectors are expressed
    # it body frame rather than world frame.
    t = log_data[:,0]
    qe = log_data[:,1:22].T
    dqe = log_data[:,23:42].T

    ## Reorder the joints
    # rbdt = DynamicsTeller.make(urdf_path, rootjoint.FREEFLYER)
    # qe, dqe = _reorderJoint(rbdt, JOINT_ORDER_EXO_SIMU, q, dq)

    # Post-processing: numerical derivation
    ddqe = np.gradient(dqe, t, axis=1)

    # Create state sequence
    evolution_robot = []
    for i in range(len(t)):
        evolution_robot.append(State(qe[:,[i]], dqe[:,[i]], ddqe[:,[i]], t[i], support_foot=SUPPORT_FOOT_NN))

    return {"evolution_robot": evolution_robot, "urdf": urdf_path}

def load_csv_log(csv_log_path):
    return np.genfromtxt(csv_log_path, delimiter=',')

def get_initial_state_simulation(trajectory_data):
    evolution_robot = trajectory_data["evolution_robot"]
    qe = evolution_robot[0].q
    dqe = evolution_robot[0].v

    x0 = np.concatenate((qe[0:3],np.array([[1]]),qe[3:6]/qe[6],qe[THOT_TO_SIMU_JOINT_MASK_POSITION],
                         dqe[0:6],dqe[THOT_TO_SIMU_JOINT_MASK_VELOCITY]))
    x0[2] -= 0.02 # Compensate optoforce wrong frame

    return x0

def get_n_steps(trajectory_data, n):
    evolution_robot = trajectory_data["evolution_robot"]
    
    state_dict = State.todict(evolution_robot)

    state_dict_sym = dict()
    support_foot = [elem for elem in SUPPORT_FOOT_ENUM if state_dict['support_foot'][0] not in elem][0]
    state_dict_sym['q'] = copy(state_dict['q'])
    state_dict_sym['q'][THOT_TO_SIMU_JOINT_MASK_POSITION] = \
        RELABELING_JOINT_MATRIX.dot(state_dict_sym['q'][THOT_TO_SIMU_JOINT_MASK_POSITION])
    state_dict_sym['v'] = copy(state_dict['v'])
    state_dict_sym['v'][THOT_TO_SIMU_JOINT_MASK_VELOCITY] = \
        RELABELING_JOINT_MATRIX.dot(state_dict_sym['v'][THOT_TO_SIMU_JOINT_MASK_VELOCITY])
    state_dict_sym['a'] = copy(state_dict['a'])
    state_dict_sym['a'][THOT_TO_SIMU_JOINT_MASK_VELOCITY] = \
        RELABELING_JOINT_MATRIX.dot(state_dict_sym['a'][THOT_TO_SIMU_JOINT_MASK_VELOCITY])
    state_dict_sym['t'] = copy(state_dict['t'])
    state_dict_sym['hzd_state'] = [None for i in range(len(state_dict['hzd_state']))]
    state_dict_sym['support_foot'] = [support_foot for i in range(len(state_dict['support_foot']))]
    state_dict_sym['f'] = [None for i in range(len(state_dict['f']))]
    state_dict_sym['tau'] = copy(state_dict['tau'])
    for key in state_dict['tau'][0].keys():
        for i in range(state_dict['q'].shape[1]):
            state_dict_sym['tau'][i][key][THOT_TO_SIMU_JOINT_MASK_VELOCITY] = \
                RELABELING_JOINT_MATRIX.dot(state_dict_sym['tau'][i][key][THOT_TO_SIMU_JOINT_MASK_VELOCITY])
    state_dict_sym['f_ext'] = [None for i in range(len(state_dict['f_ext']))]

    state_dict_gbl = dict(state_dict) # Copy
    for i in range(1,n):
        if i % 2:
            state_dict_tmp = dict(state_dict_sym) # Copy
        else:
            state_dict_tmp = dict(state_dict) # Copy
        state_dict_tmp['t'] += state_dict_gbl['t'][-1] - state_dict_tmp['t'][0]
        state_dict_tmp['q'][0] += state_dict_gbl['q'][0,-1] - state_dict_tmp['q'][0,0]

        for key in state_dict_gbl.keys():
            if isinstance(state_dict_tmp[key], list):
                state_dict_gbl[key] = state_dict_gbl[key] + state_dict_tmp[key] # list concatenation
            else:
                state_dict_gbl[key] = np.concatenate((state_dict_gbl[key], state_dict_tmp[key]), axis=-1) # list concatenation

    return {"evolution_robot": State.fromdict(state_dict_gbl), "urdf": trajectory_data["urdf"]}

def delete_scenes_viewer(*scene_names):
    client = Client()
    for scene_name in scene_names:
        if scene_name in client.gui.getNodeList():
            client.gui.deleteNode(scene_name, True)