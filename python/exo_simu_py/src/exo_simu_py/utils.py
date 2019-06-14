#!/usr/bin/env python

import json
import argparse
import os
import time
import numpy as np
from numba import jit # Use to precompile Python code
from scipy.interpolate import UnivariateSpline

from gepetto.corbaserver import Client
import pinocchio as se3
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
from wdc_dynamicsteller import *
from wdc.log_analysis_py import WDC_MESH_PATH, WDC_URDF_PATH


joint_order = ["LeftFrontalHipJoint",
               "LeftTransverseHipJoint",
               "LeftSagittalHipJoint",
               "LeftSagittalKneeJoint",
               "LeftSagittalAnkleJoint",
               "LeftHenkeAnkleJoint",
               "RightFrontalHipJoint",
               "RightTransverseHipJoint",
               "RightSagittalHipJoint",
               "RightSagittalKneeJoint",
               "RightSagittalAnkleJoint",
               "RightHenkeAnkleJoint"]


## @brief Reorder the elements position and velocity from json to dynamics_teller order
##
## @param position list of joint position
## @param velocity list of joint velocity
## @param json_order list giving the joint order in json file
## @param dynamics_teller DynamicsTeller target
def _reorderJoint(position, velocity, joint_order, dynamics_teller):
    q = np.zeros((dynamics_teller.getModelPositionSize(), 1))
    v = np.zeros((dynamics_teller.getModelVelocitySize(), 1))
    for i in range(len(joint_order)):
        q[dynamics_teller.getJointIdInPosition(joint_order[i])] = position[i]
        v[dynamics_teller.getJointIdInVelocity(joint_order[i])] = velocity[i]
    return q, v

def load_csv_log(csv_log_path):
    return np.genfromtxt(csv_log_path, delimiter=',')

## @brief visualize Replays a exo_simu-readable csv log file into gepetto viewer.
##
## @details This enables quick visual review of the trajectory (though it does not guarantee that the trajectory will work
## on the exoskeleton). Indeed, hardware stops (present in the URDF) are not checked, and soft
## bounds cannot be checked (as they depend on wanderbrain and this script does not parse it).
## This script 'tries' to replay a trajectory in real time. In practice however there will be small time dilatation
## (typically trajectory running 5 to 10% slower than usual).
##
## @param log_data Data of a csv log file stored in a numpy.array
## @param speed_ratio Ratio between real time and display time
def visualize(log_data, speed_ratio=1.0):
    # Load urdf file, create DynamicsTeller and RobotWrapper
    robot = RobotWrapper()
    robot.initFromURDF(WDC_URDF_PATH, WDC_MESH_PATH, se3.JointModelFreeFlyer())
    dynamics_teller = DynamicsTeller.make(WDC_URDF_PATH, rootjoint.FREEFLYER)

    lower_limit, upper_limit = dynamics_teller.getPositionLimits()

    # Load robot in gepetto viewer
    client = Client()
    try:
        scene = client.gui.createSceneWithFloor('world')
        if not 'replay_simulation' in client.gui.getWindowList():
            window = client.gui.createWindow('replay_simulation')
            client.gui.addSceneToWindow('world', window)
            client.gui.createGroup('world/world')
            client.gui.addLandmark('world/world', 0.1)
        else:
            window = client.gui.getWindowID('replay_simulation')
    except:
        pass
    robot.initDisplay()
    robot.loadDisplayModel("world/wdc_robot")

    # Load trajectory file
    t = log_data[:,0]
    q = log_data[:,8:20]
    positions_freeflyer = log_data[:,1:4]
    quatVec_freeflyer = log_data[:,np.concatenate([np.arange(5,8), [4]])]
    quatVec_freeflyer /= np.linalg.norm(quatVec_freeflyer, axis=1, keepdims=True)

    # Init timer in order to run in "real time"
    init_time = time.time()
    for i in range(len(t)):
        qe, v = _reorderJoint(q[i], np.zeros(12,), joint_order, dynamics_teller)
        qe[0:3] = np.asmatrix(positions_freeflyer[i]).T
        qe[3:7] = np.asmatrix(quatVec_freeflyer[i]).T

        robot.display(qe)
        client.gui.refresh()
        t_simu = (time.time() - init_time) * speed_ratio
        if t_simu < t[i]:
            time.sleep(t[i] - t_simu)

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

@jit(nopython=True)
def get_closest_left_ind(arr, target):
    n = len(arr)
    left = 0
    right = n - 1
    mid = 0

    # edge case - last or above all
    if target >= arr[n - 1]:
        return n - 1
    # edge case - first or below all
    if target <= arr[0]:
        return 0
    # BSearch solution: Time & Space: Log(N)

    while left < right:
        mid = (left + right) // 2  # find the mid
        if target < arr[mid]:
            right = mid
        elif target > arr[mid]:
            left = mid + 1
        else:
            return mid

    if target < arr[mid]:
        return mid - 1
    else:
        return mid