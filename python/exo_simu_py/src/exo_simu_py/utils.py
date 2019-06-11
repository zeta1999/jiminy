#!/usr/bin/env python

import json
import argparse
import numpy as np
import os
import time
from numpy import genfromtxt

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
    return genfromtxt(csv_log_path, delimiter=',')

## @brief visualize Replays a exo_simu-readable csv log file into gepetto viewer.
##
## @details This enables quick visual review of the trajectory (though it does not guarantee that the trajectory will work
## on the exoskeleton). Indeed, hardware stops (present in the URDF) are not checked, and soft
## bounds cannot be checked (as they depend on wanderbrain and this script does not parse it).
## This script 'tries' to replay a trajectory in real time. In practice however there will be small time dilatation
## (typically trajectory running 5 to 10% slower than usual).
##
## @param log_data Data of a csv log file stored in a numpy.array
## @param fast_mode Do not try to keep the real time factor and display as fast as possible instead
def visualize(log_data, fast_mode=False):
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
        if not fast_mode:
            t_simu = time.time() - init_time
            if t_simu < t[i]:
                time.sleep(t[i] - t_simu)
