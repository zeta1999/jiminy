from collections import OrderedDict

import numpy as np
import numba as nb

from exo_simu_py import *


spec = [('t', nb.float64[:]),
        ('q', nb.float64[:,:]),
        ('dq', nb.float64[:,:]),
        ('ddq', nb.float64[:,:]),
        ('u', nb.float64[:,:])]
@nb.jitclass(spec)
class _traj_ref_type(object):
    def __init__(self, t, q, dq, ddq, u):
        self.t = t
        self.q = q
        self.dq = dq
        self.ddq = ddq
        self.u = u

# Impossible to use from bisect import bisect_left because of numba jit ...
@nb.jit(nopython=True, nogil=True)
def bisect_left(arr, target):
    n = len(arr)
    left = 0
    right = n - 1
    mid = 0

    if target >= arr[n - 1]:
        return n - 1
    if target <= arr[0]:
        return 0

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

@nb.jit(nopython=True, nogil=True)
def _linear_interp(ratio, value_min, value_max):
    return np.atleast_2d((1 - ratio) * value_min + ratio * value_max).T

@nb.jit(nopython=True, nogil=True)
def _compute_command(Kp, Kd, traj_ref, t_rel, qe, v, forceSensorsData, imuSensorsData, encoderSensorsData):
    # Determine the relative time in the current state
    t_ind_inf = bisect_left(traj_ref.t, t_rel)
    t_ind_sup = t_ind_inf + 1
    ratio = (t_rel - traj_ref.t[t_ind_inf])/(traj_ref.t[t_ind_sup] - traj_ref.t[t_ind_inf])

    # Compute the value of the reference trajectory at that time
    q_ref = _linear_interp(ratio, traj_ref.q[:,t_ind_inf], traj_ref.q[:,t_ind_sup])
    dq_ref = _linear_interp(ratio, traj_ref.dq[:,t_ind_inf], traj_ref.dq[:,t_ind_sup])
    #ddq_ref = self._linear_interp(ratio, traj_ref['ddq'][:,t_ind_inf], traj_ref['ddq'][:,t_ind_sup])
    u_ref = _linear_interp(ratio, traj_ref.u[:,t_ind_inf], traj_ref.u[:,t_ind_sup])

    # Compute PID torques
    q = np.expand_dims(encoderSensorsData[0].transpose(), axis=1)
    dq = np.expand_dims(encoderSensorsData[1].transpose(), axis=1)
    u_pid = - (Kp * (q - q_ref) + Kd * (dq - dq_ref))

    return u_ref + u_pid
        
class pid_feedforward:
    def __init__(self, trajectory_data_ref, Kp=None, Kd=None):
        if Kp is None:
            Kp = np.array([[20000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0,
                            20000.0, 10000.0, 10000.0, 10000.0, 10000.0, 10000.0]]).T
        self.Kp = Kp
        if Kd is None:
            Kd = np.array([[250.0, 150.0, 100.0, 100.0, 150.0, 100.0, 
                            250.0, 150.0, 100.0, 100.0, 150.0, 100.0]]).T
        self.Kd = Kd
        
        self.state_machine = OrderedDict()
        evolution_robot = trajectory_data_ref['evolution_robot']
        support_foot_ref = evolution_robot[0].support_foot
        state_ref = self.state_machine[support_foot_ref] = dict()
        state_ref['t'] = np.array([s.t for s in evolution_robot])
        state_ref['q'] = np.asarray(np.concatenate(
            [s.q[JOINT_MASK_POSITION] for s in evolution_robot], axis=1))
        state_ref['dq'] = np.asarray(np.concatenate(
            [s.v[JOINT_MASK_VELOCITY] for s in evolution_robot], axis=1))
        state_ref['ddq'] = np.asarray(np.concatenate(
            [s.a[JOINT_MASK_VELOCITY] for s in evolution_robot], axis=1))
        state_ref['u'] = np.asarray( np.concatenate(
            [s.tau.items()[0][1][JOINT_MASK_VELOCITY] for s in evolution_robot], axis=1))
        support_foot_next = [elem for elem in SUPPORT_FOOT_ENUM if support_foot_ref not in elem][0]
        state_next = self.state_machine[support_foot_next] = dict()
        state_next['t'] = state_ref['t']
        state_next['q'] = RELABELING_JOINT_MATRIX.dot(state_ref['q'])
        state_next['dq'] = RELABELING_JOINT_MATRIX.dot(state_ref['dq'])
        state_next['ddq'] = RELABELING_JOINT_MATRIX.dot(state_ref['ddq'])
        state_next['u'] = RELABELING_JOINT_MATRIX.dot(state_ref['u'])

        self.time_offset = 0
        self.current_state = support_foot_ref
        self.traj_ref = self.get_traj_ref()

    def reset(self):
        self.time_offset = 0
        self.current_state = self.state_machine.keys()[0]
        self.traj_ref = self.get_traj_ref()

    def get_traj_ref(self, state=None):
        if state is None:
            state = self.current_state
        return _traj_ref_type(self.state_machine[state]['t'],
                              self.state_machine[state]['q'],
                              self.state_machine[state]['dq'],
                              self.state_machine[state]['ddq'],
                              self.state_machine[state]['u'])

    def state_machine_switch_next(self):
        self.time_offset += self.traj_ref.t[-1]
        self.current_state = [elem for elem in SUPPORT_FOOT_ENUM 
                              if self.current_state not in elem][0]
        self.traj_ref = self.get_traj_ref()
        # print("switching to next state: %s" % self.current_state)
        
    def state_machine_switch_prev(self):
        self.current_state = [elem for elem in SUPPORT_FOOT_ENUM 
                              if self.current_state not in elem][0]
        self.traj_ref = self.get_traj_ref()
        self.time_offset -= self.traj_ref.t[-1]
        # print("switching to previous state: %s" % self.current_state)

    def compute_command(self, t_cur, qe, v, forceSensorsData, imuSensorsData, encoderSensorsData):
        # Change of state if necessary, and get information about the current state
        t_rel = t_cur - self.time_offset
        if t_rel > self.traj_ref.t[-1]:
            self.state_machine_switch_next()
            t_rel = t_cur - self.time_offset
        elif t_rel < 0:
            self.state_machine_switch_prev()
            t_rel = t_cur - self.time_offset
        
        return _compute_command(self.Kp, self.Kd, self.traj_ref, t_rel, qe, v, forceSensorsData, imuSensorsData, encoderSensorsData)