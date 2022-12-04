from helpers import logistic, to_equation, differentiate, nearest_approach_to_any_vehicle, get_f_and_N_derivatives
from constants import *
import numpy as np

# COST FUNCTIONS

# NOTE: only check final configuration
def time_diff_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes trajectories that span a duration which is longer or 
    shorter than the duration requested.
    """
    _, _, t = traj
    return logistic(float(abs(t-T)) / T)

# NOTE: only check final configuration
def s_diff_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes trajectories whose s coordinate (and derivatives) 
    differ from the goal.
    """

    # NOTE: s means 6 s_coefficients
    s, _, T = traj

    target = predictions[target_vehicle].state_in(T)
    target = list(np.array(target) + np.array(delta))
    s_targ = target[:3]

    # NOTE：get [s, s_dot, s_dot_dot]
    S = [f(T) for f in get_f_and_N_derivatives(s, 2)]

    cost = 0
    for actual, expected, sigma in zip(S, s_targ, SIGMA_S):
        diff = float(abs(actual-expected))
        cost += logistic(diff/sigma)
    return cost

# NOTE: only check final configuration
def d_diff_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes trajectories whose d coordinate (and derivatives) 
    differ from the goal.
    """
    _, d_coeffs, T = traj
    
    d_dot_coeffs = differentiate(d_coeffs)
    d_ddot_coeffs = differentiate(d_dot_coeffs)

    d = to_equation(d_coeffs)
    d_dot = to_equation(d_dot_coeffs)
    d_ddot = to_equation(d_ddot_coeffs)

    D = [d(T), d_dot(T), d_ddot(T)]
    
    target = predictions[target_vehicle].state_in(T)
    target = list(np.array(target) + np.array(delta))
    d_targ = target[3:]

    cost = 0
    for actual, expected, sigma in zip(D, d_targ, SIGMA_D):
        diff = float(abs(actual-expected))
        cost += logistic(diff/sigma)
    return cost

# NOTE: check collision every 0.05 sec, from 0 to T sec
def collision_cost(traj, target_vehicle, delta, T, predictions):
    """
    Binary cost function which penalizes collisions.
    """
    nearest = nearest_approach_to_any_vehicle(traj, predictions)
    print('collision_cost, nearest ', nearest)
    if nearest < 2*VEHICLE_RADIUS: return 1.0
    else : return 0.0

# NOTE: check collision every 0.05 sec, from 0 to T sec
def buffer_cost(traj, target_vehicle, delta, T, predictions):
    """
    Penalizes getting close to other vehicles.
    """
    nearest = nearest_approach_to_any_vehicle(traj, predictions)
    print('buffer_cost, nearest ', nearest)
    return logistic(2*VEHICLE_RADIUS / nearest)
    
def stays_on_road_cost(traj, target_vehicle, delta, T, predictions):
    pass

def exceeds_speed_limit_cost(traj, target_vehicle, delta, T, predictions):
    pass

def efficiency_cost(traj, target_vehicle, delta, T, predictions):
    """
    Rewards high average speeds.
    """
    s, _, t = traj

    # NOTE: position
    s = to_equation(s)

    avg_v = float(s(t)) / t
    targ_s, _, _, _, _, _ = predictions[target_vehicle].state_in(t)
    targ_v = float(targ_s) / t

    # NOTE: means ego speed should be less than and close to target speed,
    # QA: but if ego speed is larger than target speed, seems cost will be negative and even smaller?
    return logistic(2*float(targ_v - avg_v) / avg_v)

# NOTE: only consider s axis, did not consider d axis
def total_accel_cost(traj, target_vehicle, delta, T, predictions):
    s, d, t = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)

    total_acc = 0
    dt = float(T) / 100.0
    for i in range(100):
        # NOTE: every 0.05 sec
        t = dt * i
        acc = a(t)
        total_acc += abs(acc*dt)

    acc_per_second = total_acc / T
    
    return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC )

# NOTE: only consider s axis, did not consider d axis
def max_accel_cost(traj, target_vehicle, delta, T, predictions):
    s, d, t = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    a = to_equation(s_d_dot)
    all_accs = [a(float(T)/100 * i) for i in range(100)]
    max_acc = max(all_accs, key=abs)

    if abs(max_acc) > MAX_ACCEL: return 1
    else: return 0

# NOTE: only consider s axis, did not consider d axis
def max_jerk_cost(traj, target_vehicle, delta, T, predictions):
    s, d, t = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = differentiate(s_d_dot)
    jerk = to_equation(jerk)

    all_jerks = [jerk(float(T)/100 * i) for i in range(100)]
    max_jerk = max(all_jerks, key=abs)

    if abs(max_jerk) > MAX_JERK: return 1
    else: return 0

# NOTE: only consider s axis, did not consider d axis
def total_jerk_cost(traj, target_vehicle, delta, T, predictions):
    s, d, t = traj
    s_dot = differentiate(s)
    s_d_dot = differentiate(s_dot)
    jerk = to_equation(differentiate(s_d_dot))

    total_jerk = 0
    dt = float(T) / 100.0
    for i in range(100):
        t = dt * i
        j = jerk(t)
        total_jerk += abs(j*dt)

    jerk_per_second = total_jerk / T
    return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC )