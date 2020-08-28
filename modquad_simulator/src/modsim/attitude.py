import math

from modsim.util.state import state_to_quadrotor
from modsim.util.linearalg import vee_map
from modsim.util.linearalg import hat_map
from transforms3d import quaternions
import numpy as np

# Old error
accumulated_error = np.array([0., 0., 0.])


def geo_attitude_controller(f_des, state_vector, desired_state):
    """
    Attitude controller for the entire structure as one.
    the output are force and moment vectors.
    :param f_des: desired force vector
    :param state_vector: current state of the structure, [pos(0:3), vel(3:6), qx, qy, qz, qw, omega(10:13)]
    :param desired_state: [[pos_des], [vel_des], [acc_des], [yaw_des], [yawdot_des]]
    :return tau: desired moment vector
    """
    global accumulated_error

    # access desired state
    [pos_des, vel_des, acc_des, yaw_des, des_yawdot] = desired_state

    # access current state
    # pos = state_vector[:3]
    # vel = state_vector[3:6]
    quat = np.array([state_vector[9], state_vector[6], state_vector[7], state_vector[8]])  # Somebody decided to reverse
                                                                                           # the qw in state.py
    R = quaternions.quat2mat(quat)
    omega = np.array(state_vector[10:])

    kp, kd, ki = 1.43e-5 * 1200, 1.43e-5 * 540, .00001

    # desired direction in SO(3)
    # Eq 6a - 6e
    Zb = f_des*1.0/np.linalg.norm(f_des)
    Xc = np.array([np.cos(yaw_des), np.sin(yaw_des), 0])
    Yb_not_unit = np.cross(Zb, Xc)
    Yb = Yb_not_unit*1.0/np.linalg.norm(Yb_not_unit)
    Xb = np.cross(Yb, Zb)
    Rd = np.vstack((Xb, Yb, Zb)).T

    # error in SO(3)
    # Eq 7a - 7b
    e_R = -0.5*np.array(vee_map(Rd.T.dot(R)-R.T.dot(Rd)))
    e_omega = omega - R.T.dot(Rd).dot(np.array([0, 0, des_yawdot]))

    accumulated_error += e_R
    # print accumulated_error[0]

    # Eq 8b
    tau = -kp*e_R - kd*e_omega - ki*accumulated_error
    return tau


def attitude_controller((F_newtons, roll_des, pitch_des, yaw_des), x):
    """
    Attitude controller for crazyflie, receiving pwm as input.
    the output are forces and moments.
    :type F_newtons: force in newtons.
    :param x:
    :return:
    """
    global accumulated_error

    ### Moments
    # Quaternion to angles
    quad_state = state_to_quadrotor(x)

    kpx, kdx, kix = 1.43e-5 * 250, 1.43e-5 * 60, .0002

    e = [math.radians(roll_des) - quad_state.euler[0],
         math.radians(pitch_des) - quad_state.euler[1],
         math.radians(yaw_des) - quad_state.euler[2]]

    accumulated_error += e
    # print accumulated_error[0]

    Mx = kpx * e[0] + kdx * (0 - quad_state.omega[0]) + kix * accumulated_error[0]
    My = kpx * e[1] + kdx * (0 - quad_state.omega[1]) + kix * accumulated_error[1]
    return F_newtons, [Mx, My, 0]