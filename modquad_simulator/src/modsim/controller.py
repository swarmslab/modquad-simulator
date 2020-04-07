import params
from math import sin, cos
import numpy as np
from math import sqrt
from scipy.spatial.transform import Rotation as R
from modsim.util.state import init_state, state_to_quadrotor
from transforms3d import euler as trans
from transforms3d import quaternions as trans2

accumulated_error = np.array([0., 0., 0.])

def vec_dot(a, b):
    c = np.dot(np.transpose(a), b)
    return c


def vee_map(A):
    rm = np.array([[A[2, 1]], [A[0, 2]], [A[1, 0]]])
    return rm


def vec_cross(a, b):
    c = np.array([[a[1, 0] * b[2, 0] - a[2, 0] * b[1, 0]],
                 [-a[0, 0] * b[2, 0] + a[2, 0] * b[0, 0]],
                 [a[0, 0] * b[1, 0] - a[1, 0] * b[0, 0]]])
    c.shape = (3, 1)
    return c


def hat_map(aux):
    A = np.array([[0, -aux[2, 0], aux[1, 0]], [aux[2, 0], 0, -aux[0, 0]], [-aux[1, 0], aux[0, 0], 0]])
    return A


def RotToRPY_ZXY(R):
#%written by Diego Salazar

    R = np.transpose(R)
    phi = np.arcsin(R[1, 2])
    psi = np.arctan2(-R[1, 0]/cos(phi), R[1, 1]/cos(phi))
    theta = np.arctan2(-R[0, 2]/cos(phi), R[2, 2]/cos(phi))

    return phi, theta, psi


def RPYtoRot_ZXY(phi , theta, psi):
#% written by Diego Salazar

    Rx = np.array([[1, 0, 0],
                   [0, np.cos(phi), np.sin(phi)],
                   [0, -np.sin(phi), np.cos(phi)]])

    Ry = np.array([[np.cos(theta), 0, -np.sin(theta)],
                   [0 , 1, 0],
                   [np.sin(theta), 0, np.cos(theta)]])

    Rz = np.array([[np.cos(psi), np.sin(psi), 0],
                   [np.sin(psi), np.cos(psi), 0],
                   [0, 0, 1]])

    Ra = np.transpose(Rz.dot(Ry).dot(Rx))
    return R

def position_controller(state_vector, desired_state):
    """
    PD controller to convert from position to accelerations, and accelerations to attitude.
    Controller: Using these current and desired states, you have to compute the desired controls

    :param qd: The object qt contains the current state and the desired state:
                * The current states are: qd.pos, qd.vel, qd.euler = [rollpitchyaw], qd.omega.
                * The desired states are: qd.pos_des, qd.vel_des, qd.acc_des, qd.yaw_des, qd.yawdot_des
    :return: desired thrust and attitude
    """
    global accumulated_error

    # Desired state
    [pos_des, vel_des, acc_des, yaw_des, des_yawdot] = desired_state

    # current state
    pos = state_vector[:3]
    vel = state_vector[3:6]

    # constants
    m = params.mass
    g = params.grav

    kp1_u, kd1_u, ki1_u = 10., 71., .0
    kp2_u, kd2_u, ki2_u = 10., 71., .0
    kp3_u, kd3_u, ki3_u = 10., 48., .0

    # Error
    pos_error = pos_des - pos
    vel_error = vel_des - vel
    accumulated_error += pos_error
    # print pos_error

    # Desired acceleration
    r1_acc = kp1_u * pos_error[0] + kd1_u * vel_error[0] + acc_des[0] + ki1_u * accumulated_error[0]
    r2_acc = kp2_u * pos_error[1] + kd2_u * vel_error[1] + acc_des[1] + ki2_u * accumulated_error[1]
    r3_acc = kp3_u * pos_error[2] + kd3_u * vel_error[2] + acc_des[2] + ki3_u * accumulated_error[2]

    phi_des = (r1_acc * sin(yaw_des) - r2_acc * cos(yaw_des)) / g
    theta_des = (r1_acc * cos(yaw_des) + r2_acc * sin(yaw_des)) / g
    psi_des = yaw_des

    # Thrust
    thrust = m * g + m * r3_acc

    # desired thrust and attitude
    return [thrust, phi_des, theta_des, psi_des]


def controller_GeometricSO3(state_vector, desired_state):
    """
    PD controller to convert from position to accelerations, and accelerations to attitude.
    Controller: Using these current and desired states, you have to compute the desired controls

    :param qd: The object qt contains the current state and the desired state:
                * The current states are: qd.pos, qd.vel, qd.euler = [rollpitchyaw], qd.omega.
                * The desired states are: qd.pos_des, qd.vel_des, qd.acc_des, qd.yaw_des, qd.yawdot_des
    :return: desired thrust and attitude
    """
    global accumulated_error

    # Desired state
    [x_des, v_des, a_des, b1d] = desired_state

    # current state
    pos = state_vector[:3]
    pos.shape = (3, 1)

    vel = state_vector[3:6]
    vel.shape = (3, 1)

    # constants
    #m = 4.34
    g = params.grav

    mass = 4.34
    J = params.J
    g = params.g
    e1 = params.e1
    e1.shape = (3, 1)
    e2 = params.e2
    e3 = params.e3
    e3.shape = (3, 1)

    w_desired = params.w_desired
    w_desired_dot = np.zeros(3)
    w_desired_dot.shape = (3, 1)

    x_curr, v_curr, qt, w = state_to_quadrotor(state_vector)
    x_curr.shape = (3, 1)
    v_curr.shape = (3, 1)


    Ra = np.reshape(trans.quat2mat(qt), (3, 3))

    w.shape = (3, 1)

    b3 = Ra[:, 2]
    b3.shape = (3, 1)

    dx = []

    #   %%%%%%%%%%%
    #   % CONTROL %
    #   %%%%%%%%%%%
    #   Position Control
    err_x = x_curr - x_des
    err_x.shape = (3, 1)
    err_v = v_curr - v_des
    err_v.shape = (3, 1)
    #   constants for force and moment
    k1 = params.kx
    k2 = params.kv

    A = -np.dot(k1, err_x) - np.dot(k2, err_v) - mass * g * e3 + mass * a_des

    normA = np.linalg.norm(A)

    b3_desired = np.true_divide(A, normA)

    f = -vec_dot(A, Ra.dot(e3))

    #   %%%%%%%%%%%%%%%%%%%%
    #   % Attitude Control %
    #   %%%%%%%%%%%%%%%%%%%%

    b2n = vec_cross(b3_desired, b1d)
    norm_b2n = np.linalg.norm(b2n)
    b2d = np.true_divide(b2n, norm_b2n)
    projection_b1d = -vec_cross(b3_desired, b2d)
    Rd = np.hstack((vec_cross(b2d, b3_desired), b2d, b3_desired))

    #   angluar velocity and rotation matrix constants
    kR = params.kR
    kOm = params.komega

    #   calculating error in angular velocity and attitude
    err_R = 0.5 * vee_map(np.dot(np.transpose(Rd), Ra) - np.dot(np.transpose(Ra), Rd))
    err_Om = w - np.transpose(Ra).dot(Rd).dot(w_desired)

    M = -kR * err_R - kOm * err_Om + vec_cross(w, np.dot(J, w)) \
        - J.dot((hat_map(w)).dot(np.transpose(Ra)).dot(Rd).dot(w_desired) \
                - np.transpose(Ra).dot(Rd).dot(w_desired_dot))

    xQ_dot = v_curr
    vQ_dot = g * e3 - (f / mass) * Ra.dot(e3)

    R_dot = np.dot(Ra, hat_map(w))
    Omega_dot = np.linalg.solve(J, -(vec_cross(w, np.dot(J, w))) + M)

    #   % Computing xd

    xd = np.concatenate((x_des, v_des, np.reshape(Rd, (9, 1)), w_desired), axis=0)

    #    % Computing dx
    #    %-------------

    quat_dot = trans2.mat2quat(R_dot)
    quat_dot.shape = (4, 1)

    dx = np.concatenate((xQ_dot, vQ_dot, quat_dot, Omega_dot))

    # desired thrust and attitude

    return f, M, dx

def modquad_torque_control(F, M, structure, motor_sat=False):
    """
    This function is similar to crazyflie_motion, but it is made for modular robots. So it specifies the dynamics
    of the modular structure. It receives a desired force and moment of a single robot.
    :param structure:
    :param motor_sat: motor saturation
    :param F: desired total thrust, float
    :param M: desired moments, 3 x 1 float vector
    :return: thrust and moments for the whole structure
    """
    ## From moments to rotor forces (power distribution)
    # positions of the rotors
    #         ^ X
    #    (4)  |      (1) [L, -L]
    #   Y<-----
    #    (3)         (2)
    rx, ry = [], []
    L = params.arm_length * sqrt(2) / 2.

    for x, y in zip(structure.xx, structure.yy):
        rx.append(x + L)
        rx.append(x - L)
        rx.append(x - L)
        rx.append(x + L)
        # y-axis
        ry.append(y - L)
        ry.append(y - L)
        ry.append(y + L)
        ry.append(y + L)

    sign_rx = [1 if rx_i > 0 else -1 for rx_i in rx]
    sign_ry = [1 if ry_i > 0 else -1 for ry_i in ry]
    # m = 4 * n  # Number of rotors
    A = [[0.25, sy * .25 / L, -sx * .25 / L] for sx, sy in zip(sign_rx, sign_ry)]

    rotor_forces = np.dot(A, [F, M[0], M[1]])  # Not using moment about Z-axis for limits

    # Failing motors
    for mf in structure.motor_failure:
        rotor_forces[4 * mf[0] + mf[1]] *= 0.0

    # Motor saturation
    if motor_sat:
        rotor_forces[rotor_forces > params.maxF / 4] = params.maxF / 4
        rotor_forces[rotor_forces < params.minF / 4] = params.minF / 4

    # From prop forces to total moments. Equation (1) of the modquad paper (ICRA 18)
    F = np.sum(rotor_forces)
    Mx = np.dot(ry, rotor_forces)
    My = -np.dot(rx, rotor_forces)
    # TODO Mz
    Mz = M[2]

    return F, [Mx, My, Mz], rotor_forces

