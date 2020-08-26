import params
from math import sin, cos
import numpy as np
from math import sqrt
from transforms3d import quaternions
from modsim.util.linearalg import vee_map
from modsim.util.linearalg import hat_map


accumulated_error = np.array([0., 0., 0.])


def geo_position_controller(state_vector, desired_state):
    """
    The position part of a geometric controller
    :param state_vector: [pos(0:3), vel(3:6), qx, qy, qz, qw, omega(10:13)]
    :param desired_state: [[pos_des], [vel_des], [acc_des], [yaw_des], [yawdot_des]]
    :return: desired thrust vector
    """
    global accumulated_error
    
    # access desired state
    [pos_des, vel_des, acc_des, yaw_des, des_yawdot] = desired_state
    
    # access current state
    pos = state_vector[:3]
    vel = state_vector[3:6]
    # quat = np.array([state_vector[9]] + state_vector[6:9])  # Somebody decided to reverse the qw in state.py
    # R = quaternions.quat2mat(quat)
    # omega = np.array(state_vector[11:])

    # constants
    m = params.mass
    g = params.grav
    e3 = np.array([0, 0, 1])

    kp1_u, kd1_u, ki1_u = 10., 71., .0
    kp2_u, kd2_u, ki2_u = 10., 71., .0
    kp3_u, kd3_u, ki3_u = 10., 48., .0

    # Error
    # Eq 5a - 5b
    pos_error = pos_des - pos
    vel_error = vel_des - vel
    accumulated_error += pos_error
    # print pos_error

    # Desired acceleration
    # Eq 5c
    r1_acc = kp1_u * pos_error[0] + kd1_u * vel_error[0] + acc_des[0] + ki1_u * accumulated_error[0]
    r2_acc = kp2_u * pos_error[1] + kd2_u * vel_error[1] + acc_des[1] + ki2_u * accumulated_error[1]
    r3_acc = kp3_u * pos_error[2] + kd3_u * vel_error[2] + acc_des[2] + ki3_u * accumulated_error[2]

    r_acc_des = np.array([r1_acc, r2_acc, r3_acc]) + g*e3

    # Thrust
    thrust = m * r_acc_des

    # desired thrust vector
    return thrust


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


def modquad_torque_control(F, tau, structure, state_vector, motor_sat=False):
    """
    This function is similar to crazyflie_motion, but it is made for modular robots. So it specifies the dynamics
    of the modular structure. It receives a desired force and moment of a single robot.
    :param structure:
    :param motor_sat: motor saturation
    :param F: desired total thrust, 3 x 1 float vector
    :param M: desired moments, 3 x 1 float vector
    :param state_vector: the
    :return: thrust and moments for the whole structure, plus rotor forces of each propeller
    """
    ## From moments to rotor forces (power distribution)
    # positions of the rotors
    #         ^ X
    #    (4)  |      (1) [L, -L]
    #   Y<-----
    #    (3)         (2)
    # rx, ry = [], []
    # L = params.arm_length * sqrt(2) / 2.
    #
    # for x, y in zip(structure.xx, structure.yy):
    #     rx.append(x + L)
    #     rx.append(x - L)
    #     rx.append(x - L)
    #     rx.append(x + L)
    #     # y-axis
    #     ry.append(y - L)
    #     ry.append(y - L)
    #     ry.append(y + L)
    #     ry.append(y + L)
    #
    # sign_rx = [1 if rx_i > 0 else -1 for rx_i in rx]
    # sign_ry = [1 if ry_i > 0 else -1 for ry_i in ry]
    # # m = 4 * n  # Number of rotors
    # A = [[0.25, sy * .25 / L, -sx * .25 / L] for sx, sy in zip(sign_rx, sign_ry)]
    #
    # rotor_forces = np.dot(A, [F, M[0], M[1]])  # Not using moment about Z-axis for limits
    rotor_forces = None
    # record the rank of the A matrix of the structure
    rankA = structure.rankA
    # current rotation matrix
    quat = np.array([state_vector[9], state_vector[6], state_vector[7], state_vector[8]])
    R = quaternions.quat2mat(quat)
    e3 = np.array([0, 0, 1])
    # record the moment portion of the A matrix
    A_tau = structure.A[3:, :]
    if rankA == 4:
        # rank(A)=4: slightly modified SE3 controller
        # get a pivot tilting angle
        Rp_s = structure.quads[0].getRp()
        # desired force magnitude
        # Eq 8a
        f = F.dot(R.dot(Rp_s.dot(e3)))
        # determine the motor dynamics
        f_row = np.zeros([1, 4*structure.n])
        for i in range(structure.n):
            if not np.allclose(Rp_s, np.eye(3)) and \
                    np.allclose(structure.quads[i].getRp().dot(Rp_s), np.eye(3)):
                f_row[:, 4*i:4*(i+1)] = -1
            elif np.allclose(structure.quads[i].getRp(), Rp_s):
                f_row[:, 4*i:4*(i+1)] = 1
        # print f_row, A_tau
        A_motor = np.concatenate((f_row, A_tau))
        # pseudo-inverse to optimize over energy consumption
        rotor_forces = np.linalg.pinv(A_motor).dot(np.concatenate((np.array([f]), tau)))
    elif rankA == 5:
        pass
    else:
        rotor_forces = np.linalg.pinv(structure.A).dot(np.concatenate((F, tau)))
    # Failing motors
    for mf in structure.motor_failure:
        rotor_forces[4 * mf[0] + mf[1]] *= 0.0

    # Motor saturation
    if motor_sat:
        rotor_forces[rotor_forces > params.maxF / 4] = params.maxF / 4
        rotor_forces[rotor_forces < params.minF / 4] = params.minF / 4

    # From prop forces to total moments. Equation (1) of the modquad paper (ICRA 18)
    return F, tau, rotor_forces