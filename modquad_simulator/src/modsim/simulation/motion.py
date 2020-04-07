from modsim import params
from modsim.util.quaternion import quaternion_to_matrix, matrix_to_quaternion, RPYtoRot_ZXY
import numpy as np


def state_derivative(state_vector, F, M, structure):
    """
    Calculate the derivative of the state vector. This is the function that needs to be integrated to know the next
    state of the robot.
    :param t: time
    :param state_vector: 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
    :param F: thrust input for a single robot
    :param M: 3 x 1, moments output from controller (only used in simulation)
    :return: sdot: 13 x 1, derivative of state vector s
    """

    ## State of the quadrotor
    [xdot, ydot, zdot] = state_vector[3:6]  # Linear velocity
    quat = state_vector[6:10]  # orientation
    [qX, qY, qZ, qW] = quat
    bRw = quaternion_to_matrix(quat)
    wRb = bRw.T  # Orientation in matrix form
    omega = state_vector[10:]  # angular velocity
    [p, q, r] = omega

    # Angular velocity
    K_quat = 2.  # this enforces the magnitude 1 constraint for the quaternion
    quat_error = 1 - (qW ** 2 + qX ** 2 + qY ** 2 + qZ ** 2)  # Quaternion error
    qdot = (-1. / 2) * np.dot(np.array([[0, -p, -q, -r],
                                        [p, 0, -r, q],
                                        [q, r, 0, -p],
                                        [r, -q, p, 0]]), [qW, qX, qY, qZ])
    qdot += K_quat * quat_error * np.array([qW, qX, qY, qZ])
    qdot = [qdot[1], qdot[2], qdot[3], qdot[0]]  # reorder the quaternion based on the ROS quaternion representation.

    # Angular acceleration
    angular_acceleration = np.dot(structure.inverse_inertia,
                                  (M - np.cross(omega, np.dot(structure.inertia_tensor, omega))))

    # Acceleration
    gravity_vector = np.array([0, 0, structure.n * params.mass * params.grav])
    linear_acceleration = (np.dot(wRb, [0, 0, F]) - gravity_vector) / params.mass

    ## Assemble the derivative of the state
    sdot = np.zeros(13)
    sdot[:3] = [xdot, ydot, zdot]  # linear velocity
    sdot[3:6] = linear_acceleration  # linear acceleration
    sdot[6:10] = qdot  # angular velocity as a quaternion
    sdot[10:] = angular_acceleration  # angular acceleration

    return sdot

