import numpy as np
from math import sin, cos, sqrt, pi


def RPYtoRot_ZXY(phi, theta, psi):
    """
    RPYtoRot_ZXY Converts roll, pitch, yaw to a body-to-world Rotation matrix
    The rotation matrix in this function is world to body [bRw] you will
    need to transpose this matrix to get the body to world [wRb] such that
    [wP] = [wRb] * [bP], where [bP] is a point in the body frame and [wP]
    is a point in the world frame
    written by Daniel Mellinger
    :param phi:
    :param theta:
    :param psi:
    :return:
    """
    R = [[cos(psi) * cos(theta) - sin(phi) * sin(psi) * sin(theta),
          cos(theta) * sin(psi) + cos(psi) * sin(phi) * sin(theta),
          -cos(phi) * sin(theta)],
         [-cos(phi) * sin(psi), cos(phi) * cos(psi), sin(phi)],
         [cos(psi) * sin(theta) + cos(theta) * sin(phi) * sin(psi),
          sin(psi) * sin(theta) - cos(psi) * cos(theta) * sin(phi),
          cos(phi) * cos(theta)]]
    return R


def matrix_to_quaternion(R):
    """
    ROTTOQUAT Converts a Rotation matrix into a Quaternion
    from the following website, deals with the case when tr<0
    http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
    https://github.com/moble/quaternion
    :param R: rotation matrix
    :return: quaternion
    """
    tr = R[0, 0] + R[1, 1] + R[2, 2]

    if tr > 0:
        S = sqrt(tr + 1.0) * 2  # S=4*qw
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif R[1, 1] > R[2, 2] and R[1, 1] > R[3, 3]:
        S = sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S=4*qx
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S=4*qy
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S=4*qz
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    q = np.array([qw, qx, qy, qz])
    q = q * np.sign(qw)

    return q


def quaternion_to_matrix(q):
    """
    Converts a quaternion into a rotation matrix.
    :param q: quaternion
    :return: 3x3 rotation matrix
    """
    q = np.array(q)

    q /= sqrt(sum(q ** 2))

    qahat = np.zeros((3, 3))
    qahat[0][1] = -q[2]
    qahat[0][2] = q[1]
    qahat[1][2] = -q[0]
    qahat[1][0] = q[2]
    qahat[2][0] = -q[1]
    qahat[2][1] = q[0]

    R = np.eye(3) + 2 * np.dot(qahat, qahat) + 2 * q[3] * qahat
    return R
