import numpy as np

def vee_map(M):
    """
    :param M: skew symmetric matrix to be mapped
    :return: the vector that was mapped to
    """
    return [M[2,1], M[0,2], M[1, 0]]


def hat_map(v):
    """
    :param v: R3 vector to be mapped
    :return: the skew symmetric matrix
    """
    aux = v.flatten()
    A = np.array([[0, -aux[2], aux[1]], [aux[2], 0, -aux[0]], [-aux[1], aux[0], 0]])
    return A


def vecs_to_rot(v1, v2):
    """
    find the rotation matrix that rotates vector v1 to v2
    this function does not work in the case where v1u = -v2u
    :param v1: the vector before rotation
    :param v2: the vector after rotation
    :return: the rotation matrix
    """
    v1u = v1/np.linalg.norm(v1)
    v2u = v2/np.linalg.norm(v2)
    if np.allclose(v1u, v2u):
        return np.eye(3)
    vn = np.cross(v1u, v2u)
    vncol = np.array([vn]).T
    axis = vn/(1.0*np.linalg.norm(vn))
    R = v1u.dot(v2u) * np.eye(3) + hat_map(vn) - v1u.dot(v2u)*vncol.dot(vncol.T)
    return R
