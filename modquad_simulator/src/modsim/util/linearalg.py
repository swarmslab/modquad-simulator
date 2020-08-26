import numpy as np

def vee_map(M):
    """
    :param M: skew symmetric matrix to be mapped
    :return: the vector that was mapped to
    """
    return [M[2,1], M[0,2], M[1, 0]]


def hat_map(aux):
    """
    :param aux: R3 vector to be mapped
    :return: the skew symmetric matrix
    """
    A = np.array([[0, -aux[2, 0], aux[1, 0]], [aux[2, 0], 0, -aux[0, 0]], [-aux[1, 0], aux[0, 0], 0]])
    return A