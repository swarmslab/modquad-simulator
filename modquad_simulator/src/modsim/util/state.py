import numpy as np

from transforms3d import euler as trans

###### Initial state
from modsim.datatype.quadstate import QuadState


def init_state(pos, yaw):
    # INIT_STATE Initialize 13 x 1 state vector
    s = np.zeros(13)

    # Euler: phi, theta, psi
    qw, qx, qy, qz = trans.euler2quat(0, 0, yaw)

    #Ra = np.array(trans.quat2mat([qw, qx, qy, qz]))

    s[:3] = pos
    s[3:6] = 0.  # xdot, ydot, zdot
    s[6:10] = qx, qy, qz, qw
    s[10:13] = 0.  # p, q, r
    return s


########## Quadrotor function

def state_to_quadrotor(x):
    """
    %Converts qd struct used in hardware to x vector used in simulation
    :param x: is 1 x 13 vector of state variables [pos vel quat omega]
    :return: qd is a struct including the fields pos, vel, euler, and omega
    """

    # current state
    pos = x[:3]
    vel = x[3:6]
    qt = x[6:10]
    # I dont know why it has to be negative
    #euler = -np.array(trans.quat2euler([qw, qx, qy, qz]))
    #Ra = np.array(trans.quat2mat([qw, qx, qy, qz]))
    omega = x[10:]

    return pos, vel, qt, omega
