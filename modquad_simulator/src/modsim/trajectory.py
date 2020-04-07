from scipy import linalg
from math import sin, cos, sqrt, pi
import numpy as np
import sys


class traj_data:
    """
    This class is used to store the data we only need to compute once
    for the trajectory. Specifically used in min snap trajectory
    """

    def __init__(self, times, dists, totaldist, waypts, cx, cy, cz):
        self.times = times
        self.dists = dists
        self.total_dist = totaldist
        self.waypts = waypts
        self.cx = cx
        self.cy = cy
        self.cz = cz


def trajectory_GeometricSO3(tt):
#    x_d = np.array([0.4 * tt, 0.4 * np.sin(np.pi * tt), 0.6 * np.cos(np.pi * tt)])
#    vel_d = np.array([0.4, 0.4 * np.pi * np.cos(np.pi * tt), -0.6 * np.pi * np.sin(np.pi * tt)])
#    accel_d = np.array([0, -0.4 * np.pi * np.pi * np.sin(np.pi * tt), -0.6 * np.pi * np.pi * np.cos(np.pi * tt)])



    x_d = np.array([0.04 * tt, 0.004 * np.sin(np.pi * tt), 0.006 * np.cos(np.pi * tt)])
    vel_d = np.array([0.04, 0.004 * np.pi * np.cos(np.pi * tt), -0.006 * np.pi * np.sin(np.pi * tt)])
    accel_d = np.array([0, -0.004 * np.pi * np.pi * np.sin(np.pi * tt), -0.006 * np.pi * np.pi * np.cos(np.pi * tt)])

    x_d = np.array([0., 0., 1.])
    vel_d = np.array([0., 0., 0.])
    accel_d = np.array([0., 0., 0.])

    b1d = np.array([np.cos(np.pi * tt), np.sin(np.pi * tt), 0])

    b1d = np.array([1., 0., 0.])

    if x_d.size == 3:
        x_d.shape = (3, 1)
        vel_d.shape = (3, 1)
        accel_d.shape = (3, 1)
        b1d.shape = (3, 1)

 #   vec_d = np.concatenate()

    return x_d, vel_d, accel_d, b1d

def circular_trajectory(t, t_max=30):
    """
    Circular trajectory.
    :param t_max:
    :param t:
    :return:
    """
    A = [[1, 0, 0, 0, 0, 0],
         [1, t_max, t_max ** 2, t_max ** 3, t_max ** 4, t_max ** 5],
         [0, 1, 0, 0, 0, 0],
         [0, 1, 2 * t_max, 3 * t_max ** 2, 4 * t_max ** 3, 5 * t_max ** 4],
         [0, 0, 2, 0, 0, 0],
         [0, 0, 2, 6 * t_max, 12 * t_max ** 2, 20 * t_max ** 3]]

    b = [0, 1, 0, 0, 0, 0]
    a = linalg.solve(A, b)

    q = a[0] + a[1] * t + a[2] * (t ** 2) + a[3] * (t ** 3) + a[4] * (t ** 4) + a[5] * (t ** 5)
    qv = a[1] + 2 * a[2] * t + 3 * a[3] * (t ** 2) + 4 * a[4] * t ** 3 + 5 * a[5] * t ** 4
    qa = 2 * a[2] + 6 * a[3] * t + 12 * a[4] * t ** 2 + 20 * a[5] * t ** 3

    r = 1.  # radius of the circle
    pos = [r * cos(2 * pi * q), r * sin(2 * pi * q), 0 * 2.5 * q + 0.2]
    vel = [-r * 2 * pi * sin(2 * pi * q) * qv, r * 2 * pi * cos(2 * pi * q) * qv, 0 * 2.5 * qv]
    acc = [r * (-2 * pi * sin(2 * pi * q) * qa - 4 * pi ** 2 * cos(2 * pi * q) * qv ** 2),
           r * (2 * pi * cos(2 * pi * q) * qa - 4 * pi ** 2 * sin(2 * pi * q) * qv ** 2), 0 * 2.5 * qa]

    yaw = 0
    yawdot = 0
    return [pos, vel, acc, yaw, yawdot]


def simple_waypt_trajectory(waypts, t, t_max=30):
    """
    NOTE: THIS IS EXTREMELY SIMPLE AND NONOPTIMAL
    t is current time
    t_max is target time to complete trajectory
    waypoints is numpy matrix of shape [n, 3], where n is # waypts
    """
    if len(waypts) < 2:
        raise ValueError("Not enough waypoints")

    # Find distances between waypts
    dists = np.sqrt(np.sum(((np.roll(waypts, 1, axis=0) - waypts)[1:, :]) ** 2, axis=1))
    dists = np.hstack([np.array([0]), dists])
    totaldist = np.sum(dists)

    # Target times for each leg of journey
    times = np.cumsum([dists[i] / totaldist * t_max for i in range(0, len(dists))])

    if t == 0:
        pos = waypts[0, :]
        vel = [0, 0, 0]
        acc = [0, 0, 0]
    else:
        ind = 0
        ind = [i for i in range(0, len(times) - 1) if t < times[i + 1] and t >= times[i]]
        ind = ind[0]
        last_waypt = waypts[ind, :]
        next_waypt = waypts[ind + 1, :]
        last_time = times[ind]
        next_time = times[ind + 1]
        percent_complete = (t - last_time) / (next_time - last_time)
        pos = last_waypt + percent_complete * next_waypt
        vel = (next_waypt - last_waypt) / (next_time - last_time)
        acc = [0, 0, 0]

    yaw = 0
    yawdot = 0
    return [pos, vel, acc, yaw, yawdot]


def _min_snap_init(waypts, t_max=30):
    """
    This function is called once at the beginning of the run for min snap trajectory 
    planning, in which we compute coeffs for the equation of motion describing 
    all segments of the path
    :param: waypts is the Nx3 set of (x,y,z) triples we want to hit
    :param: t_max is the time we want to complete the trajectory in
    """
    # Find distances between waypts
    dists = np.sqrt(np.sum(((np.roll(waypts, 1, axis=0) - waypts)[1:, :]) ** 2, axis=1))
    totaldist = np.sum(dists)

    # Target times for each waypt
    times = [0] + [dists[i] / totaldist * t_max for i in range(0, len(dists))]
    times = np.cumsum(np.array(times))

    num_eq = len(waypts) - 1
    num_unknown = num_eq * 8
    M = np.zeros((num_unknown, num_unknown))
    x = np.zeros((num_unknown, 1))
    y = np.zeros((num_unknown, 1))
    z = np.zeros((num_unknown, 1))
    rows = np.arange(0, 4)  # First waypoint modifies these rows
    cols = np.arange(0, 8)  # and these columns
    np.set_printoptions(precision=1)
    np.set_printoptions(suppress=True)
    np.set_printoptions(threshold=sys.maxsize)
    for i in range(0, len(waypts)):
        T = times[i]
        if i == 0 or i == len(waypts) - 1:
            if i == len(waypts) - 1:
                rows = rows[0:4]
            A = np.array([  # Used for begin/end of trajectory
                [T ** 7, T ** 6, T ** 5, T ** 4, T ** 3, T ** 2, T ** 1, T ** 0],
                [7 * T ** 6, 6 * T ** 5, 5 * T ** 4, 4 * T ** 3, 3 * T ** 2, 2 * T, 1, 0],
                [42 * T ** 5, 30 * T ** 4, 20 * T ** 3, 12 * T ** 2, 6 * T, 2, 0, 0],
                [210 * T ** 4, 120 * T ** 3, 60 * T ** 2, 24 * T, 6, 0, 0, 0],
            ])
            M[rows[0]:rows[-1] + 1, cols[0]:cols[-1] + 1] = A
            # x, y, z are all single columns
            x[rows] = np.transpose(np.reshape(np.array([waypts[i, 0], 0, 0, 0]), [1, 4]))
            y[rows] = np.transpose(np.reshape(np.array([waypts[i, 1], 0, 0, 0]), [1, 4]))
            z[rows] = np.transpose(np.reshape(np.array([waypts[i, 2], 0, 0, 0]), [1, 4]))
            rows = np.arange(rows[-1] + 1, rows[-1] + 9)
        else:
            r1 = rows[0:-1]
            c1 = cols
            r2 = rows[1:]
            c2 = cols + 8
            # Matrix of all eq of segment that is ending at the next waypoint
            A = np.array([  # Used for intermediate points
                [T ** 7, T ** 6, T ** 5, T ** 4, T ** 3, T ** 2, T ** 1, 1],
                [7 * T ** 6, 6 * T ** 5, 5 * T ** 4, 4 * T ** 3, 3 * T ** 2, 2 * T, 1, 0],
                [42 * T ** 5, 30 * T ** 4, 20 * T ** 3, 12 * T ** 2, 6 * T, 2, 0, 0],
                [210 * T ** 4, 120 * T ** 3, 60 * T ** 2, 24 * T, 6, 0, 0, 0],
                [840 * T ** 3, 360 * T ** 2, 120 * T, 24, 0, 0, 0, 0],
                [2520 * T ** 2, 720 * T, 120, 0, 0, 0, 0, 0],
                [5040 * T, 720, 0, 0, 0, 0, 0, 0],
            ])
            M[r1[0]:r1[-1] + 1, c1[0]:c1[-1] + 1] = A
            m2 = -A[1:, :]
            pvec = np.reshape(A[0, :], [1, 8])
            M[r2[0]:r2[-1] + 1, c2[0]:c2[-1] + 1] = np.vstack([m2, pvec])
            # x, y, z are all single columns
            x[rows] = np.transpose(np.reshape(np.array([waypts[i, 0], 0, 0, 0, 0, 0, 0, waypts[i, 0]]), [1, 8]))
            y[rows] = np.transpose(np.reshape(np.array([waypts[i, 1], 0, 0, 0, 0, 0, 0, waypts[i, 1]]), [1, 8]))
            z[rows] = np.transpose(np.reshape(np.array([waypts[i, 2], 0, 0, 0, 0, 0, 0, waypts[i, 2]]), [1, 8]))
            rows = rows + 8
            cols = c2
    # Solve for the equation coeffs
    cx = np.linalg.solve(M, x)
    cy = np.linalg.solve(M, y)
    cz = np.linalg.solve(M, z)
    return traj_data(times, dists, totaldist, waypts, cx, cy, cz)


def min_snap_trajectory(t, t_max=30, traj_vars=None, waypts=[]):
    """
    This is not optimized. Waypoint pruning/adding and cubic splining
    the path has not been implemented yet.
    First, call this function by passing in the Nx3 matrix of waypts 
    consisting of (x,y,z) triples. 
    Then, during the run, this is called to get the next desired state
    vector for the robot.
    :param: t is the current time
    :param: t_max is the time to complete the trajectory in
    :param: traj_vars is the object containing the persistent vars
    :param: waypts is the set N waypts we want to hit (Nx3 matrix)
    """
    if len(waypts) > 0:  # i.e. waypts passed in, then initialize
        traj_vars = _min_snap_init(waypts, t_max)
        return traj_vars
    # find where we are in the trajectory
    if traj_vars is None:
        raise ValueError("No trajectory data passed in")
    # TODO not run traj on loop...
    t = t % t_max  # Run the trajectory on a loop
    ind = [i for i in range(0, len(traj_vars.times) - 1)
           if t >= traj_vars.times[i] and t < traj_vars.times[i + 1]]
    if len(ind) != 1:
        print 'ind = ', ind
        raise ValueError("Malformed times vector for legs of journey")
    ind = ind[0]
    prev = traj_vars.waypts[ind, :]
    dest = traj_vars.waypts[ind + 1, :]
    pdiff = dest - prev
    leglen = np.sqrt(np.sum(pdiff * pdiff))
    tphase = (leglen / traj_vars.total_dist) * t_max
    tdiff = t  # tbegin is at t=0
    cind = (ind + 1) * 8 - 1
    cind = range(cind - 7, cind + 1)  # array from [cind-7 : cind]
    A = np.array([
        [t ** 7, t ** 6, t ** 5, t ** 4, t ** 3, t ** 2, t ** 1, t ** 0],
        [7 * t ** 6, 6 * t ** 5, 5 * t ** 4, 4 * t ** 3, 3 * t ** 2, 2 * t, 1, 0],
        [42 * t ** 5, 30 * t ** 4, 20 * t ** 3, 12 * t ** 2, 6 * t, 2, 0, 0],
    ])
    coeffs = np.squeeze(np.stack([traj_vars.cx[cind[0]:cind[-1] + 1],
                                  traj_vars.cy[cind[0]:cind[-1] + 1],
                                  traj_vars.cz[cind[0]:cind[-1] + 1]], axis=1))
    res = np.dot(A, coeffs)
    pos = res[0, :]
    vel = res[1, :]
    acc = res[2, :]
    yaw = 0
    yawdot = 0
    return [pos, vel, acc, yaw, yawdot]
