from scipy import linalg, interpolate as interp
from math import sin, cos, sqrt, pi
import matplotlib.pyplot as plt
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


def circular_trajectory(t, t_max=30, traj_vars=None, waypts=[], ret_snap=False):
    """
    Circular trajectory.
    :param t_max:
    :param t:
    :return:
    NOTE: this will not use traj_vars, waypts, or ret_snap, but these params included
            so that all trajectory functions have identical signatures
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


def simple_waypt_trajectory(t, t_max=30, traj_vars=None, waypts=[], ret_snap=False):
    """
    NOTE: THIS IS EXTREMELY SIMPLE AND NONOPTIMAL
    :param t: is current time
    :param t_max: is target time to complete trajectory
    :param waypts: is numpy matrix of shape [n, 3], where n is # waypts
    NOTE: this will not use traj_vars or ret_snap, but these params included
            so that all trajectory functions have identical signatures
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


def _min_snap_init(waypts, speed=0.5, t=0.0, use_splines=True):
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
    t_max = (totaldist / speed) # Projected time this traj will take to run
    cumdist = np.cumsum(dists)
    if cumdist[0] != 0:
        cumdist = np.insert(cumdist, 0, 0) 

    # Target times for each waypt
    #print('vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv')
    #print('waypts = \n{}'.format(waypts))
    #print("dists = {}".format(dists))
    #print("totaldist = {}".format(totaldist))
    #print("projected time = {}".format(t_max))
    #print("cumdist = {}".format(cumdist))
    #for i in range(len(dists)):
    #    print("\tnt = {}".format(dists[i] / totaldist * t_max))
    times = np.array([0] + [dists[i] / totaldist * t_max for i in range(len(dists))])
    time_inflate = np.ones((times.shape[0]))
    #if len(times) > 2:
    #    time_inflate[-3:] *= np.array([1.1, 1.25, 1.3])
    #    times *= time_inflate

    times = np.cumsum(times) + t # Account for time elapsed since sim began
    #sys.exit(0)
    #plt.plot(waypts[:, 0], waypts[:, 1])

    # Spline the path
    tstep = 1.00
    #print(times)
    #print(waypts[:,0])
    newtimes = np.arange(0,t_max+tstep,tstep) + t

    #print("tmax = {}".format(t_max))
    #print("dist = {}".format(totaldist))
    #print("speed = {}".format(speed))
    #print(newtimes)
    #print(np.transpose(waypts))

    #if use_splines or True:
    xq = interp.pchip_interpolate(times, waypts[:,0], newtimes)
    yq = interp.pchip_interpolate(times, waypts[:,1], newtimes)
    zq = interp.pchip_interpolate(times, waypts[:,2], newtimes)
    #xq = interp.barycentric_interpolate(times, waypts[:,0], newtimes)
    #yq = interp.barycentric_interpolate(times, waypts[:,1], newtimes)
    #zq = interp.barycentric_interpolate(times, waypts[:,2], newtimes)
    #else:
    #    print(times)
    #    print(waypts)
    #    print(newtimes)
    #    fxq = interp.interp1d(times, waypts[:,0])
    #    fyq = interp.interp1d(times, waypts[:,1])
    #    fzq = interp.interp1d(times, waypts[:,2])
    #    xq = fxq(newtimes)
    #    yq = fyq(newtimes)
    #    zq = fzq(newtimes)

    start = waypts[0, :]
    end = waypts[-1, :]
    #print(times)
    #print(waypts)
    waypts = np.transpose(np.vstack((xq, yq, zq)))
    waypts[0, :] = start
    waypts[-1, :] = end
    times = np.arange(0,t_max+tstep,tstep) + t
    #print(waypts)
    #print(newtimes)
    #print(xq)
    #print(yq)
    #print(zq)
    #print('--')
    #print(np.transpose(waypts))
    #plt.figure()
    #plt.scatter(waypts[:, 0], waypts[:, 1])
    #plt.show()

    num_eq = len(waypts) - 1
    num_unknown = num_eq * 8
    M = np.zeros((num_unknown, num_unknown))
    x = np.zeros((num_unknown, 1))
    y = np.zeros((num_unknown, 1))
    z = np.zeros((num_unknown, 1))
    rows = np.arange(0, 4)  # First waypoint modifies these rows
    cols = np.arange(0, 8)  # and these columns
    np.set_printoptions(precision=3)
    np.set_printoptions(suppress=True)
    np.set_printoptions(threshold=sys.maxsize)
    for i in range(0, len(waypts)):
        T = times[i]
        if i == 0 or i == len(waypts) - 1:
            if i == len(waypts) - 1:
                rows = rows[0:4]
            A = np.array([  # Used for begin/end of trajectory
                [      T ** 7,       T ** 6,      T ** 5,      T ** 4,     T ** 3,     T ** 2, T ** 1, T ** 0],
                [  7 * T ** 6,   6 * T ** 5,  5 * T ** 4,  4 * T ** 3, 3 * T ** 2, 2 * T     , 1     , 0     ],
                [ 42 * T ** 5,  30 * T ** 4, 20 * T ** 3, 12 * T ** 2, 6 * T     , 2         , 0     , 0     ],
                [210 * T ** 4, 120 * T ** 3, 60 * T ** 2, 24 * T     , 6         , 0         , 0     , 0     ],
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
            A = np.array([ # Used for intermediate points
                    [       T ** 7,       T ** 6,       T ** 5,      T ** 4,     T ** 3,     T ** 2, T ** 1, 1 ],
                    [   7 * T ** 6,   6 * T ** 5,   5 * T ** 4,  4 * T ** 3, 3 * T ** 2, 2 * T     , 1     , 0 ],
                    [  42 * T ** 5,  30 * T ** 4,  20 * T ** 3, 12 * T ** 2, 6 * T     , 2         , 0     , 0 ],
                    [ 210 * T ** 4, 120 * T ** 3,  60 * T ** 2, 24 * T     , 6         , 0         , 0     , 0 ],
                    [ 840 * T ** 3, 360 * T ** 2, 120 * T     , 24         , 0         , 0         , 0     , 0 ],
                    [2520 * T ** 2, 720 * T     , 120         ,  0         , 0         , 0         , 0     , 0 ],
                    [5040 * T     , 720         ,   0         ,  0         , 0         , 0         , 0     , 0 ],
                ])
            M[r1[0]:r1[-1]+1, c1[0]:c1[-1]+1] = A # Load A into the overall coeff matrix
            m2 = -A[1:, :] # The corresponding entries for the next segment
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
    #print("Projected time at end of planning: {}".format(times[-1]-times[0]))
    return traj_data(times, dists, totaldist, waypts, cx, cy, cz)

def min_snap_trajectory(t, speed=1, traj_vars=None, waypts=None, 
        ret_snap=False, use_splines=True):
    """
    This is not optimized. Waypoint pruning/adding and cubic splining
    the path has not been implemented yet.
    First, call this function by passing in the Nx3 matrix of waypts 
    consisting of (x,y,z) triples. 
    Then, during the run, this is called to get the next desired state
    vector for the robot.
    :param t: current time
    :param t_max: time to complete the trajectory in
    :param traj_vars: object containing the persistent vars
    :param waypts: set of N waypts we want to hit (Nx3 matrix)
    :param ret_snap: if true, return just the snap, not [pos, vel, acc, yaw, yawdot]
    :param use_splines: Use cubic splines to smooth the trajectory
    """
    if waypts is not None:  # i.e. waypts passed in, then initialize
        traj_vars = _min_snap_init(waypts, speed, t, use_splines)
        return traj_vars
    # find where we are in the trajectory
    if traj_vars is None:
        raise ValueError("No trajectory data passed in")
    t_max = traj_vars.times[-1]
    if t >= t_max:
        pos = traj_vars.waypts[-1,:]
        vel = [0.0, 0.0, 0.0]
        acc = [0.0, 0.0, 0.0]#9.81]
        yaw = 0
        yawdot = 0
        return [pos, vel, acc, yaw, yawdot]
        #t = t_max # Hover at final spot
    ind = [i for i in range(0, len(traj_vars.times) - 1)
           if t >= traj_vars.times[i] and t < traj_vars.times[i + 1]]
    if len(ind) != 1 and t != t_max:
        #print 'ind = ', ind
        print(traj_vars.times)
        print("Time = {}, Max Time = {}".format(t, t_max))
        raise ValueError("Malformed times vector for legs of journey")
    if t == t_max:
        ind = -2
    else:
        ind = ind[0]
    prev = traj_vars.waypts[ind, :]
    dest = traj_vars.waypts[ind + 1, :]

    #if t == t_max:
    #    print("prev waypt = {}".format(prev))
    #    print("dest waypt = {}".format(dest))

    pdiff = dest - prev
    leglen = np.sqrt(np.sum(pdiff * pdiff))
    tphase = (leglen / traj_vars.total_dist) * t_max
    #tdiff = t # tbegin is at t=0
    cind = (ind+1) * 8 - 1
    cind = range(cind-7, cind+1) # array from [cind-7 : cind]
    A = [] # to preserve scope
    if ret_snap:
        A = np.array([
                [      t ** 7,       t ** 6,       t ** 5,      t ** 4,     t ** 3,     t ** 2, t ** 1, 1 ],
                [  7 * t ** 6,   6 * t ** 5,   5 * t ** 4,  4 * t ** 3, 3 * t ** 2, 2 * t     , 1     , 0 ],
                [ 42 * t ** 5,  30 * t ** 4,  20 * t ** 3, 12 * t ** 2, 6 * t     , 2         , 0     , 0 ],
                [210 * t ** 4, 120 * t ** 3,  60 * t ** 2, 24 * t     , 6         , 0         , 0     , 0 ],
                [840 * t ** 3, 360 * t ** 2, 120 * t     , 24         , 0         , 0         , 0     , 0 ]
            ])
    else:
        A = np.array([
                [     t ** 7,       t ** 6,      t ** 5,      t ** 4,     t ** 3,     t ** 2, t ** 1, 1 ],
                [ 7 * t ** 6,   6 * t ** 5,  5 * t ** 4,  4 * t ** 3, 3 * t ** 2, 2 * t     , 1     , 0 ],
                [42 * t ** 5,  30 * t ** 4, 20 * t ** 3, 12 * t ** 2, 6 * t     , 2         , 0     , 0 ]
            ])
    coeffs = np.squeeze(np.stack([traj_vars.cx[cind[0]:cind[-1]+1] , 
                                  traj_vars.cy[cind[0]:cind[-1]+1] , 
                                  traj_vars.cz[cind[0]:cind[-1]+1]], axis=1))
    res = np.dot(A, coeffs)
    #print("At t = {:03f}, we multiply: ".format(t))
    #print(A)
    #print(coeffs)
    if ret_snap:
        return res[4,:]
    else:
        pos = res[0,:]
        vel = res[1,:]
        acc = res[2,:]
        yaw = 0
        yawdot = 0
        #print("\tAt t = {:03f}, pos = {}, vel = {}, acc = {}".format(t, pos, vel, acc))
        #print('---')
        return [pos, vel, acc, yaw, yawdot]

