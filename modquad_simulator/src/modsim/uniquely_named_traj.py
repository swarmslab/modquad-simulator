from scipy import linalg, interpolate as interp
from math import sqrt
import numpy as np

class min_snap_traj_data:
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
    times = np.array([0] + [dists[i] / totaldist * t_max for i in range(len(dists))])
    time_inflate = np.ones((times.shape[0]))

    times = np.cumsum(times) + t # Account for time elapsed since sim began

    # Spline the path
    tstep = 1.00
    newtimes = np.arange(0,t_max+tstep,tstep) + t

    xq = interp.pchip_interpolate(times, waypts[:,0], newtimes)
    yq = interp.pchip_interpolate(times, waypts[:,1], newtimes)
    zq = interp.pchip_interpolate(times, waypts[:,2], newtimes)

    start = waypts[0, :]
    end = waypts[-1, :]
    waypts = np.transpose(np.vstack((xq, yq, zq)))
    waypts[0, :] = start
    waypts[-1, :] = end
    times = np.arange(0,t_max+tstep,tstep) + t

    num_eq = len(waypts) - 1
    num_unknown = num_eq * 8
    M = np.zeros((num_unknown, num_unknown))
    x = np.zeros((num_unknown, 1))
    y = np.zeros((num_unknown, 1))
    z = np.zeros((num_unknown, 1))
    rows = np.arange(0, 4)  # First waypoint modifies these rows
    cols = np.arange(0, 8)  # and these columns

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
    return min_snap_traj_data(times, dists, totaldist, waypts, cx, cy, cz)

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

    pdiff = dest - prev
    leglen = np.sqrt(np.sum(pdiff * pdiff))
    tphase = (leglen / traj_vars.total_dist) * t_max
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
    if ret_snap:
        return res[4,:]
    else:
        pos = res[0,:]
        vel = res[1,:]
        acc = res[2,:]
        yaw = 0
        yawdot = 0
        return [pos, vel, acc, yaw, yawdot]

