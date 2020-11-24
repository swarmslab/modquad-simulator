import numpy as np
import copy
import sys
import math

#def zigzag(base, height, num_pt, start_pt=[0,0,0]):

def rect(base, height, start_pt=[0,0,0]):
    return np.array([start_pt,
                     [start_pt[0], start_pt[1]+height, start_pt[2]],
                     [start_pt[0]+base, start_pt[1]+height, start_pt[2]],
                     [start_pt[0]+base, start_pt[1], start_pt[2]],
                     start_pt])

def rising_rect(base, height, rise, start_pt=[0,0,0]):
    inc = rise / 4.0
    return np.array([start_pt,
                     [start_pt[0], start_pt[1]+height, start_pt[2] + inc],
                     [start_pt[0]+base, start_pt[1]+height, start_pt[2] + 2*inc],
                     [start_pt[0]+base, start_pt[1], start_pt[2] + 3*inc],
                     [start_pt[0], start_pt[1], start_pt[2] + rise]])
def spiral(base, height, rise, num_circ, start_pt=[0.0,0.0,0.0]):
    inc = rise / (num_circ + 0.0)
    perwinc = inc / 4.0
    arr = start_pt
    start = np.array(start_pt)
    ret = np.array(start_pt)
    for i in range(0, num_circ):
        a = np.array([[start[0], start[1]+height, start[2] + perwinc],
                      [start[0]+base, start[1]+height, start[2] + 2*perwinc],
                      [start[0]+base, start[1], start[2] + 3*perwinc],
                      [start[0], start[1], start[2] + 4*perwinc]])
        ret = np.vstack((ret, a))
        start = a[-1, :]
    return ret

def helix(radius, rise, num_circ, start_pt=[0.0,0.0,0.0]):
    theta_step = math.pi / 36.0 # Ten steps per circle
    per_circ_inc = rise / (num_circ + 0.0)
    height_step = per_circ_inc / 36.0
    height = start_pt[2]
    #loc = [start_pt[0], start_pt[1], start_pt[2]]
    loc = [0.0, 0.0, 0.0]
    waypts = [copy.copy(loc)]
    if not np.all(start_pt == 0):
        waypts.append(start_pt)
    theta = 0.0
    for i in range(int(num_circ)):
        theta = 0.0
        while theta < 2 * math.pi:
            next_pt = [radius*math.cos(theta), radius*math.sin(theta), height]
            waypts.append(next_pt)
            loc = copy.copy(next_pt)
            theta += theta_step
            height = height + height_step
            #print(theta)
    next_pt = [loc[0]+math.cos(theta), loc[1]+math.sin(theta), height+height_step]
    waypts.append(next_pt)
    waypts.append([next_pt[0], next_pt[1], 0.7]) # For safe descent
    waypts.append([next_pt[0], next_pt[1], 0.4]) # For safe descent
    waypts.append([next_pt[0], next_pt[1], 0.2]) # For safe descent
    waypts += np.array([start_pt[0], start_pt[1], start_pt[2]])
    print(" WAYPOINTS ")
    print(np.array(waypts))
    return np.array(waypts)

def hover_line(rise, start_pt=[0,0,0]):
    return np.array([start_pt, [start_pt[0], start_pt[1], start_pt[2]+rise]])

def line(start_pt=[0,0,0], end_pt=[1,1,1]):
    return np.array([start_pt, end_pt])

def waypt_set(list_of_waypts):
    return np.array(list_of_waypts)

def zigzag_xy(length, height, num_osc=2.0, start_pt=[0,0,0]):
    waypts = np.zeros((0,3))
    leninc = (length / num_osc)/2.0
    start = start_pt
    for i in range(0, int(num_osc)):
        a = np.array( [ [start[0]         , start[1]       , start[2]],
                        [start[0]+leninc  , start[1]+height, start[2]]
                       ])
        waypts = np.vstack((waypts, a))
        start = [start[0] + 2*leninc, start[1], start[2]]
    print(" WAYPOINTS ")
    print(np.array(waypts))
    return waypts

def zigzag_xz(length, height, num_osc=2.0, start_pt=[0,0,0]):
    waypts = np.zeros((0,3))
    leninc = (length / num_osc)/2.0
    ground = start_pt
    ground[-1] = 0

    waypts = np.vstack((waypts, ground))
    waypts = np.vstack((waypts, start_pt))
    start = start_pt

    for i in range(0, int(num_osc)):
        a = np.array( [ [start[0]         , start[1], start[2]       ],
                        [start[0]+leninc  , start[1], start[2]+height]
                       ])
        waypts = np.vstack((waypts, a))
        start = [start[0] + 2*leninc, start[1], start[2]]
    return waypts

def fixed_path(index):
    wayptset     = np.array([[0,0,0],
                             [1,0,0],
                             [2,0,0],
                             [2,1,0],
                             [1,1,0],
                             [0,1,0],
                             [0,0,0]])
    simple_waypt = np.array([[0,0,0],
                             [2,0,0],
                             [2,2,0],
                             [0,2,0],
                             [0,0,0]])
    zigzag = np.array([[ 0,0,0],
                       [ 1,1,0.5],
                       [ 2,2,1.0],
                       [ 3,1,1.5],
                       [ 4,0,2.0],
                       [ 5,1,2.5],
                       [ 6,2,3.0],
                       [ 7,1,3.5],
                       [ 8,0,4.0],
                       [ 9,1,4.5],
                       [10,2,5.0],
                       [11,1,5.5],
                       [12,0,6.0],
                       [13,1,6.5],
                       [14,2,7.0]])
    short_zigzag = np.array([[0  ,0  ,0  ],
                             [0.5,0.5,0.5],
                             [1  ,0.0,0.5],
                             [1.5,0.5,0.5],
                             [2  ,0.5,0.5],
                             [2.5,0.0,0.5],
                             [3  ,0.5,0.5]])
    inds = [wayptset, simple_waypt, zigzag, short_zigzag]
    return inds[index]
