from math import sqrt
import rospy
import numpy as np

from modsim import params
from modsim.datatype.structure import Structure

def fault_exists(residual):
    """
    @param residual 13x1 state vector where last three elems are ang vel [p q r]
    """
    # The fault appears in the p and q elements, ignore r element
    residual = residual[-3:-1]

    # Experimentally saw min residual ~0.3 and hover around 0 if no fault
    # Thus, picked middle ground of 0.2
    threshold = 0.2
    fault_exists = sum(abs(residual) > threshold) 
    return bool(fault_exists)


def get_faulty_quadrant_rotors(residual, structure):
    """
    This function uses the signs of the residual vector elements [p q] to
    determine which quadrant of the structure the fault is in
    Signs correspond to following mapping (+- means positive p, negative q)
    Each quadrant is labeled with the quadrant number, maps to Cartesian plane
        sign convention.
           ^
           |
     3 --  |  -+ 2
    <------------->
     4 +-  |  ++ 1
           |
           v

    @return list of tuples of (mod_id, rot_id) to indicate which rotors
            potentially faulty
    """

    # See modquad_torque_control(...) for why this param is used
    rotor_map_mode = rospy.get_param("rotor_map", 1)

    # Lists of all rotor x and y positions in structure frame
    rx, ry = [], []

    # Length from mod center of mass to rotor position
    L = params.arm_length * sqrt(2) / 2.0

    # Compute the positions of all rotors in the structure
    for x, y in zip(structure.xx, structure.yy):
        if rotor_map_mode == 1:
            # x-axis
            rx.append(x + L) # R
            rx.append(x - L)
            rx.append(x - L)
            rx.append(x + L)
            # y-axis
            ry.append(y - L)
            ry.append(y - L)
            ry.append(y + L)
            ry.append(y + L)
        else:
            # x-axis
            rx.append(x - L) # R
            rx.append(x + L)
            rx.append(x + L)
            rx.append(x - L)
            # y-axis
            ry.append(y - L)
            ry.append(y - L)
            ry.append(y + L)
            ry.append(y + L)

    # Determine which quadrant the fault is in
    residual = residual[-3:-1]
    residual = residual > 0 # Convert to sign-based
    quadrant = 1
    if residual[0] < 0 and residual[1] > 0:
        quadrant = 2
    elif residual[0] < 0 and residual[1] < 0:
        quadrant = 3
    elif residual[0] > 0 and residual[1] < 0:
        quadrant = 4

    # Determine which rotor positions are in which quadrant by using signs
    sign_rx = np.array([1 if rx_i > 0 else -1 for rx_i in rx])
    sign_ry = np.array([1 if ry_i > 0 else -1 for ry_i in ry])
    if quadrant == 1:
        x = sign_rx > 0
        y = sign_ry < 0
    elif quadrant == 2:
        x = sign_rx > 0
        y = sign_ry > 0
    elif quadrant == 3:
        x = sign_rx < 0
        y = sign_ry > 0
    else:
        x = sign_rx < 0
        y = sign_ry < 0

    # Find the rotor indices corresponding to quadrant we want
    mask = x * y
    inds = list(np.where(mask > 0)[0])

    # Tuples (mod_id, rotor_id) where fault could potentially be based on
    # quadrant the fault is IDed to be in
    return [(int(r/4)+1, r - 4*int(r/4)) for r in inds]
