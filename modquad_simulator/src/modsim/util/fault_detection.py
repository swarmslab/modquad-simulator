from math import sqrt
import rospy
import numpy as np

from modsim import params
from modsim.datatype.structure import Structure

from modquad_sched_interface.interface import rotpos_to_mat

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
    if not residual[0] and residual[1]:
        quadrant = 2
    elif not residual[0] and not residual[1]:
        quadrant = 3
    elif residual[0] and not residual[1]:
        quadrant = 4

    #print("Residual = {}".format(residual))
    #print("Quadrant of Fault seems to be: {}", quadrant)

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

def update_ramp_rotors(structure, t, next_t, groups, qidx, 
                        rotmat, ramp_rotor_set):
    """
    This function selects the sets of rotors we suspect of containing the fault
    It does not actually perform ramping of rotor thrust, only the selection
    """
    fdd_interval = rospy.get_param("fault_det_time_interval")
    next_t = t + fdd_interval

    # The full set of rotors we want to modify thrusts for
    ramp_rotor_set, qidx = _update_ramp_rotor_groups(structure, groups, 
                                                 qidx, rotmat, ramp_rotor_set)

    return next_t, ramp_rotor_set, qidx

def _update_ramp_rotor_groups(structure, groups, qidx, rotmat, ramp_rotor_set):
    """
    Moves the window of ramped rotors by one
    """
    ramp_up_set = []

    # Edge case when we first start diagnosis, don't want ramp_up_set then
    if len(ramp_rotor_set[0]) > 0: 
        ramp_up_set = groups[qidx]
        qidx += 1

        if qidx >= len(groups):
            print("None of groups has fault")
            print("Groups: {}".format(groups))
            raise Exception("Picked the wrong quadrant...")

    # The new set of rotors we are "suspecting"
    ramp_down_set = groups[qidx]

    # Return an updated ramp_rotor_set
    print("")
    print(rotmat)

    return [ramp_down_set, ramp_up_set], qidx

def _update_ramp_rotor_by_lines(structure, quadrant, qidx, 
                                rotmat, ramp_rotor_set):
    ramp_up_set = []

    print("")
    print("-------")
    print("")

    print(rotmat)
    import sys
    sys.exit(3)

    # Edge case when we first start diagnosis, don't want ramp_up_set then
    if len(ramp_rotor_set[0]) > 0: 
        ramp_up_set = [quadrant[qidx]]
        qidx += 1

        if qidx >= len(quadrant):
            print("Quadrant picked: {}".format(quadrant))
            raise Exception("Picked the wrong quadrant...")

    # The new set of rotors we are "suspecting"
    ramp_down_set = [quadrant[qidx]]

    raise Exception("This function is not implemented yet.")

def _get_rotor_position(rotor, structure):
    """
    :param rotor: tuple of (mod_id, rot_id)
    :param structure: a structure object
    :return pos: rotor pos relative to structure center of mass
    """
    # Get index where the module is stored in structure
    idx = structure.ids.index("modquad{:02d}".format(rotor[0]))
    x = structure.xx[idx]
    y = structure.yy[idx]

    # See modquad_torque_control(...) for why this param is used
    rotor_map_mode = rospy.get_param("rotor_map", 1)

    # Length from mod center of mass to rotor position
    L = params.arm_length * sqrt(2) / 2.0

    rid = rotor[1]

    # Compute the positions of all rotors in the structure
    if rotor_map_mode == 2:
        if rid == 0:
            return (x-L, y-L)
        elif rid == 1:
            return (x+L, y-L)
        elif rid == 2:
            return (x+L, y+L)
        return (x-L, y+L)
    else:
        raise Exception("Not handled yet")

def update_ramp_factors(t, next_t, ramp_factor):
    """
    This function updates ramping factors for ramping up and down so that we
    do not end up with a jerky transition from one set of rotors to another
    """
    fdd_interval = rospy.get_param("fault_det_time_interval")
    # Time left
    tdif = next_t - t
    if tdif < 0:
        raise Exception("Logic error in fault detect: tdif = next_t - t < 0")

    # No more changes left to do case
    if ramp_factor[0] < 0.0 or ramp_factor[1] > 1.0:
        return [0.0, 1.0]

    # Reach max extent of changes with 20% of fdd_interval left to go
    # 20% is arbitrary, can choose something else too
    ramp_down_factor = 0.5 * tdif / fdd_interval
    if ramp_down_factor < 0.1:
        ramp_down_factor = 0.0

    ramp_up_factor = 1.0 - ramp_down_factor

    return [ramp_down_factor, ramp_up_factor]

def form_groups(rot_list, rotmat):
    """
    Given a group of rotors and the rotmat, figure out sets of rotors to turn on
    and off for toggle searching
    :param rot_list: List of suspected rotors
    :param rotmat: Matrix telling us where each rotor is
    """

    groups = []
    gtype = rospy.get_param("fdd_group_type")

    # Each group is a single rotor
    if gtype == "indiv":
        groups = _form_indiv_rotor_groups(rot_list)
    elif gtype == "diag":
        groups = _form_diag_rotor_groups(rot_list, rotmat)
    elif gtype == "horz_line":
        groups = _form_hline_rotor_groups(rot_list, rotmat)
    else:
        raise Exception("Group type \"{}\" not implemented".format(gtype))

    return groups

def _form_indiv_rotor_groups(rot_list):
    """ Each group is a single rotor """
    return [[rotor] for rotor in rot_list]

def _form_diag_rotor_groups(rot_list, rotmat):
    """
    Given rotor matrix, divide into groups such that groups are diagonals
    """
    # Find all places where matrix is nonnegative
    indices = np.where(rotmat > -1)

    import pdb
    pdb.set_trace()
    
    pass

def _form_hline_rotor_groups(rot_list, rotmat):
    """
    Given rotor matrix, divide into groups such that groups are diagonals
    """
    # Temp dict to store each of the lines
    rotsets = {}

    # Find all places where matrix is nonnegative
    indices = np.where(rotmat > -1)

    for x in np.unique(indices[0]):
        rotsets[x] = []

    for x,y in list(zip(*indices)):
        rid = __get_rot_id(x, y)
        rotsets[x].append((int(rotmat[x, y]), rid))

    return [rotsets[x] for x in np.unique(indices[0])]

def __get_rot_id(x, y):
    """
    Given a position in the rotor matrix, determine what rotor id
    that corresponds to in the module
    """
    x = x % 2
    y = y % 2
    if x == 0 and y == 0: # Top left
        return 3
    elif x == 0: # Top right
        return 2
    elif y == 0: # Bottom left
        return 0
    return 1 # Bottom right
