from math import sqrt
import rospy
import numpy as np
import json

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

def update_rotmat(rot_list, rotmat):
    """
    For those rotors not in the rot_list, 
    make their entries in the rotmat be -1
    """
    save_list = []
    for rot in rot_list:
        mat_idx = np.where(rotmat == rot[0])

        # Get x,y of module position
        sx = int(mat_idx[0][0] / 2)
        sy = int(mat_idx[1][0] / 2)
        save_list.append([sx, sy, rot[0], rot[1]])
    rotmat[rotmat > -1] = -1

    for rot in save_list:
        x = rot[0]
        y = rot[1]
        # Label each rotor position with its module ID
        if rot[3] == 0:
            rotmat[2*x + 1, 2*y    ] = rot[2]
        elif rot[3] == 1:
            rotmat[2*x + 1, 2*y + 1] = rot[2]
        elif rot[3] == 2:
            rotmat[2*x    , 2*y + 1] = rot[2]
        else: # rot[3] == 3
            rotmat[2*x    , 2*y    ] = rot[2]

    return rotmat

def update_ramp_rotors(structure, t, next_t, groups, qidx, 
                        rotmat, ramp_rotor_set):
    """
    This function selects the sets of rotors we suspect of containing the fault
    It does not actually perform ramping of rotor thrust, only the selection
    """
    # The full set of rotors we want to modify thrusts for
    ramp_rotor_set, qidx = _update_ramp_rotor_groups(structure, groups, 
                                                 qidx, rotmat, ramp_rotor_set)

    return ramp_rotor_set, qidx

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
    elif gtype == "vert_line":
        groups = _form_vline_rotor_groups(rot_list, rotmat)
    elif gtype == "log4":
        groups = _form_log4_rotor_groups(rot_list, rotmat)
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
    Given rotor matrix, divide into groups such that groups are horizontals
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

def _form_vline_rotor_groups(rot_list, rotmat):
    """
    Given rotor matrix, divide into groups such that groups are groups of
    verticals
    """
    print(rotmat)
    # Temp dict to store each of the lines
    rotsets = {}

    # Find all places where matrix is nonnegative
    indices = np.where(rotmat > -1)

    for y in np.unique(indices[1]):
        rotsets[y] = []

    # import pdb
    # pdb.set_trace()

    for x,y in list(zip(*indices)):
        rid = __get_rot_id(x, y)
        rotsets[y].append((int(rotmat[x, y]), rid))

    return [rotsets[y] for y in np.unique(indices[1])]

def _form_log4_rotor_groups(rot_list, rotmat):

    qsize = int(len(rot_list) / 4)
    if qsize < 1:
        qsize = 1
    #import pdb
    #pdb.set_trace()

    g1 = []
    g2 = []
    g3 = []
    g4 = []

    try:
        g1 = rot_list[:qsize]
        g2 = rot_list[qsize:2*qsize]
        g3 = rot_list[2*qsize:3*qsize]
        g4 = rot_list[3*qsize:]
    except:
        pass

    # Find all places where matrix is nonnegative
    #indices = np.where(rotmat > -1)

    #submat = rotmat[indices[0][0]:indices[0][-1]+1,
    #                indices[1][0]:indices[1][-1]+1]

    #center_x = int(submat.shape[0] / 2)
    #center_y = int(submat.shape[1] / 2)

    #try:
    #    g1 = submat[0:center_x, 0:center_y]
    #    g2 = submat[center_x: , 0:center_y]
    #    g3 = submat[0:center_x, center_y: ]
    #    g4 = submat[center_x: , center_y: ]
    #except:
    #    raise Exception("Screwed up")


    return [g1, g2, g3, g4]

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

################## PROFILE BASED FUNCTIONS ######################

def find_suspects_by_profile(structure, residual_log, prof_file):
    struc_str = structure.gen_hashstring(en_fail_motor=False)
    try:
        with open(prof_file, "r") as f:
            prof = json.load(f)
    except:
        raise Exception("File read error for profiles: {}".format(prof_file))

    if struc_str not in prof:
        raise Exception("Structure not profiled: {}".format(struc_str))

    prof = prof[struc_str]

    # Extract phi dot and theta dot from residual of state vector
    log = [r[-3:-1] for r in residual_log]

    # Get a mean of the relevant entries from the residual log
    log = [entry for entry in log if entry[0]**2 + entry[1]**2 > 0.1]
    log = np.array(log)

    # Get the mean (median?) log entry
    residual_min = [round(np.min(log[:, 0]), 2), round(np.min(log[:, 1]), 2)]
    residual_max = [round(np.max(log[:, 0]), 2), round(np.max(log[:, 1]), 2)]
    residual_med = [round(np.median(log[:, 0]), 2), round(np.median(log[:, 1]), 2)]

    # Loop through the profiled rotors and see which ones match
    suspects = []
    for mod_id_str in prof:
        for rot_id_str in prof[mod_id_str]:
            record = prof[mod_id_str][rot_id_str]
            minrange = [round(r, 2) for r in record[0]]
            maxrange = [round(r, 2) for r in record[1]]


            # In addition to each residual entry between the profile and the
            # observed needing to be similar, we should observe the gap between
            # \dot{\theta} - \hat{\dot{\theta}} and 
            # \dot{\phi} - \hat{\dot{\phi}} is similar

            #gap_recorded = record[0] - record[1]
            #gap_observed = residual[0] - residual[1]

            #print("Check {}: {} | {}  || gap_rec = {:.02f}, gap_obs = {:.02f}".format(
            #        (mod_id_str, rot_id_str), record, residual,
            #        gap_recorded, gap_observed))
            # print("Check {}: [{} - {}] | {}, {}, {}".format(
            #         (mod_id_str, rot_id_str), minrange, maxrange, 
            #         residual_min, residual_med, residual_max))

            try:
                sus = (int(mod_id_str), int(rot_id_str))

                # Check over the range of residuals if any of them fall into the
                # right range to justify suspecting of this rotor
                if residual_min[0] >= minrange[0]-0.1 and residual_min[0] <= maxrange[0]+0.1:
                    if residual_min[1] >= minrange[1]-0.1 and residual_min[1] <= maxrange[1]+0.1:
                        suspects.append(sus)
                elif residual_max[0] >= minrange[0]-0.1 and residual_max[0] <= maxrange[0]+0.1:
                    if residual_max[1] >= minrange[1]-0.1 and residual_max[1] <= maxrange[1]+0.1:
                            suspects.append(sus)
                elif residual_med[0] >= minrange[0]-0.1 and residual_med[0] <= maxrange[0]+0.1:
                    if residual_med[1] >= minrange[1]-0.1 and residual_med[1] <= maxrange[1]+0.1:
                            suspects.append(sus)
            except TypeError as e:
                print(e)
                raise Exception("Profile file format might not be correct")

            # if abs(abs(residual[0]) - abs(record[0])) < 0.5:
            #     if abs(abs(residual[1]) - abs(record[1])) < 0.5:
            #         if abs(gap_recorded - gap_observed) < 0.1:
            #             suspects.append((int(mod_id_str), int(rot_id_str)))

    print("Profiling based suspects: {}".format(suspects))

    return suspects

def expand_from_epictr(structure, t, next_t, groups, qidx, rotmat, ramp_set):
    raise Exception("Not implemented yet")
