import itertools
import numpy as np
#from scheduler.scheduler.modset import modset
#from scheduler.scheduler.mqmod import mqmod
from compiled_scheduler.modset import modset
from compiled_scheduler.mqmod import mqmod
from modsim.datatype.structure import Structure
from modsim import params

def convert_modset_to_struc(mset, start_id=0):
    #ids = ['modquad{:02d}'.format(mq.mod_id + 1) for mq in mset.mods]
    #print(mset.pi + 1)
    #xpos = [params.cage_width * float(np.where(mset.pi == mq.mod_id)[1][0]) for mq in mset.mods]
    #ypos = [-params.cage_width * float(np.where(mset.pi == mq.mod_id)[0][0]) for mq in mset.mods]
    #fails = [(mq.mod_id + 1, rot_id) 
    #            for rot_id in range(4)
    #                for mq in mset.mods 
    #                    if not mq.gamma[rot_id]]
    ids = []
    xpos = []
    ypos = []
    fails = []
    i = 0
    for mid in range(start_id, start_id+mset.num_mod):#x,y in zip(*np.nonzero(mset.pi)):
        #print(mset.pi)
        #print(mid)
        loc = np.where(mset.pi == mid)
        x = loc[0][0]
        y = loc[1][0]
        ids.append('modquad{:02d}'.format(mid+1))
        #print("Modid {} is at pi[{}, {}]".format(mid+1, x, y))
        xpos.append(params.cage_width * y)
        ypos.append(params.cage_width * x)
        for rid in range(4):
            if not mset.mods[i].gamma[rid]:
                fails.append((mid+1, rid))
        i += 1
    s =  Structure(ids, xpos, ypos, fails)
    #print(ids)
    #print(xpos)
    #print(ypos)
    #print('/////\\\\\\')
    s.yy = -s.yy # Otherwise the alignment gets off due to matrix vs cartesian plane axis
    #print(s.ids)
    #print(s.xx)
    #print(s.yy)
    return s #Structure(ids, xpos, ypos, fails)

def convert_struc_to_mat(idset, xset, yset):
    """
    :param ids: modquad module ID #s involved in this structure
    :param xx: x pos of each module relative to struc center of mass
    :param yy: y pos of each module relative to struc center of mass
    :return struc: numpy matrix with each cell matching id of mod in it
    """
    #print('+++++++++')
    #print(xset)
    #print(yset)
    if type(idset[0]) != type(""):
        #int(ids[0]) # If not fail, passed as id 1
        ids = [i for i in idset]
    else:
        ids = [int(i[7:]) for i in idset] # Passed as id "modquad01"
    # Step 1: find out shape of structure using positions of mods
    xx = np.array(xset) / params.cage_width
    # matrix notation reverses where the zero-y is, so we multiply by -1
    yy = np.array(yset) / params.cage_width  * -1
    #xx.astype(int)
    #yy.astype(int)
    xx += -1 * np.min(xx)
    yy += -1 * np.min(yy)
    #print("____ Prerounding ____")
    #print(xx)
    #print(yy)
    xx = [int(round(i)) for i in list(xx)] # Switch to matrix notation 
    yy = [int(round(i)) for i in list(yy)] # From comp. screen notation
    minx = min(xx)
    maxx = max(xx)
    miny = min(yy)
    maxy = max(yy)
    struc = np.zeros((int(maxy - miny) + 1, int(maxx - minx) + 1)) - 1
    #print('+++')
    #print(ids)
    #print(xx)
    #print(yy)
    #print('************************************************')

    # NOTE: Switch myy into row and mxx into col to use matrix notation 
    #       instead of screen position notation
    for mid, mxx, myy in zip(ids, xx, yy):
        #print(mid, mxx, myy)
        struc[myy, mxx] = mid
    #print(struc)
    #print('___')
    #print('+++')
    return struc

def rotpos_to_mat(structure, rot_list):
    """
    rot_list and positions MUST have SAME indexing
    :param rot_list: list of tuples of elements (mod_id, rot_id)
    :return mat: matrix showing relative positions of rotors
    """
    # Get the structure as a matrix
    struc_mat = convert_struc_to_mat(structure.ids, structure.xx, structure.yy)
    struc_mat = np.array(struc_mat)

    mat = -1 * np.ones((2 * struc_mat.shape[0], 2 * struc_mat.shape[1]), dtype=np.int64)

    #mat = np.array( list(zip(mat.ravel(), mat.ravel())), 
    #                dtype=('i4,i4')).reshape(mat.shape)

    # Get minx, miny, maxx, maxy based on rot_list
    minx, miny, maxx, maxy = 99999, 99999, -99999, -99999
    for rot in rot_list:
        mat_idx = np.where(struc_mat == rot[0])
        assert (len(mat_idx[0]) == 1), "Mod ID has multiple mappings in X"
        assert (len(mat_idx[1]) == 1), "Mod ID has multiple mappings in Y"

        x = mat_idx[0][0]
        y = mat_idx[1][0]

        # Label each rotor position with its module ID
        if rot[1] == 0:
            mat[2*x + 1, 2*y    ] = rot[0]
        elif rot[1] == 1:
            mat[2*x + 1, 2*y + 1] = rot[0]
        elif rot[1] == 2:
            mat[2*x    , 2*y + 1] = rot[0]
        else: # rot[1] == 3
            mat[2*x    , 2*y    ] = rot[0]

    return mat
