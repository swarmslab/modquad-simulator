from math import sqrt
import rospy
import numpy as np
import json

from modsim import params
from modsim.datatype.structure import Structure

from modquad_sched_interface.interface import rotpos_to_mat

def inject_faults(mq_struc, max_faults, mset, faulty_rots, fmod, frot):
    faulty_rots = []
    num_faults = 0
    while num_faults < max_faults:
        #newfault = (random.randint(0,mset.num_mod-1), random.randint(0,3))
        newfault = (int(fmod), int(frot))
        if newfault not in faulty_rots:
            faulty_rots.append(newfault)
            num_faults += 1	
        num_faults += 1

    print("Injecting faults: {}".format(faulty_rots))
    for f in faulty_rots:
        mq_struc.motor_failure.append(f)
    #faulty_rots = [(f[0]+1, f[1]) for f in faulty_rots]
    return [(f[0], f[1]) for f in faulty_rots]
