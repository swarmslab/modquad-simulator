import numpy as np

from compiled_scheduler.mqmod import mqmod
from compiled_scheduler.modset import modset

from modsim import params
from modsim.datatype.structure import Structure

from modquad_sched_interface.interface import convert_modset_to_struc

def lin_assign(mset, start_id=0):
        # Create variables
        S   = [i for i in range(mset.num_mod)] # Module positions
        print(S)

        try:
            locs = np.nonzero(mset.struc)
            for x in S:
                mset.assign_mod(start_id + mset.num_mod - x - 1, 
                                [locs[0][x], locs[1][x]]
                )
        except:
            raise Exception("Diff # Phys mods and mod positions")

