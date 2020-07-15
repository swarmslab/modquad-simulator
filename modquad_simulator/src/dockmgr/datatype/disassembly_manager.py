#!/usr/bin/env python
"""
This module takes a map of structures to disassemblies and executes the 
disassembly of a structure
"""
import rospy
import tf2_ros
import numpy as np
import networkx as nc
import copy

# Modquad modules
from modsim import params
from modsim.datatype.structure import Structure
from modsim.datatype.structure_manager import StructureManager
from modsim.util.undocking import gen_strucs_from_split, split_srv_input_format
from modsim.util.state import init_state
from modsim.trajectory import min_snap_trajectory
from modquad_simulator.srv import SplitStructure, SplitStructureResponse

# Scheduler modules
from scheduler.gsolver import gsolve

# Interface modules
from modquad_sched_interface.interface import convert_struc_to_mat
import modquad_sched_interface.waypt_gen as waypt_gen

class DisassemblyManager:
    """
    Given structure, disassembly map, and current time t
    Plan: At what time which disassemblies happen
          Waypoints for structures to go to to avoid collision
    """
    def __init__(self, reconf_map, start_time, structure, traj_func):
        self.start_time = start_time
        self.reconf_map = reconf_map
        rospy.set_param("opmode", "disassemble")
        self.mat = convert_struc_to_mat(structure.ids, structure.xx, structure.yy)
        self.next_disassemblies = {}
        self.reserve_time = 3.0
        self.time_for_disassembly = start_time + self.reserve_time # 1 seconds per layer
        self.trajectory_function = traj_func

    def plan_next_disassembly_set(self, struc_mgr, t):
        """
        Split all structures we currently have if they need to be split as per reconfigurability
        Reconfibgurability means can all modules be placed into the new desired location
        Note we currently do not fully do this, we simply follow the map
        """
        #print("===================================================")
        speed = rospy.get_param('structure_speed', 1.0)
        self.next_disassemblies = {}

        print(" ====== Reconf Map ====== ")
        for ind in self.reconf_map:
            print("{} : {}".format(ind, self.reconf_map[ind]))
        print(" --- Finding next disassemblies --- ")
        for s in struc_mgr.strucs:
            hashstring = s.gen_hashstring()
            #print("Looking for entry to disassemble: {}".format(hashstring))
            if hashstring in self.reconf_map:
                self.next_disassemblies[hashstring] = self.reconf_map[hashstring]
            #   print("\t{} disassembles at {}".format(hashstring, self.reconf_map[hashstring]))
            #else:
            #    print("\t{} does not have an entry".format(hashstring))

        if len(self.next_disassemblies) == 0:
            #speed = 0.75
            print("Disassembly complete, returning to normal op mode")
            rospy.set_param("opmode", "assemble")
            return ""

        # Plan locations for all structures + the two new ones
        new_locs = [ [ [0.0,0.0,0.0], [0.0,0.0,0.0] ] for _ in range(len(struc_mgr.strucs))]
        for struc, new_loc in zip(struc_mgr.strucs, new_locs):
            dis = struc.gen_hashstring()
            if dis not in self.next_disassemblies:
                continue
            cur_loc = struc.state_vector[:3]
            dis_loc = self.next_disassemblies[dis]
            cur_locp= [[], []]
            zero_loc = init_state([0.0, 0.0, 0.0], 0)

            # Based on direction of disassembly, generate the new trajectories where 
            # generated substructures will be positioned
            if dis_loc[0][0] == 'x':
                new_loc[0] = [np.copy(cur_loc[0]), 
                              np.copy(cur_loc[1]) + 0.5, 
                              np.copy(cur_loc[2]) + 0.5]
                new_loc[1] = [np.copy(cur_loc[0]), 
                              np.copy(cur_loc[1]) - 0.5, 
                              np.copy(cur_loc[2]) + 0.5]
                cur_locp[0]= [np.copy(cur_loc[0]), 
                              np.copy(cur_loc[1])+1*params.cage_width * dis_loc[0][1], 
                              np.copy(cur_loc[2])]
                cur_locp[1]= [np.copy(cur_loc[0]), 
                              np.copy(cur_loc[1])-1*params.cage_width * dis_loc[0][1], 
                              np.copy(cur_loc[2])]
            else: # y disassembly
                new_loc[0] = [np.copy(cur_loc[0]) - 0.5, 
                              np.copy(cur_loc[1]), 
                              np.copy(cur_loc[2]) + 0.5]
                new_loc[1] = [np.copy(cur_loc[0]) + 0.5, 
                              np.copy(cur_loc[1]), 
                              np.copy(cur_loc[2]) + 0.5]
                cur_locp[0]= [np.copy(cur_loc[0])-1*params.cage_width * dis_loc[0][1], 
                              np.copy(cur_loc[1]), 
                              np.copy(cur_loc[2])]
                cur_locp[1]= [np.copy(cur_loc[0])+1*params.cage_width * dis_loc[0][1], 
                              np.copy(cur_loc[1]), 
                              np.copy(cur_loc[2])]
            new_loc[0] = np.array(new_loc[0]).tolist()
            new_loc[1] = np.array(new_loc[1]).tolist()
            # Split the structure physically
            split = rospy.ServiceProxy("SplitStructure", SplitStructure)
            split_dim, breakline, split_ind = dis_loc[0][0], dis_loc[0][1], dis_loc[0][2], 
            inp = split_srv_input_format(struc, int(split_dim=='y'), breakline, split_ind)
            #inp[0] = [int(inp[0][idx]) for idx in range(inp[0])]
            #inp[5] = [int(inp[5][idx]) for idx in range(inp[5])]
            for i in range(7):
                print("inp[{}] = {}".format(i, inp[i]))
            ret = split(inp[0], inp[1], inp[2], inp[3], inp[4], inp[5], inp[6], inp[7])

            # Generate the new structures post-split (i.e. generate the actual objects)
            new_strucs = gen_strucs_from_split(ret)

            # Generate state_vecs offset from orig. center of mass based on the split
            new_strucs[0].state_vector = init_state(cur_locp[0], 0)
            new_strucs[1].state_vector = init_state(cur_locp[1], 0)

            # Generate state_vecs offset from orig. center of mass based on the split
            new_strucs[0].state_vector = init_state(cur_locp[0], 0)
            new_strucs[1].state_vector = init_state(cur_locp[1], 0)
            
            # Generate trajectories for new structures
            speed = 0.55
            new_strucs[0].traj_vars = self.trajectory_function(
                    t, speed, None, 
                    waypt_gen.line(np.copy(cur_locp[0]), np.copy(new_loc[0])))

            new_strucs[1].traj_vars = self.trajectory_function(
                    t, speed, None, 
                    waypt_gen.line(np.copy(cur_locp[1]), np.copy(new_loc[1])))

            for news in new_strucs:
                print("\tNew struc: {}".format(sorted(news.ids)))
                print("\t\tHashstring: {}".format(news.gen_hashstring()))

            print("Current loc: {}".format(struc.state_vector[:3]))
            for i, loc in enumerate(new_loc):
                print("\tNew struc loc: {}".format(new_strucs[i].state_vector[:3]))
                print("\tNew goal: {}".format(loc))

            #print("Waypts for 0th structure:\n{}".format(new_strucs[0].traj_vars.waypts))

            # Update the structure manager
            struc_mgr.split_struc(struc, new_strucs)

            #print(struc_mgr.strucs[0].traj_vars.waypts)

            # Print new strucs
            print("Strucs as they now stand:")
            for s in struc_mgr.strucs:
                print("\t{}\t{}".format(s.ids, s.gen_hashstring()))
            print('##############')
        return None

    def take_step(self, struc_mgr, t, ind):
        """
        Handles when to perform the next split
        """
        if t >= self.time_for_disassembly:
            ret = self.plan_next_disassembly_set(struc_mgr, t)
            self.time_for_disassembly += self.reserve_time
            print("Splits at t = {}, ind = {}".format(t, ind))
            return ret is None
        return False
