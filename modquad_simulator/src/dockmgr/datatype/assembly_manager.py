#!/usr/bin/env python
"""
This module takes a set of structures and a desired final structure 
and executes the assembly
"""
import rospy
import tf2_ros
import numpy as np
import networkx as nc
import copy
import sys
from itertools import combinations

# Modquad modules
from modsim import params
from modsim.datatype.structure import Structure
from modsim.datatype.structure_manager import StructureManager
from modsim.util.undocking import gen_strucs_from_split, split_srv_input_format
from modsim.trajectory import min_snap_trajectory
from dockmgr.datatype.WorldPosManager import WorldPosManager

# Scheduler modules
from scheduler.assembly import assemble

# Interface modules
from modquad_sched_interface.interface import convert_struc_to_mat
import modquad_sched_interface.waypt_gen as waypt_gen

dirs = {1: 'right', 2: 'up', 4: 'left', 3: 'down'}
berth = 1.00
offset = 1.00

def extract_mods_from_string(struc_mgr, hashstring, assembly_params, target):
    """
    :param hashstring: the mod_ids BEFORE '_' are struc1, AFTER '_' are struc2
    :param assembly_set: list of tuples of form ((dir, loc), layer)
    :param struc_mgr: the structure manager, which has current state of system
    :param target: The structure (as np.array) we want to assemble
    :return modid1, modid2, dir: modid1 in struc1, modid2 in struc2, dir of adjacency
    This function will find two module IDs along the border such that modid1 
        WILL BECOME adjacent to modid2 in the direction "dir" after docking
    """
    #plan_next_z_motion(self, t, struc_mgr, modid1, modid2, zlayer, traj_func):
    #plan_next_xy_motion(self, t, struc_mgr, modid1, modid2, adj_dir, traj_func):
    modid1, modid2, direction = -1, -1, -1

    struc_strings = hashstring.split('_')
    if len(struc_strings) != 2:
        raise ValueError("There is a problem in the hashstrings for assembly")
    struc1_str = struc_strings[0]
    struc2_str = struc_strings[1]
    struc1_ids = [int(mid) for mid in struc1_str.split(',')]
    struc2_ids = [int(mid) for mid in struc2_str.split(',')]
    
    # Find the structures containing these IDs
    # NOTE: We only need to use a single module to find the structure, as the others
    #       are already attached
    struc1 = struc_mgr.find_struc_of_mod(struc1_ids[0])
    struc2 = struc_mgr.find_struc_of_mod(struc2_ids[0])

    # Easier to work with the matrix representation
    struc1 = convert_struc_to_mat(struc1.ids, struc1.xx, struc1.yy)
    struc2 = convert_struc_to_mat(struc2.ids, struc2.xx, struc2.yy)
    #print(assembly_params)
    if assembly_params[0] == 0: # x assembly
        row1 = struc1[-1, :]
        xy1 = np.where(target == row1)
        row2 = struc2[0, :]
        xy2 = np.where(target == row2)

        print(target)
        print(struc1)
        print(struc2)
        print(row1)
        print(xy1)
        print(row2)
        print(xy2)
        # Now find a column in which two modules will be adjacent in target
        # Any two indices where the column val is the same are adjacent
        adj_locs = xy1[1] == xy2[1]
        ind = adj_locs.tolist().index(True)
        #print(ind)
        modid1 = target[xy1[0][ind], xy1[1][ind]]
        modid2 = target[xy2[0][ind], xy2[1][ind]]
        adj_dir = 'down'
    else: # y assembly
        col1 = struc1[:, -1:]
        xy1 = np.where(target == col1)
        col2 = struc2[:, 0:1]
        xy2 = np.where(target == col2)
        # Now find a row in which two modules will be adjacent in target
        # Any two indices where the row val is the same are adjacent
        print(target)
        print(struc1)
        print(struc2)
        print(col1)
        print(xy1)
        print(col2)
        print(xy2)
        adj_locs = xy1[0] == xy2[0]
        ind = adj_locs.tolist().index(True)
        if len(xy1[0]) < len(xy2[0]):
            ind1 = xy1.index(xy2[0][ind])
            ind2 = ind
        elif len(xy2[0]) < len(xy1[0]):
            ind1 = ind
            ind2 = xy2.index(xy1[0][ind])
        else:
            ind1 = ind
            ind2 = ind
        print(adj_locs)
        print(ind)
        print(ind1, ind2)
        modid1 = target[xy1[0][ind1], xy1[1][ind1]]
        modid2 = target[xy2[0][ind2], xy2[1][ind2]]
        adj_dir = 'right'
    #print(modid1, modid2, adj_dir)
    return int(modid1), int(modid2), adj_dir

class AssemblyManager:
    """
    Given structure to assemble, current structures, and current time t
    Plan: At what time which assemblies happen
          Waypoints for structures to go to to avoid collision
    """
    def __init__(self, start_time, final_struc_mat, traj_func):
        self.start_time = start_time
        rospy.set_param("opmode", "assemble")

        self.mat = np.array(final_struc_mat)

        # Find places where there is no module in structure, i.e., empty space
        # AssemblyManager fills these spaces with 0s instead of -1
        zerolocs = np.where(self.mat == -1)
        print(zerolocs)
        if len(zerolocs) > 0:
            if not zerolocs[0].shape == (0,):
                self.mat[zerolocs[0], zerolocs[1]] = 0

        self.reserve_time = 3.0
        self.time_for_assembly = start_time + self.reserve_time # seconds per layer
        self.trajectory_function = traj_func
        self.dockings = None
        self.n = 2 #rospy.get_param("num_used_robots", 5)
        self.pos_manager = WorldPosManager(self.n)
        self.pos_manager.subscribe()
        self.next_plan_z = True
        self.assembly_layer = 1 # Used to find which assemblies to do next
        self.assemblies = {} # Mapping of assemblies to perform
        self.next_time_to_plan = float("inf")
        self.next_assemblies = {} # the assemblies for a particular layer
        self.num_next_dockings_achieved = 0
        self.docked_pairs = []

    def plan_assemblies(self, struc_mgr):
        """
        Split all structures we currently have if they need to be split as per reconfigurability
        Reconfibgurability means can all modules be placed into the new desired location
        Note we currently do not fully do this, we simply follow the map
        """
        L = [convert_struc_to_mat(s.ids, s.xx, s.yy) for s in struc_mgr.strucs]
        A = {}
        num_layers, self.assemblies = assemble(A, L, self.mat)
        print("============= Assembly mappings ============")
        print("Target: \n{}".format(self.mat))
        print("Current mats")
        for ell in L: print(ell)
        print('---')
        for x in sorted(A):
            print("{} | {}".format(x, A[x]))
        print("============================================")

    def start_assembling_at_time(self, t):
        self.next_time_to_plan = t

    def take_step(self, struc_mgr, t):
        """
        Handles when to perform the next assembly
        Provide extra seconds per assembly
        """
        if (t >= self.next_time_to_plan and len(self.next_assemblies) == 0) and self.next_plan_z :
            if len(self.next_assemblies) > 0:
                print("Supposedly Finished layer {} of assembly".format(self.assembly_layer-1))
                if self.num_next_dockings_achieved != len(self.next_assemblies):
                    for i, mapping in enumerate(self.next_assemblies):
                        print("Redoing {}".format(mapping))
                        sys.exit(0)
                        modid1, modid2, adj_dir = extract_mods_from_string(struc_mgr, mapping[0], mapping[1][0], self.mat)
                        self.plan_corrective_motion(t, struc_mgr, modid1, modid2, adj_dir, self.trajectory_function)
                    return True
                print("Actually Finished layer {} of assembly".format(self.assembly_layer-1))
                sys.exit(0)
            # Get next pairs of structures to dock and plan the heights to dock them at
            self.next_assemblies = [(string, self.assemblies[string]) for string in self.assemblies if self.assemblies[string][1] == self.assembly_layer]
            if len(self.next_assemblies) == 0:
                if rospy.get_param("reset_docking") == 1:
                    self.dockings = None
                    print("Wait for docking reset")
                    return True
                print("Completed assembly")
                rospy.set_param("opmode", "normal")
                return False
            print("At t = {}, set up assemblies: {}".format(t, self.next_assemblies))

            # The z-locations at which the pairs will dock
            zlayers = [i+1.0 for i in range(len(self.next_assemblies))]
            
            for i, mapping in enumerate(self.next_assemblies):
                modid1, modid2, _ = extract_mods_from_string(struc_mgr, mapping[0], mapping[1][0], self.mat)
                self.plan_next_z_motion(t, struc_mgr, modid1, modid2, zlayers[i], self.trajectory_function)

            self.assembly_layer += 1
            #if self.assembly_layer == 4:
            #    rospy.set_param("print_pos_error", 1)
            return True

        elif t >= 15 + self.next_time_to_plan:
            #print("Finished ascending to the desired heights")
            for i, mapping in enumerate(self.next_assemblies):
                modid1, modid2, adj_dir = extract_mods_from_string(struc_mgr, mapping[0], mapping[1][0], self.mat)
                self.plan_next_xy_motion(t, struc_mgr, modid1, modid2, adj_dir, self.trajectory_function)
            return True
        return False

    def plan_corrective_motion(self, t, struc_mgr, modid1, modid2, adj_dir, traj_func):
        global berth, offset
        speed = rospy.get_param("structure_speed", 0.5)
        struc1 = struc_mgr.find_struc_of_mod(modid1)
        struc2 = struc_mgr.find_struc_of_mod(modid2)
        if struc1.ids == struc2.ids:
            return

        # Want to be on same level as struc1, even if it moved a bit
        zpos = struc1.state_vector[2]

        # We need to check that the structures are oriented relative 
        #  to each other correctly, and if not move them
        i1  = struc1.ids.index('modquad{:02d}'.format(modid1))
        i2  = struc2.ids.index('modquad{:02d}'.format(modid2))
        x1  = struc1.xx[i1]
        y1  = struc1.yy[i1]
        x2  = struc2.xx[i2]
        y2  = struc2.yy[i2]
        s1x = struc1.state_vector[0]
        s1y = struc1.state_vector[1]
        s2x = struc2.state_vector[0]
        s2y = struc2.state_vector[1]

        # World frame pos of modid1
        x1w = x1 + s1x
        y1w = y1 + s1y

        # Where struc2 should end up relative to struc1
        desire_x = 0.0 
        desire_y = 0.0

        print("Trying to connect {}-{} in dir {}".format(modid1, modid2, adj_dir))

        # Compute whether oriented and what desired x,y for struc2 are
        if adj_dir == 'up':
            desire_x = x1w
            desire_y = y1w + params.cage_width
        elif adj_dir == 'down':
            desire_x = x1w
            desire_y = y1w - params.cage_width
        elif adj_dir == 'left':
            desire_x = x1w - params.cage_width
            desire_y = y1w
        else: #adj_dir == 'right'
            desire_x = x1w - params.cage_width
            desire_y = y1w

        # Get desired pos of center of mass of struc 2 
        desire_x -= x2
        desire_y -= y2
        desire_pos = np.array([desire_x, desire_y, zpos])

        # Current state
        curstate = struc2.state_vector[:3]

        # Start new trajectory where the structures currently are
        waypts2 = [curstate]

        if adj_dir == 'up':
            waypts2.append([desire_x, desire_y + berth*0.30, zpos+0.25])
        elif adj_dir == 'down':
            waypts2.append([desire_x, desire_y - berth*0.30, zpos+0.25])
        elif adj_dir == 'left':
            waypts2.append([desire_x - berth*0.30, desire_y, zpos+0.25])
        else:
            waypts2.append([desire_x + berth*0.30, desire_y, zpos+0.25])

        # Finally, we want them to come together and attach
        # struc1 stays stationary, struc 2 moves in
        waypts2.append(desire_pos)

        struc2.traj_vars = traj_func(t, speed, None, np.array(waypts2))
        #print("Corrective trajectory: \n{}".format(struc2.traj_vars.waypts))
        print("\tPlanned corrective trajectory")
        if struc2.traj_vars.times[-1] > self.next_time_to_plan:
            self.next_time_to_plan = struc2.traj_vars.times[-1]

    def plan_next_xy_motion(self, t, struc_mgr, modid1, modid2, adj_dir, traj_func):
        global berth, offset
        speed = 0.25 #rospy.get_param("structure_speed", 0.3)
        struc1 = struc_mgr.find_struc_of_mod(modid1)
        struc2 = struc_mgr.find_struc_of_mod(modid2)

        locations = self.pos_manager.get_locations()
        if None in locations:
            rospy.logwarn('Assembly manager xy motion planner does not have all locs: {}'\
                    .format(locations))

        # Temp change: larger struc is the one to move
        # Having smaller structure be the one to move
        smaller_struc = 2
        if len(struc1.xx) < len(struc2.xx):
            smaller_struc = 1
            if adj_dir == 'down':
                adj_dir = 'up'
            elif adj_dir == 'up':
                adj_dir = 'down'
            elif adj_dir == 'right':
                adj_dir = 'left'
            else: # adj_dir == 'left'
                adj_dir = 'right'
           
        print("Smaller structure is struc{}".format(smaller_struc))

        # Start new trajectory where the structures currently are
        if smaller_struc == 2:
            waypts2 = [np.copy(struc2.state_vector[:3])]
        else:
            waypts1 = [np.copy(struc1.state_vector[:3])]

        # Want to be on same level as struc1, even if it moved a bit
        if smaller_struc == 2:
            zpos = struc1.traj_vars.waypts[-1,:][2]
        else:
            zpos = struc2.traj_vars.waypts[-1,:][2]

        # We need to check that the structures are oriented relative 
        #  to each other correctly, and if not move them
        i1  = struc1.ids.index('modquad{:02d}'.format(modid1))
        i2  = struc2.ids.index('modquad{:02d}'.format(modid2))
        x1  = struc1.xx[i1]
        y1  = struc1.yy[i1]
        x2  = struc2.xx[i2]
        y2  = struc2.yy[i2]
        if smaller_struc == 2:
            s2x = struc2.state_vector[0]
            s2y = struc2.state_vector[1]
            s1x = struc1.state_vector[0] #struc1.traj_vars.waypts[-1,:][0]
            s1y = struc1.state_vector[1] #struc1.traj_vars.waypts[-1,:][1]
            #s1x = struc1.traj_vars.waypts[-1,:][0]
            #s1y = struc1.traj_vars.waypts[-1,:][1]
        else:
            s1x = struc1.state_vector[0]
            s1y = struc1.state_vector[1]
            s2x = struc2.state_vector[0] #struc2.traj_vars.waypts[-1,:][0]
            s2y = struc2.state_vector[1] #struc2.traj_vars.waypts[-1,:][1]
            #s2x = struc2.traj_vars.waypts[-1,:][0]
            #s2y = struc2.traj_vars.waypts[-1,:][1]
            #s1x = struc1.state_vector[0]
            #s1y = struc1.state_vector[1]

        # World frame pos of modid1
        print("Locations: ")
        print(locations)
        print(modid1)
        print(modid2)
        x1w = locations[modid1-1][0] #x1 + s1x
        y1w = locations[modid1-1][1] #y1 + s1y
        x2w = locations[modid2-1][0] #x2 + s2x
        y2w = locations[modid2-1][1] #y2 + s2y

        # Where struc2 should end up relative to struc1
        # or vice versa, depending on which struc is smaller
        desire_x = 0.0 
        desire_y = 0.0

        # Whether strucs are facing right way for linear trajs to connect them
        oriented = False
        print("\n=========================================================\n")
        print("Trying to connect {}-{} in dir {}".format(modid1, modid2, adj_dir))
        print("Struc Internal Pos of Mod {}: ({}, {})".format(modid1, x1, y1))
        print("Struc Internal Pos of Mod {}: ({}, {})".format(modid2, x2, y2))
        print("World Pos of Mod {}: ({}, {})".format(modid1, x1w, y1w))
        print("World Pos of Mod {}: ({}, {})".format(modid2, x2w, y2w))

        # Find the world pos of mods of each struc
        struc1_ids = [int(x[7:])-1 for x in struc1.ids]
        struc2_ids = [int(x[7:])-1 for x in struc2.ids]
        struc1_wpos_x = [locations[i][0] for i in struc1_ids]
        struc2_wpos_x = [locations[i][0] for i in struc2_ids]
        struc1_wpos_y = [locations[i][1] for i in struc1_ids]
        struc2_wpos_y = [locations[i][1] for i in struc2_ids]
        struc1_wpos_z = [locations[i][2] for i in struc1_ids]
        struc2_wpos_z = [locations[i][2] for i in struc2_ids]

        struc1_cmass = [np.mean(struc1_wpos_x), 
                        np.mean(struc1_wpos_y), 
                        locations[modid1-1][2]]
        struc2_cmass = [np.mean(struc2_wpos_x), 
                        np.mean(struc2_wpos_y), 
                        locations[modid2-1][2]]

        print("Struc 1 Mod X Pos: {}".format(struc1_wpos_x))
        print("Struc 1 Mod Y Pos: {}".format(struc1_wpos_y))
        print("Struc 1 CMass    : {}".format(struc1_cmass))
        print("Struc 2 Mod X Pos: {}".format(struc2_wpos_x))
        print("Struc 2 Mod Y Pos: {}".format(struc2_wpos_y))
        print("Struc 2 CMass    : {}".format(struc2_cmass))
        x1diff = x1w - struc1_wpos_x[0]
        y1diff = y1w - struc1_wpos_y[0]
        x2diff = x2w - struc2_wpos_x[0]
        y2diff = y2w - struc2_wpos_y[0]
        # Compute whether oriented and what desired x,y for struc2 are
        if smaller_struc == 2:
            if adj_dir == 'up':
                oriented = y1w < y2w# and abs(x2 - x1) > 0.1
                desire_x = x1w - x2diff
                    #struc1_wpos_x[0] - (x2w - struc2_wpos_x[0])
                    #np.mean(struc1_wpos_x) #struc1.state_vector[0] #x1w
                desire_y = y1w + params.cage_width - y2
            elif adj_dir == 'down':
                oriented = y1w > y2w# and abs(x2 - x1) > 0.1
                desire_x = x1w - x2diff
                    #struc1_wpos_x[0] - (x2w - struc2_wpos_x[0])
                    #np.mean(struc1_wpos_x) #struc1.state_vector[0] #x1w
                desire_y = y1w - params.cage_width - y2
            elif adj_dir == 'left':
                oriented = x1w > x2w# and abs(y2 - y1) > 0.1
                desire_x = x1w - params.cage_width - x2
                desire_y = y1w - y2diff
                    #struc1_wpos_y[0] - (y2w - struc2_wpos_y[0])
                    #np.mean(struc1_wpos_y) #struc1.state_vector[1] #y1w
            else: #adj_dir == 'right'
                oriented = x1w < x2w# and abs(y2 - y1) > 0.1
                desire_x = x1w + params.cage_width - x2
                desire_y = y1w - y2diff
                    #struc1_wpos_y[0] - (y2w - struc2_wpos_y[0])
                    #np.mean(struc1_wpos_y) #struc1.state_vector[1] #y1w
        else:
            if adj_dir == 'up':
                oriented = y2w < y1w# and abs(x2 - x1) > 0.1
                desire_x = x2w - x1diff
                    #struc2_wpos_x[0] - (x1w - struc1_wpos_x[0])
                    #np.mean(struc2_wpos_x) #struc2.state_vector[0] #x2w
                desire_y = y2w + params.cage_width - y1
            elif adj_dir == 'down':
                oriented = y2w > y1w# and abs(x2 - x1) > 0.1
                desire_x = x2w - x1diff
                    #struc2_wpos_x[0] - (x1w - struc1_wpos_x[0])
                    #np.mean(struc2_wpos_x) #struc2.state_vector[0] #x2w
                desire_y = y2w - params.cage_width - y1
            elif adj_dir == 'left':
                oriented = x2w > x1w# and abs(y2 - y1) > 0.1
                desire_x = x2w - params.cage_width - x1
                desire_y = y2w - y1diff
                    #struc2_wpos_y[0] - (y1w - struc1_wpos_y[0])
                    #np.mean(struc2_wpos_y) #struc2.state_vector[1] #y2w
            else: #adj_dir == 'right'
                oriented = x2w < x1w# and abs(y2 - y1) > 0.1
                desire_x = x2w + params.cage_width - x1
                desire_y = y2w - y1diff
                    #struc2_wpos_y[0] - (y1w - struc1_wpos_y[0])
                    #np.mean(struc2_wpos_y) #struc2.state_vector[1] #y2w

        # Get desired pos of center of mass of struc 2 
        #if smaller_struc == 2:
        #    desire_x -= x1
        #    desire_y -= y1
        #else:
        #    desire_x -= x2
        #    desire_y -= y2

        desire_pos = np.array([desire_x, desire_y, zpos])

        # Current state
        if smaller_struc == 2:
            curstate = struc2.state_vector[:3]
        else:
            curstate = struc1.state_vector[:3]

        # Center about the mod pos to attach to
        if smaller_struc == 2:
            center = [s1x, s1y, zpos]
        else:
            center = [s2x, s2y, zpos]

        # Generate the four possible approach points
        up_pt = [desire_x, desire_y + berth, zpos]
        down_pt = [desire_x, desire_y - berth, zpos]
        left_pt = [desire_x - berth, desire_y, zpos]
        right_pt = [desire_x + berth, desire_y, zpos]

        # Generate approach point
        approach_point = []
        if adj_dir == 'up':
            approach_pt = up_pt
        elif adj_dir == 'down':
            approach_pt = down_pt
        elif adj_dir == 'left':
            approach_pt = left_pt
        elif adj_dir == 'right':
            approach_pt = right_pt

        # 4x3 matrix
        approaches = np.array([up_pt, left_pt, down_pt, right_pt])
        print("Approach Points to Desired: {}".format(desire_pos))
        print("Up:    {}".format(up_pt))
        print("Left:  {}".format(left_pt))
        print("Down:  {}".format(down_pt))
        print("Right: {}".format(right_pt))

        # Find the closest approach point to this module
        dist_to_approach_pt = np.sqrt( np.sum( (approaches - curstate) ** 2, axis=1) )
        print("\nDistances from current loc {} to approach points".format(curstate))
        print(dist_to_approach_pt)
        closest_circle_pt_idx = np.argmin(dist_to_approach_pt)
        print("Argmin for closest approach point: {}".format(closest_circle_pt_idx))
        closest_circle_pt = approaches[closest_circle_pt_idx, :]

        if smaller_struc == 2:
            waypts2.append(closest_circle_pt.tolist())
        else:
            waypts1.append(closest_circle_pt.tolist())

        # Next, figure out relative direction to move
        # If the approach point is on opposite side, use intermediate waypt
        # NOTE: The 1.5x factor is because berth = 1 meter
        new_dist_to_approach = np.sqrt( np.sum( (approach_pt - closest_circle_pt) ** 2) ) 
        print("\nApproach point ({}) = {}".format(adj_dir, approach_pt))
        print("new_dist_to_approach = {}, 1.5 * berth = {}, 0.25 * berth = {}".format(
            new_dist_to_approach, 1.5 * berth, 0.25 * berth))
        if new_dist_to_approach > 1.5 * berth:
            # The approach is on the opposite side
            print("Approach from opposite side")
            if smaller_struc == 2:
                waypts2.append(
                        np.roll(approaches, 1, axis=0)[closest_circle_pt_idx, :].tolist())
                waypts2.append(approach_pt)
            else:
                waypts1.append(
                        np.roll(approaches, 1, axis=0)[closest_circle_pt_idx, :].tolist())
                waypts1.append(approach_pt)
        elif new_dist_to_approach > berth * 0.25:
            print("Migrate to approach point")
            # The approach is not from here, but not on opposite side either
            if smaller_struc == 2:
                waypts2.append(approach_pt)
            else:
                waypts1.append(approach_pt)
        else: 
            print("Already at approach pt")
        # else: closest_circle_pt is the approach_pt

        #print('oriented: {}'.format(oriented))
        #if not oriented:
        #    if adj_dir == 'up':
        #        waypts2.append([curstate[0]+offset, curstate[1]+berth, zpos])
        #    elif adj_dir == 'down':
        #        waypts2.append([curstate[0]+offset, curstate[1]-berth, zpos])
        #    elif adj_dir == 'left':
        #        waypts2.append([curstate[0], curstate[1]-berth, zpos])
        #    elif adj_dir == 'right':
        #        waypts2.append([curstate[0], curstate[1]+berth, zpos])
        #else:
        #    offset = 0.25
        #    if adj_dir == 'up':
        #        waypts2.append([curstate[0]+offset, curstate[1], zpos])
        #    elif adj_dir == 'down':
        #        waypts2.append([curstate[0]+offset, curstate[1], zpos])
        #    elif adj_dir == 'left':
        #        waypts2.append([curstate[0], curstate[1]+offset, zpos])
        #    elif adj_dir == 'right':
        #        waypts2.append([curstate[0], curstate[1]+offset, zpos])

        if adj_dir == 'up':
            #waypts2.append([desire_x, desire_y + berth    , zpos])
            #waypts2.append([desire_x, desire_y + berth*0.75, zpos])
            if smaller_struc == 2:
                waypts2.append([desire_x, desire_y + berth*0.55, zpos])
                waypts2.append([desire_x, desire_y + berth*0.35, zpos])
            else:
                waypts1.append([desire_x, desire_y + berth*0.55, zpos])
                waypts1.append([desire_x, desire_y + berth*0.35, zpos])
        elif adj_dir == 'down':
            #waypts2.append([desire_x, desire_y - berth    , zpos])
            #waypts2.append([desire_x, desire_y - berth*0.75, zpos])
            if smaller_struc == 2:
                waypts2.append([desire_x, desire_y - berth*0.55, zpos])
                waypts2.append([desire_x, desire_y - berth*0.35, zpos])
            else:
                waypts1.append([desire_x, desire_y - berth*0.55, zpos])
                waypts1.append([desire_x, desire_y - berth*0.35, zpos])
        elif adj_dir == 'left':
            #waypts2.append([desire_x - berth    , desire_y, zpos])
            #waypts2.append([desire_x - berth*0.75, desire_y, zpos])
            if smaller_struc == 2:
                waypts2.append([desire_x - berth*0.55, desire_y, zpos])
                waypts2.append([desire_x - berth*0.35, desire_y, zpos])
            else:
                waypts1.append([desire_x - berth*0.55, desire_y, zpos])
                waypts1.append([desire_x - berth*0.35, desire_y, zpos])
        else:
            #waypts2.append([desire_x + berth    , desire_y, zpos])
            #waypts2.append([desire_x + berth*0.75, desire_y, zpos])
            if smaller_struc == 2:
                waypts2.append([desire_x + berth*0.50, desire_y, zpos])
                waypts2.append([desire_x + berth*0.35, desire_y, zpos])
            else:
                waypts1.append([desire_x + berth*0.50, desire_y, zpos])
                waypts1.append([desire_x + berth*0.35, desire_y, zpos])

        # Finally, we want them to come together and attach
        # struc1 stays stationary, struc 2 moves in
        if smaller_struc == 2:
            waypts2.append(desire_pos)
        else:
            waypts1.append(desire_pos)

        print("struc1 pos: {}".format(struc1.state_vector[:3]))
        print('-------------')
        print("struc2 pos: {}".format(struc2.state_vector[:3]))
        print('-------------')
        if smaller_struc == 2:
            print("struc2 desire pos: {}".format(desire_pos))
            print('-------------')
            print("2nd set of waypts: \n{}".format(np.array(waypts2)))
        else:
            print("struc1 desire pos: {}".format(desire_pos))
            print('-------------')
            print("1st set of waypts: \n{}".format(np.array(waypts1)))

        if smaller_struc == 2:
            struc2.traj_vars = traj_func(t, speed, None, np.array(waypts2))
            print('-------------')
            print(struc2.traj_vars.times)
            print("{}".format(struc2.traj_vars.waypts))
            print('struc 1 time/path')
            print(struc1.traj_vars.times)
            print("{}".format(struc1.traj_vars.waypts))
            print('struc 2 time/path')
            print(struc2.traj_vars.times)
            print("{}".format(struc2.traj_vars.waypts))
        else:
            struc1.traj_vars = traj_func(t, speed, None, np.array(waypts1))
            print('-------------')
            print('struc 1 time/path')
            print(struc1.traj_vars.times)
            print("{}".format(struc1.traj_vars.waypts))
            print('struc 2 time/path')
            print(struc2.traj_vars.times)
            print("{}".format(struc2.traj_vars.waypts))

        next_time_to_plan = 15+max([struc1.traj_vars.times[-1], struc2.traj_vars.times[-1]])
        if next_time_to_plan > self.next_time_to_plan:
            self.next_time_to_plan = next_time_to_plan
        print("Planned xy, t = {}, projected finish t = {}".format(t, self.next_time_to_plan))

        # Next time we are ready, plan the z motion for the next layer
        self.next_plan_z = True

    def plan_next_z_motion(self, t, struc_mgr, modid1, modid2, zlayer, traj_func):
        speed = 0.75#rospy.get_param("structure_speed", 1.0)
        struc1 = struc_mgr.find_struc_of_mod(modid1)
        struc2 = struc_mgr.find_struc_of_mod(modid2)

        # Start new trajectory where the structures currently are
        waypts1 = [np.copy(struc1.state_vector[:3])]
        waypts2 = [np.copy(struc2.state_vector[:3])]

        # Move them to be at the same z height
        nextpt1 = np.copy(struc1.state_vector[:3]) + np.array([0.1, 0.1, 0.0])
        nextpt1[2] = zlayer
        waypts1.append(nextpt1)
        nextpt2 = np.copy(struc2.state_vector[:3]) +  np.array([0.1, 0.1, 0.0])
        nextpt2[2] = zlayer
        waypts2.append(nextpt2)

        struc1.traj_vars = traj_func(t, speed, None, np.array(waypts1))
        struc2.traj_vars = traj_func(t, speed, None, np.array(waypts2))

        next_time_to_plan = 15+max([struc1.traj_vars.times[-1], struc2.traj_vars.times[-1]])
        if next_time_to_plan > self.next_time_to_plan:
            self.next_time_to_plan = next_time_to_plan

        # At next time to plan, plan xy motion
        self.next_plan_z = False
        #print(struc1.traj_vars.times)
        #print(struc2.traj_vars.times)
        print("Finished planning z for height {}".format(zlayer))
        print("t = {}, projected finish t = {}".format(t, self.next_time_to_plan))
        print(struc1.traj_vars.waypts)
        print(struc2.traj_vars.waypts)

    def handle_dockings_msg(self, struc_mgr, msg, traj_func, t, start_id=1):
        global dirs
        np.set_printoptions(precision=3)
        if self.dockings is None:
            rospy.loginfo("Init dockings to msg: {}".format(msg))
            self.dockings = np.array(msg.data)
            return
        dockings = np.array(msg.data) - self.dockings
        if np.sum(dockings) == 0:
            #rospy.loginfo("No dockings reported")
            return
        else:
            #print("HANDLING A DOCKING MSG: {}".format(msg))
            pairs = list(combinations(range(start_id,self.n+start_id), 2))
            dock_ind = np.nonzero(dockings)
            self.dockings = dockings # Contains both new and old dockings
            for x in dock_ind[0]:
                if ( pairs[x] in self.docked_pairs or 
                    (pairs[x][1], pairs[x][0]) in self.docked_pairs):
                    # Only handle new dockings
                    continue
                else:
                    self.docked_pairs.append(pairs[x])
                print('-----')
                print(msg)
                print("dockings = {}".format(dockings))
                print("saved dockings = {}".format(self.dockings))
                print("new dock ind = {}".format(dock_ind))
                print("pairs = {}".format(pairs))
                print("new docking of strucs with mods: {}".format(pairs[x]))
                # Find containing structures of these modules
                p1 = struc_mgr.find_struc_of_mod(pairs[x][0])
                p2 = struc_mgr.find_struc_of_mod(pairs[x][1])
                print("States of the now-docked structures")
                print(convert_struc_to_mat(p1.ids, p1.xx, p1.yy))
                print("\t{}".format(p1.state_vector[:3]))
                print(convert_struc_to_mat(p2.ids, p2.xx, p2.yy))
                print("\t{}".format(p2.state_vector[:3]))
                print("dock struc with ids: {}".format(p1.ids))
                print("to dock with ids: {}".format(p2.ids))
                if p1.ids == p2.ids:
                    continue # Already joined the relevant structures

                # We only recognize two directions, so adapt
                if dockings[x] == 2 or dockings[x] == 1: # up or left
                    p3 = p1
                    p1 = p2
                    p2 = p3
                    pairs[x] = (pairs[x][1], pairs[x][0])

                p1m = convert_struc_to_mat(p1.ids, p1.xx, p1.yy)
                if len(np.nonzero(p1)) > 1:
                    p1_nums = [p1m[i,j] for i,j in zip(*np.nonzero(p1m))]
                    p1str = ','.join(str(int(num)) for num in sorted(p1_nums))
                else:
                    p1_nums = [p1m[i] for i in zip(*np.nonzero(p1m))]
                    p1str = ','.join(str(int(num)) for num in sorted(p1_nums))
                
                p2m = convert_struc_to_mat(p2.ids, p2.xx, p2.yy)
                if len(np.nonzero(p2)) > 1:
                    p2_nums = [p2m[i,j] for x,y in zip(*np.nonzero(p2m))]
                    p2str = ','.join(str(int(num)) for num in sorted(p2_nums))
                else:
                    p2_nums = [p2m[i] for i in zip(*np.nonzero(p2m))]
                    p2str = ','.join(str(int(num)) for num in sorted(p2_nums))
                

                #if (dirs[dockings[x]] == 'up' or dirs[dockings[x]] == 'left'): 
                hashstring = '_'.join([p2str, p1str])
                #else:
                #    hashstring = '_'.join([p1str, p2str])

                # Updates
                print('- - - - - Triggering a join - - - - - - ')
                print(x)
                print(dock_ind)
                print(p1.ids, p2.ids)
                print("Docking direction detected at t = {}: {}".format(t, dockings[x]))
                struc_mgr.join_strucs(p1, p2, pairs[x], dockings[x], traj_func, t)
                todel = [i for i,to in enumerate(self.next_assemblies) if to[0] == hashstring]
                print("Generated hashstring: {}".format(hashstring))
                print("Deleting these assemblies because complete: {}".format(todel))
                for i in todel:
                    del self.next_assemblies[i]
                #self.num_next_dockings_achieved += 1

