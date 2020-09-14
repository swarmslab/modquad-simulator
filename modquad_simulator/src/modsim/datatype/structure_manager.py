#!/usr/bin/env python
# TODO not excessively use np.copy........

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from numpy import copy
import networkx as nx
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy

from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import circular_trajectory, simple_waypt_trajectory, \
    min_snap_trajectory

from modsim import params
from modsim.attitude import attitude_controller
# from modsim.plot.drawer_vispy import Drawer

from modsim.datatype.structure import Structure

from modsim.util.comm import publish_odom, publish_transform_stamped, \
        publish_odom_relative, publish_transform_stamped_relative, \
        publish_pos

from modsim.util.state import init_state, state_to_quadrotor
from modsim.util.undocking import gen_strucs_from_split, split_srv_input_format

from modquad_simulator.srv import Dislocation, DislocationResponse
from modquad_simulator.srv import SplitStructure, SplitStructureResponse

from modsim.simulation.ode_integrator import simulation_step

from dockmgr.datatype.ComponentManager import ComponentManager
from dockmgr.datatype.OdometryManager import OdometryManager

from modquad_sched_interface.interface import convert_modset_to_struc, convert_struc_to_mat
import modquad_sched_interface.waypt_gen as waypt_gen
"""
Uses ComponentManager to manage the structures that are available in
whatever the current simulation happens to be
"""

#thrust_newtons, roll, pitch, yaw = 0.0, 0.0, 0.0, 0.0

def publish_for_attached_mods(robot_id, structure_x, structure_y, xx, yy, main_id, odom_publishers, tf_broadcaster):
    publish_odom_relative(structure_x - xx[0], structure_y - yy[0], robot_id, main_id, odom_publishers[robot_id])
    publish_transform_stamped_relative(robot_id, main_id, structure_x - xx[0], structure_y - yy[0], tf_broadcaster)

def publish_structure_odometry(structure, odom_publishers, tf_broadcaster):
    ids, xx, yy, x = structure.ids, structure.xx, structure.yy, structure.state_vector

    main_id = ids[0]
    publish_odom(x, odom_publishers[main_id])
    publish_transform_stamped(main_id, x, tf_broadcaster)

    # show the other robots
    [publish_for_attached_mods(robot_id, structure_x, structure_y, xx, yy, 
        main_id, odom_publishers, tf_broadcaster) 
        for robot_id, structure_x, structure_y in list(zip(ids, xx, yy))[1:]]

def publish_structure_acceleration(structure, sdot, acc_publishers, tf_broadcaster):
    #ids, xx, yy, x = structure.ids, structure.xx, structure.yy, structure.state_vector

    #main_id = ids[0]
    #publish_acc(x, acc_publishers[main_id])
    #publish_transform_stamped(main_id, x, tf_broadcaster)

    ## show the other robots
    #[publish_for_attached_mods(robot_id, structure_x, structure_y, xx, yy, 
    #    main_id, odom_publishers, tf_broadcaster) 
    #    for robot_id, structure_x, structure_y in list(zip(ids, xx, yy))[1:]]
    return

def publish_mod_pos(structure, pos_publishers):
    ids, xx, yy, x = structure.ids, structure.xx, structure.yy, structure.state_vector

    main_id = ids[0]
    publish_pos(x, pos_publishers[main_id])

    # Find world pos relative to the main_id and publish
    # Only the position values are actually used
    # TODO make more efficient by using custom position message
    [publish_pos(
        [x[0] + xval-xx[0], x[1] + yval-yy[0], x[2], # World position
         0.0, 0.0, 0.0,      # Velocities
         0.0, 0.0, 0.0, 0.0, # Orientation
         0.0, 0.0, 0.0],     # Ang vel
        pos_publishers[idval])
        for idval, xval, yval in list(zip(ids, xx, yy))[1:]]

class StructureManager:
    def __init__(self, struclist):
        self.strucs = struclist # init based on those passed in
        self.demo_trajectory = rospy.get_param('~demo_trajectory', True)
        self.freq = rospy.get_param("freq", 100)
        self.state_vecs_log = [[] for _ in range(len(self.strucs))]
        self.desired_states_log = [[] for _ in range(len(self.strucs))]

    def control_step(self, t, trajectory_function, speed, 
            odom_publishers, pos_publishers, acc_publishers, tf_broadcaster):

        # NOTE: for some reason loop by value over a zip() does not work
        for i, structure in enumerate(self.strucs):

            # Publish odometry
            publish_structure_odometry(structure, odom_publishers, tf_broadcaster)

            # Publish world pos of each mod -- needed for docking detection
            publish_mod_pos(structure, pos_publishers)

            desired_state = trajectory_function(t, speed, structure.traj_vars)
            #if i == 0:# and t % 1.0 < 0.05:
            #    print("Desired state[{}] = {}".format(t, desired_state))
            #    print("Current state = {}".format(structure.state_vector))

            # Overwrite the control input with the demo trajectory
            [thrust_newtons, roll, pitch, yaw] = \
                    position_controller(structure, desired_state)

            try:
                self.desired_states_log[i].append(desired_state[0])
            except:
                # The joining of strucs cause the value of i to be out of range
                pass

            # Control output based on crazyflie input
            F_single, M_single = \
                    attitude_controller(structure, (thrust_newtons, roll, pitch, yaw))

            # Control of Moments and thrust
            F_structure, M_structure, rotor_forces = modquad_torque_control(
                            F_single, M_single, structure, motor_sat=False)

            # Simulate
            try:
                self.state_vecs_log[i].append(structure.state_vector[:3])
            except:
                # The joining of strucs cause the value of i to be out of range
                pass

            # sdot is the state vector derivative
            structure.state_vector, sdot = \
                            simulation_step(structure, structure.state_vector, 
                                            F_structure, M_structure, \
                                            1. / self.freq)

            # For IMU data, we need this information
            publish_structure_acceleration(structure, sdot, \
                                            acc_publishers, tf_broadcaster)
        
    def get_data_by_ind(self, index):
        if index < 0 or index > len(self.strucs):
            return None

        return self.strucs[index], \
               self.strucs[index].state_vector, \
               self.strucs[index].traj_vars

    def get_states(self): 
        """
        Returns list of locations of all structures 
        """
        return [structure.state_vector for structure in self.strucs]
    
    def split_struc(self, struc_to_replace, struclist):
        # Remove the structure that was split apart and its vars
        replace_ind = self.strucs.index(struc_to_replace)
        del self.strucs[replace_ind]
        org_des_stae = self.desired_states_log[replace_ind]
        org_stt_vecs = self.state_vecs_log[replace_ind]
        del self.desired_states_log[replace_ind]
        del self.state_vecs_log[replace_ind]

        # Add the new structures and assoc. vars to class instance vars
        self.strucs = self.strucs + struclist

        self.desired_states_log += [org_des_stae, copy.copy(org_des_stae)]
        self.state_vecs_log += [org_stt_vecs, copy.copy(org_stt_vecs)]

    def find_struc_of_mod(self, mod_id):
        mod_id = 'modquad{:02d}'.format(mod_id)
        struc = [s for s in self.strucs if mod_id in s.ids] # list of len 1
        return struc[0] # Return structure obj containing the module

    def join_strucs(self, struc1, struc2, ids_pair, direction, traj_func, t):
        """
        :param struc1: structure containing ids_pair[0]
        :param struc2: structure containing ids_pair[1]
        :param ids_pair: tuple of non-stringified mod ids specifying the join
        """
        dirs = {1: 'right', 2: 'up', 3: 'left', 4: 'down'}
        print("Joining the pair {}, ({} and {}) in adj dir {}".format(
            ids_pair, struc1.ids, struc2.ids, dirs[direction]))

        i = [j for j,v in enumerate(struc1.ids) 
                if v == 'modquad{:02d}'.format(ids_pair[0])]
        i = i[0] # Assuming that Structure is properly created, all ids are unique
        x1 = struc1.xx[i]
        y1 = struc1.yy[i]
        i = [j for j,v in enumerate(struc2.ids) 
                if v == 'modquad{:02d}'.format(ids_pair[1])]
        i = i[0] # Assuming that Structure is properly created, all ids are unique
        x2 = struc2.xx[i]
        y2 = struc2.yy[i]
        
        # Define all struc2 mods relative to x, y
        xx2 = np.copy(struc2.xx)
        yy2 = np.copy(struc2.yy)

        ## DO NOT DELETE - USEFUL DEBUGGING PRINTS
        # print("Struc1 State: {}".format(struc1.state_vector[:3]))
        # print("Struc2 State: {}".format(struc2.state_vector[:3]))
        # print("mods = {}".format(struc1.ids))
        # print("World Pos X: {}".format(struc1.xx + struc1.state_vector[0]))
        # print("World Pos Y: {}".format(struc1.yy + struc1.state_vector[1]))
        # print("mods = {}".format(struc2.ids))
        # print("World Pos X: {}".format(struc2.xx + struc2.state_vector[0]))
        # print("World Pos Y: {}".format(struc2.yy + struc2.state_vector[1]))
        # print("x1 = {}".format(x1))
        # print("y1 = {}".format(y1))
        # print("mods = {}".format(struc1.ids))
        # print("old xx1 = {}".format(struc1.xx))
        # print("old yy1 = {}".format(struc1.yy))
        # print('-')
        # print("mods = {}".format(struc2.ids))
        # print("old xx2 = {}".format(xx2))
        # print("old yy2 = {}".format(yy2))

        xdiff = x1 - x2
        ydiff = y1 - y2
        # print('xdiff = {}'.format(xdiff))
        # print('ydiff = {}'.format(ydiff))

        # Shift the struc2 coordinates to match the center of mass of struc1
        #       dirs = {1: 'right', 2: 'up', 3: 'left', 4: 'down'}
        if direction == 1:
            delta_x = (x1 - params.cage_width) - x2
            xx2 += delta_x #(x1 + params.cage_width + xdiff)
            yy2 += ydiff
        elif direction == 2:
            xx2 += xdiff
            delta_y = (y1 + params.cage_width) - y2
            yy2 += delta_y #(y1 + params.cage_width + ydiff)
            #print("new xx2 = {}".format(xx2))
            #print("new yy2 = {}".format(yy2))
        elif direction == 3:
            delta_x = (x1 - params.cage_width) - x2
            xx2 += delta_x #(x1 + params.cage_width - xdiff)
            yy2 += ydiff
        elif direction == 4:
            xx2 += xdiff
            delta_y = (y1 + params.cage_width) - y2
            yy2 += delta_y #(y1 + params.cage_width - ydiff)
        else:
            raise ValueError("Unknown direction of joining")

        xx = np.hstack((np.copy(struc1.xx), xx2))
        yy = np.hstack((np.copy(struc1.yy), yy2))
        mids = struc1.ids + struc2.ids
        fails = struc1.motor_failure + struc2.motor_failure

        newstruc = Structure(ids=mids, xx=xx, yy=yy, motor_failure=fails)

        # The index i corresponds to the second new adjacency module
        center_of_mass_shift_x = newstruc.xx[i] - x2
        center_of_mass_shift_y = newstruc.yy[i] - y2
        state_x = struc2.state_vector[0] + center_of_mass_shift_x
        state_y = struc2.state_vector[1] + center_of_mass_shift_y
        state_z = struc2.state_vector[2]

        newstruc.state_vector = init_state([state_x, state_y, state_z], 0.0)

        ###### TEMPORARY TRAJECTORY
        newstruc.traj_vars = traj_func(t, rospy.get_param("structure_speed", 0.5), None,
                waypt_gen.line(np.copy(newstruc.state_vector[:3]), np.copy(newstruc.state_vector[:3]+0.1)))

        print("New structure is the following: ")
        print(convert_struc_to_mat(newstruc.ids, newstruc.xx, newstruc.yy))

        # Delete the old structures
        _, _ = self.del_struc(struc1)
        old_des_states, old_actual_states = self.del_struc(struc2)

        # Add the new structure and assoc. vars to class instance vars
        self.add_struc(newstruc, old_des_states, old_actual_states)

    def del_struc(self, struc_to_delete):
        replace_ind = self.strucs.index(struc_to_delete)
        del self.strucs[replace_ind]
        org_des_stae = self.desired_states_log[replace_ind]
        org_stt_vecs = self.state_vecs_log[replace_ind]
        del self.desired_states_log[replace_ind]
        del self.state_vecs_log[replace_ind]
        return org_des_stae, org_stt_vecs

    def add_struc(self, struc_to_add, old_des_state, old_actual_state):
        self.strucs = self.strucs + [struc_to_add]
        self.desired_states_log += [old_des_state]
        self.state_vecs_log += [old_actual_state]

    def make_plots(self):
        # NOTE This doesn't work yet if you join structures
        # - Works if all you do is disassemble a structure or not (dis)assemble at all
        plt.style.use('dark_background')
        tstep = 1.0 / self.freq
        fig = plt.figure()
        plt.subplot(len(self.strucs), 1, 1)
        plt.title('X pos')
        for i,_ in enumerate(self.strucs):
            sveclog = np.array(self.state_vecs_log[i])
            dstatelog = np.array(self.desired_states_log[i])
            plt.subplot(len(self.strucs), 1, i+1)
            plt.plot(
                tstep*np.arange(0, dstatelog[:,0].shape[0]), dstatelog[:, 0], 'c')
            plt.plot(
                tstep*np.arange(0, sveclog[:,0].shape[0]), sveclog[:, 0], 'r')
        plt.xlabel('Time (sec)')
        fig2 = plt.figure()
        plt.subplot(len(self.strucs), 1, 1)
        plt.title('Y pos')
        for i,_ in enumerate(self.strucs):
            sveclog = np.array(self.state_vecs_log[i])
            dstatelog = np.array(self.desired_states_log[i])
            plt.subplot(len(self.strucs), 1, i+1)
            plt.plot(
                tstep*np.arange(0, dstatelog[:,1].shape[0]), dstatelog[:, 1], 'c')
            plt.plot(
                tstep*np.arange(0, sveclog[:,1].shape[0]), sveclog[:, 1], 'r')
        plt.xlabel('Time (sec)')
        fig3 = plt.figure()
        plt.subplot(len(self.strucs), 1, 1)
        plt.title('Z pos')
        for i,_ in enumerate(self.strucs):
            sveclog = np.array(self.state_vecs_log[i])
            dstatelog = np.array(self.desired_states_log[i])
            plt.subplot(len(self.strucs), 1, i+1)
            plt.plot(
                tstep*np.arange(0, dstatelog[:,2].shape[0]), dstatelog[:, 2], 'c')
            plt.plot(
                tstep*np.arange(0, sveclog[:,2].shape[0]), sveclog[:, 2], 'r')
        plt.xlabel('Time (sec)')
        plt.show()
