#!/usr/bin/env python
"""
This simulation only tests undocking.
You can change the structure type at the bottom, and nominally pass in
a trajectory. 

setup_and_test() will introduce a fault, find the disassembly path, and
disassemble the structure accordingly. This will not reassemble
"""
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8MultiArray

import numpy as np
from numpy import copy
import networkx as nx
import sys
import copy

from modsim.trajectory import circular_trajectory, simple_waypt_trajectory, min_snap_trajectory

from modsim import params

from modsim.datatype.structure import Structure
from modsim.datatype.structure_manager import StructureManager
from dockmgr.datatype.disassembly_manager import DisassemblyManager
from dockmgr.datatype.assembly_manager import AssemblyManager

from modsim.util.comm import publish_odom, publish_transform_stamped, publish_odom_relative, \
    publish_transform_stamped_relative
from modsim.util.state import init_state, state_to_quadrotor
from modsim.util.undocking import gen_strucs_from_split, split_srv_input_format
from modquad_simulator.srv import Dislocation, DislocationResponse, SplitStructure, SplitStructureResponse
from modsim.simulation.ode_integrator import simulation_step

from modquad_sched_interface.interface import convert_modset_to_struc, convert_struc_to_mat
import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

from scheduler.gsolver import gsolve
from scheduler.reconfigure import reconfigure

# Structure Manager
struc_mgr = None
assembler = None
t = 0.0
traj_func = min_snap_trajectory

def docking_callback(msg):
    global assembler, struc_mgr, traj_func, t
    if assembler is not None:
        assembler.handle_dockings_msg(struc_mgr, msg, traj_func, t)
    else:
        raise ValueError("Assembler object does not exist")

def simulate(oldstruc, newstruc, reconf_map, t_step=0.01, speed=0.5, loc=[1., .0, .0], 
        waypts=None, figind=1, filesuffix="", split_dim=0, breakline=1, split_ind=0):
    global struc_mgr, assembler, t, traj_func
    trajectory_function = traj_func
    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]

    init_x = rospy.get_param('~init_x', 0.)
    init_y = rospy.get_param('~init_y', 0.)
    init_z = rospy.get_param('~init_z', 0.)
    demo_trajectory = rospy.get_param('~demo_trajectory', True)

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use

    # Location of current structure
    loc = [init_x, init_y, init_z]
    state_vector = init_state(loc, 0)

    # Make a copy of the state vector for each structure we have
    oldstruc.state_vector = state_vector

    # Init structure manager
    struc_mgr = StructureManager([oldstruc])

    # Set up topics
    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'
    pos_topic = rospy.get_param('world_pos_topic', '/world_pos')  

    # Subscribe to /dockings so that you can tell when to combine structures
    rospy.Subscriber('/dockings', Int8MultiArray, docking_callback) 

    # Odom publisher
    odom_publishers = {id_robot: 
        rospy.Publisher('/' + id_robot + odom_topic, Odometry, queue_size=0) 
        for id_robot in oldstruc.ids}

    pos_publishers = {id_robot: 
        rospy.Publisher('/' + id_robot + pos_topic, Odometry, queue_size=0) 
        for struc in struc_mgr.strucs for id_robot in oldstruc.ids}

    # TF publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    ########### Simulator ##############
    # Time based on avg desired speed (actual speed *not* constant)
    tmax = oldstruc.traj_vars.total_dist / speed
    overtime = 1.5

    # Params
    undocked = False
    freq = 100   # 100hz
    rate = rospy.Rate(freq)
    t = 0
    rospy.set_param("freq", freq)

    # Don't start with a disassembler object
    disassembler = None

    # After disassembly we need to use the assembler
    newstruc_mat = convert_struc_to_mat(newstruc.ids, newstruc.xx, newstruc.yy)

    already_assembling = False
    ind = 0
    while not rospy.is_shutdown():
        rate.sleep()
        t += 1. / freq

        opmode = rospy.get_param('opmode', 'normal')
        if opmode == 'disassemble':
            disassembler.take_step(struc_mgr, t, ind)
        elif opmode == 'assemble':
            if not already_assembling:
                rospy.set_param("reset_docking", 1)
                rospy.Rate(9).sleep() # Wait for it to start up
                assembler = AssemblyManager(t, newstruc_mat, trajectory_function)
                assembler.plan_assemblies(struc_mgr)
                assembler.start_assembling_at_time(t)
                already_assembling = True
                print("Set flag to reset dockings")
            else:
                assembler.take_step(struc_mgr, t)

        # Assuming adherence to trajectory that is already loaded in
        # StructureManager handles doing the actual physics of the simulation for
        # all structures, and hence for all individual modules
        struc_mgr.control_step(t, trajectory_function, speed, 
                odom_publishers, pos_publishers, tf_broadcaster)

        if t > 4.0 and not undocked: # Split the structure
            rospy.wait_for_service('SplitStructure')
            print("Parallelized undocking procedure triggered")
            disassembler = DisassemblyManager(reconf_map, t, struc_mgr.strucs[0], trajectory_function)
            undocked = True

        ind += 1
    #struc_mgr.make_plots()

def test_undock_along_path(mset1, wayptset, speed=0.5, test_id="", split_dim=0, breakline=1, split_ind=0):
    # Import here in case want to run w/o mqscheduler package
    from modquad_sched_interface.interface import convert_modset_to_struc
    from compiled_scheduler.modset import modset
    global traj_func

    # Setup
    trajectory_function = traj_func
    traj_vars = trajectory_function(0, speed, None, wayptset)

    # Order of calls is important
    # 1. solve as if nothing was wrong
    gsolve(mset1, waypts=traj_vars.waypts, speed=speed)

    # 2. introduce fault, which means we need to reconfigure
    mset1.fault_rotor(3, 0)

    # 3. Generate the Structure object with the fault
    struc1 = convert_modset_to_struc(mset1)
    struc1.traj_vars = traj_vars

    # 4. Generate the modset object we will store reallocation in
    mset2 = copy.deepcopy(mset1)
    #modset(mset1.num_mod, np.copy(mset1.struc), mset1, mset1.mod_ids)

    # 5. Reallocate modules to positions
    gsolve(mset2, waypts=traj_vars.waypts, speed=speed)

    # 6. Generate goal structure
    struc2 = convert_modset_to_struc(mset2)
    struc2.traj_vars = traj_vars

    # 7. Find path of disassembly
    [cost, reconf_map] = reconfigure(mset1, mset2, waypts=traj_vars.waypts, speed=speed)

    print("Reconfigure this structure: {}".format(""))#struc1.gen_hashstring()))
    #struc1 = convert_modset_to_struc(mset1)
    struc3 = convert_modset_to_struc(mset1)
    print(mset1.pi + 1)
    #print(struc3)
    s1 = convert_struc_to_mat(struc1.ids, struc1.xx, struc1.yy)
    print(s1)
    print(struc1.gen_hashstring())
    print(struc1.motor_failure)
    #print(struc1.ids)
    #print(struc1.xx)
    #print(struc1.yy)
    print("To this structure: {}".format(""))#struc2.gen_hashstring()))
    struc2 = convert_modset_to_struc(mset2)
    struc2.traj_vars = traj_vars
    print(mset2.pi + 1)
    #print(struc4)
    s2 = convert_struc_to_mat(struc2.ids, struc2.xx, struc2.yy)
    print(s2)
    print(struc2.gen_hashstring())
    print(struc2.motor_failure)
    #print(struc2.ids)
    #print(struc2.xx)
    #print(struc2.yy)
    print('=======================')

    # 8. Run the simulation of the breakup and reassembly
    simulate(struc1, struc2, reconf_map, waypts=wayptset, loc=[0,0,0], 
            figind=1, speed=speed, filesuffix="{}_noreform".format(test_id))

if __name__ == '__main__':
    print("Starting Undocking Simulation")
    test_undock_along_path(
                       #structure_gen.square(3), 
                       structure_gen.plus(3,3), 
                       #structure_gen.rect(4,1), 
                       #structure_gen.airplane(5,5,3),
                       waypt_gen.line([0,0,0], [10,15,1]), 
                       speed=0.25, test_id="redisassembly")
