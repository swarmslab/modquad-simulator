#!/usr/bin/env python
"""
This simulation only tests docking.
You can change the structure type at the bottom, and nominally pass in
a trajectory. 

setup_and_test() will instantiate nine structures and invoke the IROS 2017 
algorithm to assemble the modquad modules into a desired structure
"""
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from numpy import copy
import networkx as nx
import sys

from std_msgs.msg import Int8MultiArray

from modsim.trajectory import circular_trajectory, simple_waypt_trajectory, min_snap_trajectory

from modsim import params

from modsim.datatype.structure import Structure
from modsim.datatype.structure_manager import StructureManager
#from dockmgr.datatype.disassembly_manager import DisassemblyManager
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

num_mod = 9
assembler = None
struc_mgr = None
traj_func = min_snap_trajectory
t = 0.0 # current time

def docking_callback(msg):
    global assembler, struc_mgr, traj_func, t
    if assembler is not None:
        assembler.handle_dockings_msg(struc_mgr, msg, traj_func, t)
    else:
        raise ValueError("Assembler object does not exist")


def simulate(pi, trajectory_function):
    global num_mod, assembler, struc_mgr, t
    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]

    demo_trajectory = rospy.get_param('~demo_trajectory', True)

    speed = rospy.get_param('structure_speed', 0.55)
    rospy.set_param('opmode', 'normal')
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use
    rospy.set_param('num_used_robots', num_mod)

    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'
    modpos_world_topic = rospy.get_param('world_pos_topic', '/world_pos')

    # Odom publisher
    odom_publishers = {id_robot: 
        rospy.Publisher('/' + id_robot + odom_topic, Odometry, queue_size=0) 
        for struc in struc_mgr.strucs for id_robot in struc.ids}

    # This is purely for docking detection so that absolute world position is known
    pos_publishers = {id_robot:
        rospy.Publisher('/' + id_robot + modpos_world_topic, Odometry, queue_size=0) 
        for struc in struc_mgr.strucs for id_robot in struc.ids}

    # TF publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    ########### Simulator ##############
    # Time based on avg desired speed (actual speed *not* constant)
    tmax = 30.0
    overtime = 3.5

    # Params
    freq = 100   # 100hz
    rate = rospy.Rate(freq)
    rospy.set_param("freq", freq)
    t = 0

    # As a start, first get everyone to move so we see they are functional
    for s in struc_mgr.strucs:
        s.traj_vars = trajectory_function(0, speed, None, 
                waypt_gen.line(s.state_vector[:3], s.state_vector[:3] + 1))


    # Plan assemblies before starting time
    assembler = AssemblyManager(t, pi + 1, trajectory_function)
    assembler.plan_assemblies(struc_mgr)

    # Subscribe to /dockings so that you can tell when to combine structures
    rospy.Subscriber('/dockings', Int8MultiArray, docking_callback) 

    docking = False
    while not rospy.is_shutdown():# and t < overtime * tmax:
        rate.sleep()
        t += 1. / freq

        opmode = rospy.get_param('opmode', 'normal')
        if opmode == 'assemble':
            assembler.take_step(struc_mgr, t)

        # Assuming adherence to trajectory that is already loaded in
        # StructureManager handles doing the actual physics of the simulation for
        # all structures, and hence for all individual modules
        struc_mgr.control_step(t, trajectory_function, speed, 
                odom_publishers, pos_publishers, tf_broadcaster)

        if t > 1.0 and not docking: # Start assembling
            print("Parallelized undocking procedure triggered")
            rospy.set_param("opmode", "assemble")
            docking = True
            assembler.start_assembling_at_time(t)

    #struc_mgr.make_plots()

def test_assembly(mset1, wayptset):
    # Import here in case want to run w/o mqscheduler package
    from modquad_sched_interface.interface import convert_modset_to_struc
    from compiled_scheduler.modset import modset

    global num_mod, struc_mgr
    speed = rospy.get_param('structure_speed', 1.0)

    # Setup
    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)
    #gsolve(mset1, waypts=traj_vars.waypts, speed=speed)
    mset1.pi = np.array([[0,1,6],[2,3,7],[4,5,8]])

    # Generate all nine structures as individual modules and make struc_mgr
    # Note that we don't set up the trajectory for these
    struc1 = Structure(
           ['modquad05', 'modquad03', 'modquad01', 'modquad06', 'modquad04', 'modquad02'],
           xx=[0.0, 0.0, 0.0, params.cage_width, params.cage_width, params.cage_width], 
           yy=[0.0, params.cage_width, 2*params.cage_width, 
               0.0, params.cage_width, 2*params.cage_width])
    struc2 = Structure(['modquad09', 'modquad08', 'modquad07'], xx=[0.0, 0.0, 0.0], 
                yy=[0.0, params.cage_width, 2*params.cage_width])
    strucs = [struc1, struc2]
    for i,s in enumerate(strucs):
        s.state_vector = init_state([2*i+1.0,0.0,0.0], 0)
    struc_mgr = StructureManager(strucs)

    print("Assemble this structure (No modid = 0 -- that is empty space):")
    print(mset1.pi + 1)
    print('=======================')

    # 8. Run the simulation of the breakup and reassembly
    simulate(mset1.pi, trajectory_function)

if __name__ == '__main__':
    print("Starting Assembly Simulation")
    rospy.set_param('structure_speed', 1.0)
    test_assembly(structure_gen.zero(2,2), waypt_gen.line([0,0,0],[10,15,2]))
