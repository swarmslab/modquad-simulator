#!/usr/bin/env python
"""
This simulation only tests the functioning of the docking detection.
"""
# Python
import numpy as np
from numpy import copy
import networkx as nx
import sys

# ROS std
import rospy
from std_msgs.msg import Int8MultiArray
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Custom
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


## Control Input
thrust_newtons, roll, pitch, yaw = 0., 0., 0., 0.
num_mod = 4

global assembler, struc_mgr
assembler = None
dislocation_srv = (0., 0.)
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
    #global dislocation_srv, thrust_newtons, roll, pitch, yaw
    global assembler, struc_mgr, t
    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]

    demo_trajectory = rospy.get_param('~demo_trajectory', True)

    speed = rospy.get_param('structure_speed', 1.0)
    rospy.set_param('opmode', 'normal')
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use

    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'

    # Odom publisher for each modquad node (i.e. each quadrotor)
    odom_publishers = {id_robot: 
        rospy.Publisher('/' + id_robot + odom_topic, Odometry, queue_size=0) 
        for struc in struc_mgr.strucs for id_robot in struc.ids}

    # TF publisher -- for world frame?
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    ########### Simulator ##############
    # Time based on avg desired speed (actual speed *not* constant)
    tmax = 20.0
    overtime = 5.5

    # Params
    freq = 100   # 100hz
    rate = rospy.Rate(freq)
    rospy.set_param("freq", freq)
    t = 0

    # Make dummy assembler that only handles docking, doesn't plan them
    assembler = AssemblyManager(0, pi + 1, trajectory_function)

    # Given the current structures, plan out order of attachments
    #assembler.generate_assembly_order(struc_mgr) # Note: this is "offline phase"

    # Subscribe to /dockings so that you can tell when to combine structures
    rospy.Subscriber('/dockings', Int8MultiArray, docking_callback) 

    unplanned = True
    doneplanning = False
    while not rospy.is_shutdown() and t < overtime * tmax:
        rate.sleep()
        t += 1. / freq

        # Assuming adherence to trajectory that is already loaded in
        # StructureManager handles doing the actual physics of the simulation for
        # all structures, and hence for all individual modules
        struc_mgr.control_step(t, trajectory_function, speed, 
                odom_publishers, tf_broadcaster)

        if len(struc_mgr.strucs) == 2 and unplanned:
            assembler.plan_next_z_motion(t, struc_mgr, modid1=2, modid2=3, 
                    zlayer=3, traj_func=trajectory_function)
            unplanned = False
        if len(struc_mgr.strucs) == 2 and not unplanned and t >= assembler.next_time_to_plan and not doneplanning:
            assembler.plan_next_xy_motion(t, struc_mgr, modid1=2, modid2=3, 
                    adj_dir='right', traj_func=trajectory_function)
            doneplanning = True

    # Once everything is done, plot the desired vs. actual positions
    #struc_mgr.make_plots()

if __name__ == '__main__':
    global struc_mgr, num_mod
    print("Starting Assembly Simulation")
    rospy.set_param('structure_speed', 0.5)

    trajectory_function = min_snap_trajectory
    speed = 0.5 # m/s
    rospy.set_param('structure_speed', speed)
    rospy.set_param('num_used_robots', num_mod)

    # Generate trajectories that will put the quads w/i attaching distance
    m = 0.85 # Spacing factor
    traj1 = trajectory_function(0, speed, None, 
            waypt_gen.line([-1,  0,0], [-m*params.cage_width, 0, 1]))
    traj2 = trajectory_function(0, speed, None, 
            waypt_gen.line([ 1,  0,0], [ m*params.cage_width, 0, 1]))
    traj3 = trajectory_function(0, speed, None, 
            waypt_gen.line([-4, -1,0], [-4, -m*params.cage_width, 1]))
    traj4 = trajectory_function(0, speed, None, 
            waypt_gen.line([-4,  1,0], [-4,  m*params.cage_width, 1]))
    #print(traj1.times)
    #print(traj1.waypts)
    print('--------------------------')
    #print(traj2.times)
    #print(traj2.waypts)

    # Generate the single-mod structures
    strucs = [Structure(['modquad{:02d}'.format(i+1)], xx=[0.0], yy=[0.0]) for i in range(num_mod)]
    strucs[0].traj_vars = traj1
    strucs[1].traj_vars = traj2
    strucs[2].traj_vars = traj3
    strucs[3].traj_vars = traj4

    pi = np.array([[1,2,3],[0,0,4]])
    # Initial position of structures should match the first waypt
    for i,s in enumerate(strucs):
        s.state_vector = init_state(s.traj_vars.waypts[0,:], 0)

    # Make the structure manager to handle motion
    struc_mgr = StructureManager(strucs)

    # 8. Run the simulation of the breakup and reassembly
    simulate(pi, trajectory_function)
