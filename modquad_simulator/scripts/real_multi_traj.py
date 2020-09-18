#!/usr/bin/env python3

import numpy as np
import time

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
from std_srvs.srv import Empty

from modsim.datatype.structure import Structure
from modsim.datatype.structure_manager import StructureManager
from dockmgr.datatype.disassembly_manager import DisassemblyManager
from dockmgr.datatype.assembly_manager import AssemblyManager

from crazyflie_driver.srv import UpdateParams
from threading import Thread

from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import min_snap_trajectory

from modsim import params
from modsim.params import RunType
from modsim.util.state import init_state, state_to_quadrotor
from modsim.util.thrust import convert_thrust_newtons_to_pwm

from dockmgr.datatype.OdometryManager import OdometryManager

from modquad_sched_interface.interface import convert_modset_to_struc, \
                                              convert_struc_to_mat

import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

from modquad_sched_interface.simple_scheduler import lin_assign

# Set up for Structure Manager
struc_mgr = None
assembler = None
t = 0.0
traj_func = min_snap_trajectory
start_id = 13 # 1 indexed

def docking_callback(msg):
    global assembler, struc_mgr, traj_func, t, start_id
    if assembler is not None:
        # start_id 1-indexed for handling
        assembler.handle_dockings_msg(struc_mgr, msg, traj_func, t, start_id)
        rospy.set_param("~docked", True)
    else:
        raise ValueError("Assembler object does not exist")

def run(traj_vars, t_step=0.01, speed=1):

    global struc_mgr, assembler, t, traj_func, start_id
    rospy.loginfo("!!READY!!")

    worldFrame = rospy.get_param("~worldFrame", "/world")

    rospy.set_param("kalman/resetEstimation", 1)

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use

    robot_id1 = rospy.get_param('~robot_id', 'modquad')
    rids = [robot_id1]

    # tmax = structure.traj_vars.total_dist / speed

    # Set up topics
    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'
    pos_topic = rospy.get_param('world_pos_topic', '/odom')  

    # TF publisher - what is this for??
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    already_assembling = False
    ind = 0

    freq = 80.0  # 100hz
    rate = rospy.Rate(freq)
    t = 0

    num_robot = 2

    odom_mgr = OdometryManager(2, '/modquad', start_id=start_id-1) # 0-indexed start val
    odom_mgr.subscribe()

    # Publish here to control
    # crazyflie_controller/src/controller.cpp has been modified to subscribe to
    # this topic, and if we are in the ModQuad state, then the Twist message
    # from mq_cmd_vel will be passed through to cmd_vel
    # TODO: modify so that we publish to all modules in the struc instead of
    # single hardcoded one
    publishers = [ rospy.Publisher('/modquad{:02d}/mq_cmd_vel'.format(mid), Twist, queue_size=100) for mid in range (start_id, start_id + num_robot) ]

    # Publish to robot
    msg = Twist()

    np.set_printoptions(precision=1)

    # First few msgs will be zeros
    msg.linear.x = 0 # roll [-30, 30] deg
    msg.linear.y = 0 # pitch [-30, 30] deg
    msg.linear.z = 0 # Thrust ranges 10000 - 60000
    msg.angular.z = 0 # yaw rate

    # Start by sending NOPs so that we have known start state
    # Useful for debugging and safety
    t = 0
    while t < 5:
        t += 1.0 / freq
        [ p.publish(msg) for p in publishers ]
        if round(t, 2) % 1.0 == 0:
            rospy.loginfo("Sending zeros at t = {}".format(t))
        rate.sleep()

    # shutdown
    rospy.on_shutdown(landing)

    # Update odom
    rospy.sleep(1)

    structure = struc_mgr.strucs[0]
    
    structure.state_vector = odom_mgr.get_new_state(0)

    """
    THIS WILL NOT AUTOMATICALLY CAUSE THE ROBOT TO DO ANYTHING!!
    YOU MUST PAIR THIS WITH MODIFIED CRAZYFLIE_CONTROLLER/SRC/CONTROLLER.CPP
    AND USE JOYSTICK TO SWITCH TO MODQUAD MODE FOR THESE COMMANDS TO WORK
    """
    tstart = rospy.get_time()
    t = 0
    rospy.loginfo("Start Control")
    while not rospy.is_shutdown():
        # Update time
        t = rospy.get_time() - tstart

        # Obtain new state from VICON
        structure.state_vector = odom_mgr.get_new_state(0)

        desired_state = traj_func(t, speed, traj_vars)

        # Get new control inputs
        [thrust_newtons, roll, pitch, yaw] = \
                position_controller(structure, desired_state)

        # Convert thrust to PWM range
        thrust = convert_thrust_newtons_to_pwm(thrust_newtons)
 
        # Update message content
        msg.linear.x  = pitch # pitch [-30, 30] deg
        msg.linear.y  = roll  # roll [-30, 30] deg
        msg.linear.z  = thrust # Thrust ranges 10000 - 60000
        msg.angular.z = yaw # yaw rate

        if round(t, 2) % 0.5 == 0:
            rospy.loginfo("[{}] {}".format(t, 
                                        np.array([thrust, roll, pitch, yaw])))
            rospy.loginfo("     Des={}, Is={}".format(
                                            np.array(desired_state[0]), 
                                            np.array(structure.state_vector[:3])))

        # Send control message
        # velpub.publish(msg)
        [ p.publish(msg) for p in publishers ]

        # The sleep preserves sending rate
        rate.sleep()

def landing():
    prefix = 'modquad'
    n = rospy.get_param("num_robots", 2)
    land_services = [rospy.ServiceProxy('/modquad{:02d}/land'.format(mid), Empty) for mid in range(13, 13 + n)]
    for land in land_services:
        land()
    rospy.sleep(5)

def test_shape_with_waypts(num_struc, wayptset, speed=1, test_id="", 
        doreform=False, max_fault=1, rand_fault=False):

    global assembler, struc_mgr, traj_func, t, start_id
    # Need to call before reset_docking to ensure it gets right start_id
    start_mod_id = 12
    rospy.set_param('start_mod_id', start_mod_id) # 0-indexed

    # Switching to True indicates StructureManager initialized
    # and onboard control params are updated
    rospy.set_param('~docked', False)

    # Subscribe to /dockings so that you can tell when to combine structures
    rospy.Subscriber('/dockings', Int8MultiArray, docking_callback) 

    traj_func = min_snap_trajectory
    traj_vars = traj_func(0, speed, None, wayptset)
    loc=[0,0,0]
    state_vector = init_state(loc, 0)

    struc_list = []
    for s in range(num_struc):
        # Generate the structure
        mset = structure_gen.rect(1, 1)
        lin_assign(mset)
        struc = convert_modset_to_struc(mset)
        struc.state_vector = state_vector
        struc.traj_vars = traj_vars

        # Overwrite the conversion id
        struc.ids=['modquad{:02d}'.format(s + start_id)]

        print("IDs of this new struc are: {}".format(struc.ids))

        # Add to the list of single-robot structures
        struc_list.append(struc)

    pi_list = [convert_struc_to_mat(struc.ids, struc.xx, struc.yy) for struc in struc_list]
    print("Structures Used: ")
    [print("    {}".format(pi.astype(np.int64))) for pi in pi_list]

    finalpi=np.array([14,13])
    assembler = AssemblyManager(t, finalpi, traj_func)
    assert assembler != None

    # Initialize the StructureManager
    [print(s) for s in struc_list]
    struc_mgr = StructureManager(struc_list)
    assert struc_mgr != None

    rospy.on_shutdown(landing)

    rospy.init_node('modrotor_simulator')

    time.sleep(2)

    # Reset dockings for dock_detector
    rospy.loginfo("Reset docking flag")
    rospy.set_param("reset_docking", 1)

    # Wait for structure to be recognized
    rospy.loginfo("Wait for /dockings callback to update control params")
    while not rospy.get_param('~docked', False):
        rospy.Rate(5).sleep()
    rospy.loginfo("Docked structure entered into StructureManager")

    run(speed=speed, traj_vars=traj_vars)

if __name__ == '__main__':
    print("starting simulation")

    # The place, according to mocap, where robot will start
    x =  6.3#6.68 
    y = -1.0#0.64 
    z =  0.5#0.00 

    num_struc = 2
    results = test_shape_with_waypts(
                       num_struc, 
                       #waypt_gen.zigzag_xy(2.5, 1.0, 4, start_pt=[x,y,0.2]),
                       #waypt_gen.helix(radius=0.5, 
                       #                rise=0.75, 
                       #                num_circ=3, 
                       #                start_pt=[x, y, 0.0]),
                       waypt_gen.waypt_set([[x    , y    , 0.0],
                                            [x    , y    , 0.1],
                                            [x    , y    , 0.5],
                                            [x    , y    , 0.8]]),
                       #waypt_gen.waypt_set([[x    , y    , 0.0],
                       #                     [x    , y    , 0.1],
                       #                     [x    , y    , 0.5],
                       #                     [x    , y    , 0.8],
                       #                     [x    , y+1.5, 0.8],
                       #                     [x+0.5, y+1.5, 0.8],
                       #                     [x+1.5, y+1.5, 0.8],
                       #                     [x+0.5, y+0.5, 0.8],
                       #                     [x    , y    , 0.8],
                       #                     [x    , y    , 0.2]
                       #                    ]
                       #                   ),
                       speed=1.25, test_id="controls", 
                       doreform=True, max_fault=1, rand_fault=False)
    print("---------------------------------------------------------------")
