#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import numpy as np

from crazyflie_driver.srv import UpdateParams
from threading import Thread

from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import min_snap_trajectory
from modsim.datatype.structure_manager import StructureManager

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

def run(structure, trajectory_function, sched_mset, t_step=0.01, speed=1):
    rospy.init_node('modrotor_simulator')
    rospy.loginfo("!!READY!!")

    worldFrame = rospy.get_param("~worldFrame", "/world")

    rospy.set_param("kalman/resetEstimation", 1)

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)

    # So that modquad_torque_control knows which mapping to use
    rospy.set_param('rotor_map', 2) 

    robot_id1 = rospy.get_param('~robot_id', 'modquad')
    rids = [robot_id1]

    tmax = structure.traj_vars.total_dist / speed

    # Plotting coeffs
    overtime = 1.0

    freq = 80.0  # 100hz
    rate = rospy.Rate(freq)
    t = 0

    odom_mgr = OdometryManager(1, '/modquad', start_id=12) # 0-indexed start val
    odom_mgr.subscribe()

    # Publish here to control
    # crazyflie_controller/src/controller.cpp has been modified to subscribe to
    # this topic, and if we are in the ModQuad state, then the Twist message
    # from mq_cmd_vel will be passed through to cmd_vel
    # TODO: modify so that we publish to all modules in the struc instead of
    # single hardcoded one
    velpub = rospy.Publisher('/modquad13/mq_cmd_vel', Twist, queue_size=100)

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
        velpub.publish(msg)
        if round(t, 2) % 1.0 == 0:
            rospy.loginfo("Sending zeros at t = {}".format(t))
        rate.sleep()

    # shutdown
    rospy.on_shutdown(landing)

    # Update odom
    rospy.sleep(1)
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

        desired_state = trajectory_function(t, speed, structure.traj_vars)

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
        velpub.publish(msg)

        # The sleep preserves sending rate
        rate.sleep()

def landing():
    prefix = 'modquad'
    n = rospy.get_param("num_robots", 1)
    land_services = [rospy.ServiceProxy('/modquad13/land', Empty)]
    for land in land_services:
        land()
    rospy.sleep(5)

def test_shape_with_waypts(mset, wayptset, speed=1, test_id="", 
        doreform=False, max_fault=1, rand_fault=False):

    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)
    loc=[0,0,0]
    state_vector = init_state(loc, 0)

    # Generate the structure
    lin_assign(mset)
    struc1 = convert_modset_to_struc(mset)
    struc1.state_vector = state_vector
    struc1.traj_vars = traj_vars

    pi = convert_struc_to_mat(struc1.ids, struc1.xx, struc1.yy)
    print("Structure Used: \n{}".format(pi.astype(np.int64)))

    rospy.on_shutdown(landing)
    run(struc1, trajectory_function, mset, speed=speed)

if __name__ == '__main__':
    print("starting simulation")

    # The place, according to mocap, where robot will start
    x = 6.68 # 6.3
    y = 0.64 #-1.0
    z = 0.00 # 0.5

    results = test_shape_with_waypts(
                       structure_gen.rect(2, 1), 
                       #waypt_gen.zigzag_xy(2.5, 1.0, 4, start_pt=[x,y,0.2]),
                       waypt_gen.helix(radius=0.5, 
                                       rise=1.5, 
                                       num_circ=3, 
                                       start_pt=[x, y, 0.0]),
                       #waypt_gen.waypt_set([[x    , y    , 0.0],
                       #                     [x    , y    , 0.1],
                       #                     [x    , y    , 0.5],
                       #                     [x    , y    , 0.8]]),
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
