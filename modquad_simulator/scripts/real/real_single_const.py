#!/usr/bin/env python2

import numpy as np
import time
import matplotlib.pyplot as plt

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
t = 0.0
start_id = 15 # 1 indexed

def setup_params():
    rospy.set_param("kalman/resetEstimation", 1)
    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', 1)
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use

def run():

    global t, start_id

    freq = 100.0  # 100hz
    rate = rospy.Rate(freq)
    t = 0

    num_robot = 1

    # Publish here to control
    # crazyflie_controller/src/controller.cpp has been modified to subscribe to
    # this topic, and if we are in the ModQuad state, then the Twist message
    # from mq_cmd_vel will be passed through to cmd_vel
    # TODO: modify so that we publish to all modules in the struc instead of
    # single hardcoded one
    publishers = [ rospy.Publisher('/modquad{:02d}/mq_cmd_vel'.format(mid), Twist, queue_size=100) for mid in range (start_id, start_id + num_robot) ]

    rospy.loginfo("Made publisher: /modquad{:02d}/mq_cmd_vel".format(start_id))

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
    while t < 3:
        t += 1.0 / freq
        [ p.publish(msg) for p in publishers ]
        if round(t, 2) % 1.0 == 0:
            rospy.loginfo("Sending zeros at t = {}".format(round(t, 2)))
        rate.sleep()

    # Update odom
    rospy.sleep(1)

    """
    THIS WILL NOT AUTOMATICALLY CAUSE THE ROBOT TO DO ANYTHING!!
    YOU MUST PAIR THIS WITH MODIFIED CRAZYFLIE_CONTROLLER/SRC/CONTROLLER.CPP
    AND USE JOYSTICK TO SWITCH TO MODQUAD MODE FOR THESE COMMANDS TO WORK
    """
    roll   =     0.0
    pitch  =     0.0
    yaw    =     0.0
    thrust = 60000.0

    tstart = rospy.get_time()
    t = 0
    rospy.loginfo("Start Control")
    while not rospy.is_shutdown() and t < 10.0:
        # Update time
        t += 1.0/freq

        # Update message content
        msg.linear.x  = pitch  # pitch [-30, 30] deg
        msg.linear.y  = roll   # roll [-30, 30] deg
        msg.linear.z  = thrust # Thrust ranges 10000 - 60000
        msg.angular.z = yaw    # yaw rate

        # Send control message
        [ p.publish(msg) for p in publishers ]

        # The sleep preserves sending rate
        rate.sleep()

    # Shutdown
    msg.linear.x   = 0.0
    msg.linear.y   = 0.0
    msg.linear.z   = 0.0
    msg.angular.z  = 0.0
    [ p.publish(msg) for p in publishers ]

if __name__ == '__main__':
    print("starting simulation")
    rospy.init_node('modquad_tune')
    setup_params()
    run()
    print("---------------------------------------------------------------")
