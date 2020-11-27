#!/usr/bin/env python

"""
This is largely a set of functions related to preset flight manouevres
common to all structures
"""
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
from numpy import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import sys
import json
from tf.transformations import euler_from_quaternion
import math

from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import circular_trajectory, simple_waypt_trajectory, \
    min_snap_trajectory

from modsim import params
from modsim.attitude import attitude_controller

from modsim.datatype.structure import Structure

# Functions to publish odom, transforms, and acc
from modsim.util.comm import publish_acc, publish_odom, \
                             publish_transform_stamped, \
                             publish_odom_relative,     \
                             publish_transform_stamped_relative, \
                             publish_structure_acc,     \
                             publish_acc_for_attached_mods, \
                             publish_structure_odometry, \
                             publish_odom_for_attached_mods

from modsim.util.state import init_state, state_to_quadrotor
from modsim.simulation.ode_integrator import simulation_step

from modsim.util.thrust import convert_thrust_pwm_to_newtons

def sim_takeoff(structure, freq, odom_publishers, tf_broadcaster):
    rate = rospy.Rate(freq)

    # Publish to robot
    msg = Twist()

    # TAKEOFF
    taken_off = False

    # Message init for takeoff
    msg.linear.x  = 0  # pitch [-30, 30] deg
    msg.linear.y  = 0  # roll [-30, 30] deg
    msg.linear.z  = 0  # Thrust ranges 10000 - 60000
    msg.angular.z = 0  # yaw rate
    yaw_des = 0 
    pidz_ki = 3500
    rospy.loginfo("Start Control")
    rospy.loginfo("TAKEOFF")
    roll, pitch, yawrate, thrust_pwm = 0,0,0,0
    while not taken_off:
        # Publish odometry
        publish_structure_odometry(structure, odom_publishers, tf_broadcaster)

        if structure.state_vector[2] > 0.05 or msg.linear.z > 50000:
            structure.pos_accumulated_error = msg.linear.z / pidz_ki
            taken_off = True

        # Convert thrust to PWM range
        thrust_pwm += 10000 * (1.0/freq)
        thrust_newtons = convert_thrust_pwm_to_newtons(thrust_pwm)

        # Control output based on crazyflie input
        F_single, M_single = \
                attitude_controller(structure, (thrust_newtons, roll, pitch,
yawrate), yaw_des)

        en_motor_sat = True
        en_fail_rotor_act = True

        # Control of Moments and thrust
        F_structure, M_structure, rotor_forces = \
                modquad_torque_control(
                        F_single, M_single, structure,
                        en_motor_sat, en_fail_rotor_act)

        # Simulate, obtain new state and state derivative
        structure.state_vector = simulation_step( structure,
                                                  structure.state_vector,
                                          F_structure,
                                                  M_structure,
                                                  1.0 / freq             )

        # The sleep preserves sending rate
        rate.sleep()
    rospy.loginfo("COMPLETED TAKEOFF")

def sim_land(structure, freq, odom_publishers, tf_broadcaster):
    rate = rospy.Rate(freq)

    # Publish to robot
    msg = Twist()

    # LANDING
    landed = False

    # Message init for takeoff
    msg.linear.x  = 0  # pitch [-30, 30] deg
    msg.linear.y  = 0  # roll [-30, 30] deg
    msg.linear.z  = 0  # Thrust ranges 10000 - 60000
    msg.angular.z = 0  # yaw rate
    yaw_des = 0 
    thrust_pwm = 35000 # Usually it's somewhere slightly above this for hover
    rospy.loginfo("LANDING")
    roll, pitch, yawrate, thrust_pwm = 0,0,0,0
    while not landed:
        # Publish odometry
        publish_structure_odometry(structure, odom_publishers, tf_broadcaster)

        if structure.state_vector[2] < 0.05 or msg.linear.z < 11000:
            landed = True

        # Convert thrust to PWM range
        thrust_pwm -= 5000 * (1.0/freq)
        thrust_newtons = convert_thrust_pwm_to_newtons(thrust_pwm)

        # Control output based on crazyflie input
        F_single, M_single = \
                attitude_controller(structure, (thrust_newtons, roll, pitch,
yawrate), yaw_des)

        en_motor_sat = True
        en_fail_rotor_act = True

        # Control of Moments and thrust
        F_structure, M_structure, rotor_forces = \
                modquad_torque_control(
                        F_single, M_single, structure,
                        en_motor_sat, en_fail_rotor_act)

        # Simulate, obtain new state and state derivative
        structure.state_vector = simulation_step( structure,
                                                  structure.state_vector,
                                                  F_structure,
                                                  M_structure,
                                                  1.0 / freq             )

        # The sleep preserves sending rate
        rate.sleep()

    rospy.loginfo("COMPLETED LANDING")

