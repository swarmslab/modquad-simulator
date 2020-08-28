#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
import numpy as np
import random

from crazyflie_driver.srv import UpdateParams
from threading import Thread

from modsim.controller import position_controller
from modsim.trajectory import min_snap_trajectory

from modsim import params
from modsim.params import RunType
from modsim.util.state import init_state, state_to_quadrotor

from dockmgr.datatype.OdometryManager import OdometryManager

from modquad_sched_interface.interface import convert_modset_to_struc, \
                                              convert_struc_to_mat

import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

from modquad_sched_interface.simple_scheduler import lin_assign

def ros_setup(speed):
    # ROS SETUP
    rospy.init_node('modrotor_simulator', anonymous=True)

    rospy.wait_for_service('/cf1/update_params')
    rospy.loginfo("found update_params service")

    rospy.set_param("kalman/resetEstimation", 1)

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use

def run(mset, wayptset, t_step=0.01, speed=1):

    ros_setup(speed)

    worldFrame = rospy.get_param("~worldFrame", "/world")
    update_params = rospy.ServiceProxy('/cf1/update_params', UpdateParams)

    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]

    demo_trajectory = rospy.get_param('~demo_trajectory', True)
    odom_topic = rospy.get_param('~odom_topic', '/odom')  

    # TF publisher
    #tf_broadcaster = tf2_ros.TransformBroadcaster()

    # --- OTHER SETUP
    state_log = []
    forces_log = []
    pos_err_log = [0,0,0]

    # Plotting coeffs
    overtime = 1.0

    freq = 500  # 100hz
    rate = rospy.Rate(freq)
    t = 0

    # Subscribe to VICON for feedback
    odom_mgr = OdometryManager(1, '/vicon/modquad13', RunType.VICON)
    odom_mgr.subscribe()

    # Publisher to robot for control
    velpub = rospy.Publisher('/cf1/cmd_vel', Twist, queue_size=1)

    # Publish to robot
    msg = Twist()

    # Get initial state
    state_vector = odom_mgr.get_new_state(0)

    # Add initial position to waypts so that we don't go back to origin
    wayptset = [x + state_vector[:3] for x in wayptset]
    wayptset = np.array([x.tolist() for x in wayptset])

    # Set up the trajectory
    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)

    # Generate the structure
    lin_assign(mset)
    structure = convert_modset_to_struc(mset)
    structure.state_vector = state_vector
    structure.traj_vars = traj_vars

    pi = convert_struc_to_mat(structure.ids, structure.xx, structure.yy)
    print("Structure Used: \n{}".format(pi.astype(np.int64)))

    tmax = structure.traj_vars.total_dist / speed
    while not rospy.is_shutdown(): #t < overtime*tmax + 1.0 / freq:

        # Simulate, obtain new state and state derivative
        structure.state_vector = odom_mgr.get_new_state(0)

        desired_state = trajectory_function(t, speed, structure.traj_vars)

        # Get new control inputs
        [thrust_newtons, roll, pitch, yaw] = \
                position_controller(structure, desired_state)

        # Cap by max thrust
        # https://wiki.bitcraze.io/misc:investigations:thrust
        if thrust_newtons > params.max_thrust: # Newtons
            thrust_newtons = params.max_thrust

        # Convert thrust to PWM range
        thrust = (thrust_newtons / params.max_thrust) * 60000

        # Bound pitch
        if pitch > 30:
            pitch = 30
        elif pitch < -30:
            pitch = -30

        # Bound roll
        if roll > 30:
            roll = 30
        elif roll < -30:
            roll = -30
    
        msg.linear.x = pitch
        msg.linear.y = roll
        msg.linear.z = thrust # Thrust ranges 10000 - 60000
        msg.angular.z = 0 # Yaw rate

        #rospy.loginfo(structure.state_vector[:3])

        velpub.publish(msg)

        t += 1. / freq

        # The sleep is to make the simulation look like it would in real life
        rate.sleep()

if __name__ == '__main__':
    print("starting simulation")
    rospy.set_param("fdd_group_type", "log4")
    random.seed(1)
    p = 1

    results = run( structure_gen.rect(1, 1), 
                   #waypt_gen.helix(radius=2.5, rise=3, num_circ=2),
                   #waypt_gen.waypt_set([[0,0,0],[0,0,1],
                   #                     [p,0,1],[p,p,1],
                   #                     [0,p,1],[0,0,1],
                   #                     [0,0,0]        ]
                   #                   ),
                   waypt_gen.waypt_set([[0,0,0], [0,0.1,p], 
                                        [0.1,0.1,p], [0.1,0,p], 
                                        [0,0,p], [0,0,p/2],
                                        [0,0,p/4],[0,0,p/8]]),
                   t_step=0.01,
                   speed=2.5
                 )
    print("---------------------------------------------------------------")
