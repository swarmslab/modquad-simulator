#!/usr/bin/env python3

import sys
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import numpy as np
import matplotlib.pyplot as plt

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

start_id = 13 # 1-indexed

def run(structure, trajectory_function, sched_mset, t_step=0.01, speed=1):
    rospy.init_node('modrotor_simulator')
    rospy.loginfo("!!READY!!")

    worldFrame = rospy.get_param("~worldFrame", "/world")

    rospy.set_param("kalman/resetEstimation", 1)

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use

    robot_id1 = rospy.get_param('~robot_id', 'modquad')
    rids = [robot_id1]

    tmax = structure.traj_vars.total_dist / speed

    # Plotting coeffs
    overtime = 1.0

    # TF publisher - what is this for??
    #tf_broadcaster = tf2_ros.TransformBroadcaster()

    freq = 100.0  # 100hz
    rate = rospy.Rate(freq)
    t = 0

    # 1-indexed
    if len(sys.argv) == 2:
        start_id = int(sys.argv[1])

    num_robot = 1

    odom_mgr = OdometryManager(1, '/modquad', start_id=start_id-1) # 0-indexed start val
    odom_mgr.subscribe()

    # Publish here to control
    # crazyflie_controller/src/controller.cpp has been modified to subscribe to
    # this topic, and if we are in the ModQuad state, then the Twist message
    # from mq_cmd_vel will be passed through to cmd_vel
    # TODO: modify so that we publish to all modules in the struc instead of
    # single hardcoded one
    publishers = [rospy.Publisher('/modquad{}/mq_cmd_vel'.format(start_id), Twist, queue_size=100)]

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
        publishers[0].publish(msg)
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
    tstart = round(rospy.get_time(), 2)
    t = 0
    rospy.loginfo("Start Control")

    pos_log = np.zeros((0,3))
    tlog = []
    xlog = []
    ylog = []
    zlog = []

    vxlog = []
    vylog = []
    vzlog = []

    desx = []
    desy = []
    desz = []

    desvx = []
    desvy = []
    desvz = []

    thrustlog = []
    rollog = []
    pitchlog = []

    while not rospy.is_shutdown() and t < 25:
        # Update time
        t = round(rospy.get_time(),2) - tstart
        tlog.append(t)

        structure.state_vector = odom_mgr.get_new_state(0)
        xlog.append(structure.state_vector[0])
        ylog.append(structure.state_vector[1])
        zlog.append(structure.state_vector[2])

        #vel = structure.state_vector[3:6]
        vxlog.append(structure.state_vector[3])
        vylog.append(structure.state_vector[4])
        vzlog.append(structure.state_vector[5])

        desired_state = trajectory_function(t, speed, structure.traj_vars)
        desx.append(desired_state[0][0])
        desy.append(desired_state[0][1])
        desz.append(desired_state[0][2])

        desvx.append(desired_state[1][0])
        desvy.append(desired_state[1][1])
        desvz.append(desired_state[1][2])

        # Get new control inputs
        [thrust_newtons, roll, pitch, yaw] = \
                position_controller(structure, desired_state)
        rollog.append(roll)
        pitchlog.append(pitch)

        # Convert thrust to PWM range
        thrust = convert_thrust_newtons_to_pwm(thrust_newtons)
        thrustlog.append(thrust)
 
        # Update message content
        msg.linear.x  = pitch # pitch [-30, 30] deg
        msg.linear.y  = roll  # roll [-30, 30] deg
        msg.linear.z  = thrust # Thrust ranges 10000 - 60000
        msg.angular.z = yaw # yaw rate

        if round(t, 2) % 2.0 == 0:
            rospy.loginfo("[{}] {}".format(round(t,1), 
                                        np.array([thrust, roll, pitch, yaw])))
            rospy.loginfo("     Des={}, Is={}".format(
                                            np.array(desired_state[0]), 
                                            np.array(structure.state_vector[:3])))

        # Send control message to all modules
        publishers[0].publish(msg)

        # The sleep preserves sending rate
        rate.sleep()

    landing()

    plt.figure()
    plt.subplot(3,2,1)
    plt.plot(tlog, xlog, 'r')
    plt.plot(tlog, desx, 'k')
    plt.ylabel("X (m)")

    ax1 = plt.subplot(3,2,2)
    ax1.plot(tlog, vxlog, 'r')
    ax1.plot(tlog, desvx, 'k')
    ax1.set_ylabel("X (m/s)")
    ax2 = ax1.twinx()
    ax2.plot(tlog, pitchlog, 'c')

    plt.subplot(3,2,3)
    plt.plot(tlog, ylog, 'g')
    plt.plot(tlog, desy, 'k')
    plt.ylabel("Y (m)")

    ax3 = plt.subplot(3,2,4)
    ax3.plot(tlog, vylog, 'g')
    ax3.plot(tlog, desvy, 'k')
    ax3.set_ylabel("Y (m/s)")
    ax4 = ax3.twinx()
    ax4.plot(tlog, rollog, 'c' )

    plt.subplot(3,2,5)
    plt.plot(tlog, zlog, 'b')
    plt.plot(tlog, desz, 'k')
    plt.ylabel("Z (m)")

    ax5 = plt.subplot(3,2,6)
    ax5.plot(tlog, vzlog, 'b')
    ax5.plot(tlog, desvz, 'k')
    ax5.set_ylabel("Z (m/s)")
    ax6 = ax5.twinx()
    ax6.plot(tlog, thrustlog, 'c' )


    plt.show()

def landing():
    #prefix = 'modquad'
    #n = rospy.get_param("num_robots", 1)
    # land_services = [rospy.ServiceProxy('/modquad{:02d}/land'.format(mid), Empty) for mid in range(13, 13)]
    # for land in land_services:
    #     land()

    land = rospy.ServiceProxy("/modquad{:02d}/land".format(start_id), Empty)
    land()
    rospy.sleep(1)

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
    x =  6.3# 4.89  #6.68 
    y = -1.0#-0.92  #0.64 
    z = 0.00# 0.00  0.5 

    results = test_shape_with_waypts(
                       structure_gen.rect(1, 1), 
                       #waypt_gen.zigzag_xy(2.5, 1.0, 4, start_pt=[x,y,0.2]),
                       #waypt_gen.helix(radius=0.25, 
                       #                rise=0.75, 
                       #                num_circ=3, 
                       #                start_pt=[x, y, 0.0]),
                       waypt_gen.waypt_set([[x    , y    , 0.0],
                                            [x    , y    , 0.1],
                                            [x    , y    , 0.5]
                                            #[x    , y    , 0.8],
                                            #[x+1  , y    , 0.8],
                                            #[x    , y    , 0.8],
                                            #[x    , y    , 0.5],
                                            #[x    , y    , 0.1],
                                            #[x    , y    , 0.0]
                                           ]),
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
                       speed=0.2, test_id="controls", 
                       doreform=True, max_fault=1, rand_fault=False)
    print("---------------------------------------------------------------")
