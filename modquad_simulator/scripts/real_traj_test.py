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

    # rospy.wait_for_service('/modquad13/update_params')
    # rospy.loginfo("found update_params service")
    # update_params = rospy.ServiceProxy('/modquad13/update_params', UpdateParams)

    rospy.set_param("kalman/resetEstimation", 1)

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use

    robot_id1 = rospy.get_param('~robot_id', 'modquad')
    rids = [robot_id1]

    #demo_trajectory = rospy.get_param('~demo_trajectory', True)
    #odom_topic = rospy.get_param('~odom_topic', '/odom')  

    tmax = structure.traj_vars.total_dist / speed

    # Plotting coeffs
    overtime = 1.0

    # TF publisher
    #tf_broadcaster = tf2_ros.TransformBroadcaster()

    freq = 80.0  # 100hz
    rate = rospy.Rate(freq)
    t = 0

    #odom_mgr = OdometryManager(1, '/vicon/modquad13', start_id=12) # 0-indexed start val
    odom_mgr = OdometryManager(1, '/modquad', start_id=12) # 0-indexed start val
    odom_mgr.subscribe()

    # Publish here to control
    velpub = rospy.Publisher('/modquad13/mq_cmd_vel', Twist, queue_size=100)

    # # Takeoff service
    # rospy.loginfo("Taking off, wait a couple of seconds.")
    # takeoff_services = [rospy.ServiceProxy('/modquad13/takeoff', Empty)]

    # Publish to robot
    msg = Twist()

    np.set_printoptions(precision=1)

    # First few msgs will be zeros
    msg.linear.x = 0 # roll [-30, 30] deg
    msg.linear.y = 0 # pitch [-30, 30] deg
    msg.linear.z = 0 # Thrust ranges 10000 - 60000
    msg.angular.z = 0 # yaw rate

    t = 0
    while t < 5:
        t += 1.0 / freq
        velpub.publish(msg)
        if round(t, 2) % 1.0 == 0:
            rospy.loginfo("Sending zeros at t = {}".format(t))
        rate.sleep()

    # # takeoff for all robots
    # rospy.loginfo("Request takeoff")
    # for takeoff in takeoff_services:
    #     takeoff()

    # shutdown
    rospy.on_shutdown(landing)

    # Update odom
    rospy.sleep(1)
    structure.state_vector = odom_mgr.get_new_state(0)

    tstart = rospy.get_time()
    t = 0
    rospy.loginfo("Start Control")
    while not rospy.is_shutdown(): # and t < overtime*tmax + 1.0 / freq:
        t = rospy.get_time() - tstart

        # Simulate, obtain new state and state derivative
        structure.state_vector = odom_mgr.get_new_state(0)
        #rospy.loginfo(structure.state_vector[:3])

        # print(structure.state_vector)

        desired_state = trajectory_function(t, speed, structure.traj_vars)

        # Overwrite for hover
        #desired_state[0] = [6.3, 0, 0.8] #structure.state_vector[:3]

        # # Get new control inputs
        [thrust_newtons, roll, pitch, yaw] = \
                position_controller(structure, desired_state)

        # # Convert thrust to PWM range
        thrust = convert_thrust_newtons_to_pwm(thrust_newtons)
 
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

        #t += 1.0 / freq

        velpub.publish(msg)

        # The sleep is to make the simulation look like it would in real life
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
#,[x,y-0.5,z-0.5],
#                                            [x+1,y,z],[x,y+1,z],
#                                            [x,y,z],[x,y,0.1]
    x = 6.68 # 6.3
    y = 0.64 #-1.0
    z = 0.00 # 0.5

    results = test_shape_with_waypts(
                       structure_gen.rect(1, 1), 
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
