#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
import time
import numpy as np

from crazyflie_driver.srv import UpdateParams

from dockmgr.datatype.OdometryManager import OdometryManager

from modsim import params
from modsim.params import RunType
from modsim.controller import position_controller

import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen
from modquad_sched_interface.interface import convert_modset_to_struc, \
                                              convert_struc_to_mat

from modquad_sched_interface.simple_scheduler import lin_assign

from modsim.uniquely_named_traj import min_snap_trajectory

def run(mset, wayptset):

    # --- ROS Setup
    rospy.init_node('modrotor_simulator', anonymous=True)

    worldFrame = rospy.get_param("~worldFrame", "/world")

    rospy.wait_for_service('/cf1/update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('/cf1/update_params', UpdateParams)

    rospy.set_param("kalman/resetEstimation", 1)

    speed = rospy.get_param("speed", 1.25)

    velpub = rospy.Publisher('/cf1/cmd_vel', Twist, queue_size=1)

    # --- Other Setup

    # Time
    t = 0.0
    tmax = 20.0

    # Get the odom
    odom_mgr = OdometryManager(1, '/vicon/modquad13', run_type=RunType.VICON)
    odom_mgr.subscribe()

    # Prepare msg object
    msg = Twist()

    msg.linear.x = 0 # pitch [-30, 30] degrees
    msg.linear.y = 0 # roll [-30, 30] degrees
    msg.angular.z = 0 # yaw rate [-200, 200] deg/sec
    msg.linear.z = 5000

    # # Get initial state
    state_vector = odom_mgr.get_new_state(0)
    while state_vector[:3] == [0,0,0]:
        state_vector = odom_mgr.get_new_state(0)
        velpub.publish(msg)
        rospy.loginfo(np.round(state_vector[:3], 3))
        time.sleep(0.01)

    # Add initial position to waypts so that we don't go back to origin
    wayptset = [x + state_vector[:3] for x in wayptset]
    wayptset = np.array([x.tolist() for x in wayptset])

    # Set up the trajectory
    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)

    # 100 Hz loop
    freq = 200
    rate = rospy.Rate(freq)

    # Generate the structure
    lin_assign(mset)
    structure = convert_modset_to_struc(mset)
    structure.state_vector = state_vector
    structure.traj_vars = traj_vars

    pi = convert_struc_to_mat(structure.ids, structure.xx, structure.yy)
    print("Structure Used: \n{}".format(pi.astype(np.int64)))

    tmax = structure.traj_vars.total_dist / speed
    t = 0.0

    rospy.loginfo("Start Control")
    while not rospy.is_shutdown():

        # Simulate, obtain new state and state derivative
        structure.state_vector = odom_mgr.get_new_state(0)

        desired_state = trajectory_function(t, speed, structure.traj_vars)

        # Get new control inputs
        [thrust_newtons, roll, pitch, yaw] = \
                position_controller(structure, desired_state)

        # Cap by max thrust
        # https://wiki.bitcraze.io/misc:investigations:thrust
        # if thrust_newtons > params.max_thrust: # Newtons
        #     thrust_newtons = params.max_thrust
        # elif thrust_newtons < 0:
        #     thrust_newtons = params.max_thrust / 2.0

        # # Convert thrust to PWM range
        thrust = thrust_newtons #(thrust_newtons / params.max_thrust) * 50000
        # roll = 0
        # pitch = 0

        if (round(t,2) % 2.0 == 0):
            res = structure.state_vector[:3] - desired_state[0]
            rospy.loginfo("\tRESIDUAL = {}".format(np.round(res, 3)))
            rospy.loginfo("\tCur = {}".format(
                            np.round(structure.state_vector[:3],3)))
            rospy.loginfo("\tDes = {}".format(
                            np.round(desired_state[0],3)))
            rospy.loginfo("\tRoll = {}, Pitch = {}".format(
                            round(roll, 4), round(pitch, 4)))
            rospy.loginfo("\tN = {}, Thrust = {}".format(
                            round(thrust_newtons, 5), round(thrust, 2)))
            rospy.loginfo("------------------------------------")

        # # Bound pitch
        # if pitch > 30:
        #     pitch = 30
        # elif pitch < -30:
        #     pitch = -30

        # # Bound roll
        # if roll > 30:
        #     roll = 30
        # elif roll < -30:
        #     roll = -30
    
        msg.linear.x = pitch # [-30, 30] degrees
        msg.linear.y = roll  # [-30, 30] degrees
        msg.angular.z = 0    # yaw rate [-200, 200] deg/sec
        msg.linear.z = thrust

        velpub.publish(msg)

        t += 1. / freq

        # The sleep is to make the simulation look like it would in real life
        rate.sleep()

if __name__ == '__main__':

    rospy.loginfo("Start Sim")
    p = 0.2
    rospy.set_param("speed", 0.1)
    results = run( structure_gen.rect(1,1),
                   waypt_gen.waypt_set([[0,0,0], [0,0,p], [0, 0, p*2]]))
                                        # [0.2,0,p], [0.2,0.2,p], 
                                        # [0,0.2,p], [0,0,p],
                                        # [0,0,p/2],[0,0,p/4]]))
    print("---------------------------------------------------------------")

