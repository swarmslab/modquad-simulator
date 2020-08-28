#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
import numpy as np

from crazyflie_driver.srv import UpdateParams
from threading import Thread

from modsim.controller import position_controller, modquad_torque_control
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


def run(structure, trajectory_function, sched_mset, t_step=0.01, speed=1):
    rospy.init_node('modrotor_simulator', anonymous=True)

    worldFrame = rospy.get_param("~worldFrame", "/world")

    rospy.wait_for_service('/cf1/update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('/cf1/update_params', UpdateParams)

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

    freq = 100  # 100hz
    rate = rospy.Rate(freq)
    t = 0

    odom_mgr = OdometryManager(1, '/vicon/modquad13', run_type=RunType.VICON)
    odom_mgr.subscribe()

    velpub = rospy.Publisher('/cf1/cmd_vel', Twist, queue_size=100)

    # Publish to robot
    msg = Twist()

    rospy.loginfo("Start Control")
    while not rospy.is_shutdown() and t < overtime*tmax + 1.0 / freq:

        # Simulate, obtain new state and state derivative
        #structure.state_vector = odom_mgr.get_new_state(0)

        desired_state = trajectory_function(t, speed, structure.traj_vars)

        # Get new control inputs
        [thrust_newtons, roll, pitch, yaw] = \
                position_controller(structure, desired_state)

        ## Cap by max thrust
        ## https://wiki.bitcraze.io/misc:investigations:thrust
        if thrust_newtons > params.max_thrust: # Newtons
            thrust_newtons = params.max_thrust

        # Convert thrust to PWM range
        thrust = (thrust_newtons / params.max_thrust) * 60000
 
        msg.linear.x = 0 # roll [-30, 30] deg
        msg.linear.y = 0 # pitch [-30, 30] deg
        msg.linear.z = thrust # Thrust ranges 10000 - 60000
        msg.angular.z = 0 # yaw rate

        t += 1. / freq

        velpub.publish(msg)

        # The sleep is to make the simulation look like it would in real life
        rate.sleep()

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

    run(struc1, trajectory_function, mset, speed=speed)

if __name__ == '__main__':
    print("starting simulation")

    p = 5
    results = test_shape_with_waypts(
                       structure_gen.rect(1, 1), 
                       #waypt_gen.helix(2.5, 2, 3),
                       waypt_gen.waypt_set([[0,0,0],[0,0,1],
                                            [p,0,1],[p,p,1],
                                            [0,p,1],[0,0,1],
                                            [0,0,0]        ]
                                          ),
                       speed=2.5, test_id="controls", 
                       doreform=True, max_fault=1, rand_fault=False)
    print("---------------------------------------------------------------")
