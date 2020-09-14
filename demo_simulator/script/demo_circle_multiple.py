#!/usr/bin/env python


import math

import tf
from geometry_msgs.msg import PoseStamped
import rospy
from std_srvs.srv import Empty

from modsim.controller import position_controller
from modsim.trajectory import min_snap_trajectory

from modsim import params
from modsim.params import RunType

from dockmgr.datatype.OdometryManager import OdometryManager

def goal_to_pose(x, y, z, yaw):
    goal = PoseStamped()
    goal.header.seq = 0
    goal.header.frame_id = '/world'

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]
    return goal


def landing():
    prefix = 'modquad'
    n = rospy.get_param("num_robots", 1)
    land_services = [rospy.ServiceProxy('/%s0%d/land' % (prefix, i + 1), Empty) for i in range(n)]
    for land in land_services:
        land()
    rospy.sleep(5)


def circular_motion():
    rospy.init_node('circular', anonymous=True)
    n = rospy.get_param("num_robots", 1)
    # Prefix
    prefix = 'modquad'

    # Goal publishers
    publishers = [rospy.Publisher('/%s0%d/goal' % (prefix, i + 1), PoseStamped, queue_size=1) for i in range(n)]

    # Takeoff service
    rospy.loginfo("Taking off, wait a couple of seconds.")
    takeoff_services = [rospy.ServiceProxy('/%s0%d/takeoff' % (prefix, i + 1), Empty) for i in range(n)]

    odom_mgr = OdometryManager(1, '/modquad', start_id=12) # 0-indexed start val
    odom_mgr.subscribe()

    # takeoff for all robots
    for takeoff in takeoff_services:
        takeoff()

    # shutdown
    rospy.on_shutdown(landing)
    # Time counter
    t = 1.
    s = 100.
    # Circle loop
    # while not rospy.is_shutdown():
    #     for i in range(n):
    #         theta = t / s + i * 2 * math.pi / n
    #         publishers[i].publish(goal_to_pose(math.cos(theta), math.sin(theta), 0.2*math.sin(1*theta)+1, theta + math.pi/2))

        # Simulate, obtain new state and state derivative
        structure.state_vector = odom_mgr.get_new_state(0)
        rospy.loginfo(structure.state_vector[:3])

        # print(structure.state_vector)

        desired_state = trajectory_function(t, speed, structure.traj_vars)

        # # Get new control inputs
        # [thrust_newtons, roll, pitch, yaw] = \
        #         position_controller(structure, desired_state)

        # thrust_newtons = 30000

        # ## Cap by max thrust
        # ## https://wiki.bitcraze.io/misc:investigations:thrust
        # if thrust_newtons > params.max_thrust: # Newtons
        #     thrust_newtons = params.max_thrust

        # # Convert thrust to PWM range
        # thrust = (thrust_newtons / params.max_thrust) * 60000
 
        # msg.linear.x = 0 # roll [-30, 30] deg
        # msg.linear.y = 0 # pitch [-30, 30] deg
        # msg.linear.z = thrust # Thrust ranges 10000 - 60000
        # msg.angular.z = 0 # yaw rate

        # t += 1. / freq

        # velpub.publish(msg)


    #     t += 1
    #     rospy.sleep(.1)


if __name__ == '__main__':
    circular_motion()
