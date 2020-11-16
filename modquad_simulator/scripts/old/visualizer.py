#!/usr/bin/env python

import numpy as np
import rospy
from nav_msgs.msg import Odometry

from modsim.plot.drawer_vispy import Drawer
from modsim.util.state import state_to_quadrotor


def visualize():
    rospy.init_node('modrotor_visualizer', anonymous=True)
    n = rospy.get_param('num_robots', 1)

    ### Odometry for all robots
    states = {}

    def callback_odom(odom):
        # state vector
        s = np.zeros(13)
        s[0] = odom.pose.pose.position.x
        s[1] = odom.pose.pose.position.y
        s[2] = odom.pose.pose.position.z

        # Velocities
        s[3] = odom.twist.twist.linear.x
        s[4] = odom.twist.twist.linear.y
        s[5] = odom.twist.twist.linear.z

        # Orientation
        s[6] = odom.pose.pose.orientation.x
        s[7] = odom.pose.pose.orientation.y
        s[8] = odom.pose.pose.orientation.z
        s[9] = odom.pose.pose.orientation.w

        # Omega
        s[10] = odom.twist.twist.angular.x
        s[11] = odom.twist.twist.angular.y
        s[12] = odom.twist.twist.angular.z

        topic = odom._connection_header['topic']

        states[topic] = s

    for i in range(n):
        # subscriber
        rospy.Subscriber('/crazy%02d/odom' % (i + 1), Odometry, callback_odom)

    ## Functions



    drawer = None

    # print x

    freq = 50  # 100hz
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        rate.sleep()

        quads = [state_to_quadrotor(s) for s in states.values()]

        # wait for all robots
        if len(quads) < n:
            continue
        if drawer is None:
            drawer = Drawer(quads, refresh_rate=50)
            print 'Starting simulator'

        # Plot
        drawer.plot(quads)


if __name__ == '__main__':
    visualize()
