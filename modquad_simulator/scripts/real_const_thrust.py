#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Twist

import rospy
import tf
from crazyflie_driver.srv import UpdateParams
from threading import Thread

def run():
    # Time
    t = 0.0
    tmax = 20.0

    velpub = rospy.Publisher('/cf1/cmd_vel', Twist, queue_size=2)

    # 100 Hz loop
    freq = 100
    rate = rospy.Rate(freq)

    msg = Twist()

    while not rospy.is_shutdown() and t < tmax:
        # Roll and Pitch
        msg.linear.x = 0 # pitch [-30, 30] degrees
        msg.linear.y = 0 # roll [-30, 30] degrees

        # Yaw Rate
        msg.angular.z = 0 # yaw rate [-200, 200] deg/sec

        # Thrust ranges 10000 - 60000 mapped to PWM output
        if t < tmax / 2:
            msg.linear.z = ( t / (tmax / 2.0) ) * 60000
        else:
            msg.linear.z = ( (tmax - t) / (tmax / 2.0) ) * 60000

        velpub.publish(msg)

        t += 1. / freq

        # The sleep is to make the simulation look like it would in real life
        rate.sleep()

if __name__ == '__main__':
    print("starting simulation")

    rospy.init_node('const_thrust_test', anonymous=True)

    worldFrame = rospy.get_param("~worldFrame", "/world")

    rospy.wait_for_service('/cf1/update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('/cf1/update_params', UpdateParams)

    rospy.set_param("kalman/resetEstimation", 1)

    run()    
    print("---------------------------------------------------------------")
