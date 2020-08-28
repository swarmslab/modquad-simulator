#/usr/bin/env python3

import tf
from geometry_msgs.msg import PoseStamped
import rospy

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


def landing(prefix='modquad'):
    n = rospy.get_param("nr", 1)
    land_services = [rospy.ServiceProxy('/{:s}{:02d}/land'.format(\
                        prefix, i + 1)), Empty) \
                        for i in range(n)]
    for land in land_services:
        land()
    rospy.sleep(5)
