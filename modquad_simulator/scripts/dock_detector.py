#!/usr/bin/env python
"""
This runs a detector that uses modquad module positions to determine whether 
a docking action should occur
"""
#TODO Write service to publish number of used robots, otherwise we have to change
#    it in multiple files everytime we change it

# Python packages
import numpy as np
from itertools import combinations
from math import sqrt, degrees, atan2

# ROS std packages
import rospy
from std_msgs.msg import Int8MultiArray

# Custom  
from modsim import params
from modsim.datatype.structure import Structure
from modsim.datatype.structure_manager import StructureManager
from dockmgr.datatype.OdometryManager import OdometryManager
#from dock_manager.srv import ManualDock, ManualDockResponse

#In rviz, we have more robots instantiated than we necessarily use, 
#   so don't use "num_robots"
#n = rospy.get_param('num_robots', 2)
n = rospy.get_param('num_used_robots', 4)

# Docking vector: this vector represents the element of the triangular matrix of matches
# The number 1,2,3,4 represents if the connection is up, right, down or left respectively.
docking = [0 for _ in range(n * (n - 1) / 2)]
manual_docking = [-1 for _ in range(n * (n - 1) / 2)]


def compute_docking_array(x, n, docking_d, min_z_dif=0.005):
    """
    Identifies the pairs of docked robots based on the robot locations. 
    It uses the global variable: docking vector to avoid to remove  attachments that were already-made. 
    :param x: a vector with the locations in 3D. it has size nx3
    :param n: number of robots
    :param docking_d: docking distance
    :return: return a docking vector, which has size comb(n,2)
    """
    global docking

    # Pairs of robots.
    pairs = list(combinations(range(n), 2))
    #print(pairs)

    # Check every pair
    for k, (i, j) in enumerate(pairs):
        if docking[k] > 0:
            continue

        # Distance
        diff = x[j] - x[i]
        dist = sqrt(np.dot(diff, diff.T))
        z_diff = abs(x[j][2] - x[i][2])

        if dist < docking_d and z_diff < min_z_dif:
            dx, dy = diff[:2]
            angle = degrees(atan2(dy, dx))

            if -45 < angle <= 45:  # up 1
                docking[k] = 1
            elif -135 < angle <= -45:  # right 2
                docking[k] = 2
            elif 45 < angle <= 135:  # left 4
                docking[k] = 4
            else:  # down 3
                docking[k] = 3

    print(docking, docking_d)
    return docking


def overwrite_manual_docking(docking_array):
    """
    Takes the docking array vector and replaces the manual entries by the ones that were manually set.
    :param docking_array: original vector that comes from odometry
    :return: overwritten vector.
    """
    # Check the manual elements to be replaced
    for i, dm in enumerate(manual_docking):
        if dm != -1:
            docking_array[i] = dm

    return docking_array


def manual_dock_service(manual_dock_srv):
    """
    This service receives the manual dockings that will overritte the odometry attachments
    :param manual_dock_srv:
    :return:
    """
    global manual_docking

    tmp_manual_docking = []
    for i, val in enumerate(manual_dock_srv.attachments):
        # If it is -1, put whatever it was in the manual docking
        if val == -1:
            val = manual_docking[i]

        tmp_manual_docking.append(val)
        # manual_docking[i] = val

    manual_docking = tmp_manual_docking
    print manual_docking
    return ManualDockResponse()


def detect_dockings():
    rospy.init_node('docking_detector', anonymous=True)
    # service for manual dock
    #rospy.Service('manual_dock', ManualDock, manual_dock_service)

    cage_width = params.cage_width #rospy.get_param('cage_width', 0.158)
    freq = rospy.get_param('~frequency', 100)  # 100hz
    tolerance = rospy.get_param('~docking_tolerance', 0.030)
    min_z_diff = rospy.get_param('~docking_zdiff', 0.030)
    docking_d = cage_width + tolerance  # Docking distance.

    # publisher
    dock_pub = rospy.Publisher('/dockings', Int8MultiArray, queue_size=0)

    # Gets all robot locations
    odometry_manager = OdometryManager(n)
    odometry_manager.subscribe()

    # FIXME this can be fixed for the same frequency of the odometry
    rate = rospy.Rate(freq)
    while not rospy.is_shutdown():
        rate.sleep()

        locations = odometry_manager.get_locations()

        if None in locations:
            rospy.logwarn('Docking detector does not have odometry from all robots' + str(locations))
            rospy.sleep(3)
            continue

        # Create the package based on odometry
        docking_array = compute_docking_array(np.array(locations), n, docking_d,min_z_diff)
        docking_array = overwrite_manual_docking(docking_array)

        # Publish
        msg = Int8MultiArray()
        msg.data = docking_array
        dock_pub.publish(msg)

if __name__ == '__main__':
    detect_dockings()