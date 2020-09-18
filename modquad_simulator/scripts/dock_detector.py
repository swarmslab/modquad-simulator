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
#from dockmgr.datatype.OdometryManager import OdometryManager
from dockmgr.datatype.WorldPosManager import WorldPosManager
#from dock_manager.srv import ManualDock, ManualDockResponse

#In rviz, we have more robots instantiated than we necessarily use, 
#   so don't use "num_robots"
#n = rospy.get_param('num_robots', 2)
#n = rospy.get_param('num_used_robots', 3)
#pos_manager = WorldPosManager(n)

# Docking vector: this vector represents the element of the triangular matrix of matches
# The number 1,2,3,4 represents if the connection is up, right, down or left respectively.
#docking = [0 for _ in range(n * (n - 1) / 2)]

def add_offset_to_pair(pair, offset):
    return (pair[0] + offset, pair[1] + offset)

def compute_docking_array(x, n, docking_d, min_z_dif=0.005, start_id=1):
    """
    Identifies the pairs of docked robots based on the robot locations. 
    It uses the global variable: docking vector to avoid to remove  attachments that were already-made. 
    :param x: a vector with the locations in 3D. it has size nx3
    :param n: number of robots
    :param docking_d: docking distance
    :return: return a docking vector, which has size comb(n,2)
    """
    global docking
    tolerance = rospy.get_param('~docking_tolerance', 0.005)#0.01)#0.030)

    # Pairs of robots.
    pairs = sorted(list(combinations(range(n), 2)))

    # Check every pair
    for k, (i, j) in enumerate(pairs):
        # Ignore docks already made
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

            print('------------')
            print('Pairs: {}'.format(pairs))
            print("valid docking index pair {} with params:\n\tdist = {}, tolerance = {}, zdiff = {}, minzdiff = {}".format(
                (i+start_id, j+start_id), dist, docking_d, z_diff, min_z_dif))
            print(x)
            print(docking)

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
    print(manual_docking)
    return ManualDockResponse()

def detect_dockings():
    global pos_manager, n
    global docking, manual_docking
    rospy.init_node('docking_detector', anonymous=True)
    reset_docking = rospy.set_param('reset_docking', 0)

    # service for manual dock
    #rospy.Service('manual_dock', ManualDock, manual_dock_service)

    cage_width = params.cage_width #rospy.get_param('cage_width', 0.158)
    freq = rospy.get_param('~frequency', 100)  # 100hz
    tolerance = rospy.get_param('~docking_tolerance', 0.030)
    min_z_diff = rospy.get_param('~docking_zdiff', 0.030)
    docking_d = cage_width + tolerance  # Docking distance.

    # publisher
    dock_pub = rospy.Publisher('/dockings', Int8MultiArray, queue_size=0)

    print("Started docking detected, wait for signal to start detecting")
    # Don't need to assemble off the bat
    while not rospy.get_param('reset_docking') == 1:
        rospy.Rate(5).sleep()

    # Use new parameters
    rospy.set_param('reset_docking', 0)

    rospy.Rate(5).sleep()

    # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # FIXME - Currently need to change this manually#!
    n = 2 #rospy.get_param('num_used_robots', 9) # !!!!!!
    # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    start_id = rospy.get_param("start_mod_id", 1)
    pos_manager = WorldPosManager(n, start_id=start_id)
    docking = [0 for _ in range(int(n * (n - 1) / 2))]
    manual_docking = [0] #[-1 for _ in range(n * (n - 1) / 2)]

    # Gets all robot locations
    pos_manager.subscribe()

    # FIXME this can be fixed for the same frequency of the odometry
    rate = rospy.Rate(freq)

    iteration = 0

    while not rospy.is_shutdown():
        rate.sleep()
        #if rospy.get_param('reset_docking') == 1:
        #    docking = [0 for _ in range(n * (n - 1) / 2)]
        #    rospy.set_param('reset_docking', 0)
        #    print("Reset the dockings")

        locations = pos_manager.get_locations()

        if None in locations:
            rospy.logwarn('Docking detector does not have position from all robots' + 
                    str(locations))
            rospy.sleep(3)
            continue

        # Create the package based on odometry
        #docking_array = compute_docking_array(np.array(locations), n, docking_d, min_z_diff, start_id=start_id)
        docking_array = overwrite_manual_docking(manual_docking)

        # Publish
        msg = Int8MultiArray()
        msg.data = docking_array
        dock_pub.publish(msg)

        if iteration > 10:
            manual_docking = [4]
        else:
            iteration += 1

if __name__ == '__main__':
    detect_dockings()
