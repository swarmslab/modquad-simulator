#!/usr/bin/env python

import rospy
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Empty

### MODSIM
from modsim.trajectory import min_snap_trajectory
from modsim.controller import position_controller
from modsim.util.state import init_state, state_to_quadrotor

### SCHEDULER INTERFACE
import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

from modquad_sched_interface.simple_scheduler import lin_assign

from modquad_sched_interface.interface import convert_modset_to_struc, \
                                              convert_struc_to_mat

### DOCK MANAGER
from dockmgr.datatype.OdometryManager import OdometryManager

def test_shape_with_waypts(mset, wayptset, speed=1, test_id="", 
        doreform=False, max_fault=1, rand_fault=False):

    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "world")
    name = rospy.get_param("~name")
    r = rospy.get_param("~rate")
    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")

    r = 100
    rate = rospy.Rate(r)


    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)
    loc=[4.27,-0.724,0]
    state_vector = init_state(loc, 0)

    # Generate the structure
    lin_assign(mset)
    struc1 = convert_modset_to_struc(mset)
    struc1.state_vector = state_vector
    struc1.traj_vars = traj_vars

    pi = convert_struc_to_mat(struc1.ids, struc1.xx, struc1.yy)
    print("Structure Used: \n{}".format(pi.astype(np.int64)))

    # msg = PoseStamped()
    # msg.header.seq = 0
    # msg.header.stamp = rospy.Time.now()
    # msg.header.frame_id = worldFrame
    # msg.pose.position.x = x
    # msg.pose.position.y = y
    # msg.pose.position.z = z
    # quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    # msg.pose.orientation.x = quaternion[0]
    # msg.pose.orientation.y = quaternion[1]
    # msg.pose.orientation.z = quaternion[2]
    # msg.pose.orientation.w = quaternion[3]

    # pub = rospy.Publisher(name, PoseStamped, queue_size=1)

    odom_mgr = OdometryManager(1, '/modquad', start_id=12) # 0-indexed start val
    odom_mgr.subscribe()

    # Publish here to control
    velpub = rospy.Publisher('/modquad13/cmd_vel', Twist, queue_size=1)

    modder = 100

    t = 0.0

    np.set_printoptions(precision=2)

    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 10000
    msg.angular.z = 0

    rospy.sleep(6)

    print("START START START START START START !!!!")
    t = 0
    while t < 5:
        velpub.publish(msg)
        t += 1 / r
        rate.sleep()

    print("DONE DONE DONE DONE DONE DONE !!!!")

    # t = 0
    # while not rospy.is_shutdown():
    #     desired_state = trajectory_function(t, speed, struc1.traj_vars)
    #     msg.header.seq += 1
    #     msg.header.stamp = rospy.Time.now()

    #     if round(t, 2) % 1.0 == 0:
    #         rospy.loginfo("[{}] Goal = {}".format(round(t, 2), np.array(desired_state[0])))

    #     msg.pose.position.x = desired_state[0][0]
    #     msg.pose.position.y = desired_state[0][1]
    #     msg.pose.position.z = desired_state[0][2]
    #     quaternion = tf.transformations.quaternion_from_euler(\
    #                                             0, 0, 0)
    #     msg.pose.orientation.x = quaternion[0]
    #     msg.pose.orientation.y = quaternion[1]
    #     msg.pose.orientation.z = quaternion[2]
    #     msg.pose.orientation.w = quaternion[3]
    #     pub.publish(msg)

    #     rate.sleep()

    #     t += 1.0 / r
    #     # if t > 8:
    #     #     t = 2


if __name__ == '__main__':
    print("starting simulation")

    x = 0.1 # 4.27
    y = 0 #-0.724
    z = 0 # 0.5

# zigzag_xz(length, height, num_osc=2.0, start_pt=[0,0,0]):

# zigzag_xy(length, height, num_osc=2.0, start_pt=[0,0,0]):

    print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
    #print(waypt_gen.helix(1, 2, 0.5, [0, 0, 0]))
    print(waypt_gen.zigzag_xy(4, 0.5, 8, [0,0,0.5]))

    results = test_shape_with_waypts(
                       structure_gen.rect(1, 1), 
                       waypt_gen.zigzag_xy(4, 0.75, 8, [0,0,0.5]),
                       #waypt_gen.helix(1, 2, 0.5, [0, 0, 0]),
                       #waypt_gen.waypt_set([[x  , y    , 0],
                       #                     [x  , y    , z],
                       #                     [x  , y+0.5, z],
                       #                     [x+1, y+0.5, z],
                       #                     [x+1, y    , z],
                       #                     [x  , y    , z]
                       #                    ]
                       #                   ),
                       speed=0.25, test_id="controls", 
                       doreform=True, max_fault=1, rand_fault=False)
    print("---------------------------------------------------------------")
