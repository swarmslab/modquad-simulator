#!/usr/bin/env python3

import numpy as np
import time
import matplotlib.pyplot as plt

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
from std_srvs.srv import Empty, EmptyRequest

from modsim.datatype.structure import Structure
from modsim.datatype.structure_manager import StructureManager
from dockmgr.datatype.disassembly_manager import DisassemblyManager
from dockmgr.datatype.assembly_manager import AssemblyManager

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

# Set up for Structure Manager
t = 0.0
traj_func = min_snap_trajectory
start_id = 16 # 1 indexed

def update_att_ki_gains(start_id, num_robot):
    # Zero the attitude I-Gains
    srv_name_set = ['/modquad{:02d}/zero_att_i_gains'.format(mid) for mid in range(start_id, start_id+num_robot)]
    zero_att_i_gains_set = [ rospy.ServiceProxy(srv_name, Empty) 
                             for srv_name in srv_name_set
                           ]
    rospy.loginfo('Wait for all zero_att_gains services')
    [rospy.wait_for_service(srv_name) for srv_name in srv_name_set]
    rospy.loginfo('Found all zero_att_gains services')
    msg = EmptyRequest()
    [zero_att_i_gains(msg) for zero_att_i_gains in zero_att_i_gains_set]


def run(traj_vars, t_step=0.01, speed=1):
    global t, traj_func, start_id
    rospy.loginfo("!!READY!!")

    worldFrame = rospy.get_param("~worldFrame", "/world")

    rospy.set_param("kalman/resetEstimation", 1)

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use
    rospy.set_param('is_modquad_sim', False)

    freq = 100.0  # 100hz
    rate = rospy.Rate(freq)
    t = 0
    num_robot = 4
    np.set_printoptions(precision=1)

    # Publish here to control
    # crazyflie_controller/src/controller.cpp has been modified to subscribe to
    # this topic, and if we are in the ModQuad state, then the Twist message
    # from mq_cmd_vel will be passed through to cmd_vel
    publishers = [ rospy.Publisher('/modquad{:02d}/mq_cmd_vel'.format(mid), Twist, queue_size=100) for mid in range (start_id, start_id + num_robot) ]

    update_att_ki_gains(start_id, num_robot) 

    # First few msgs will be zeros
    msg = Twist()
    msg.linear.x = 0 # roll [-30, 30] deg
    msg.linear.y = 0 # pitch [-30, 30] deg
    msg.linear.z = 0 # Thrust ranges 10000 - 60000
    msg.angular.z = 0 # yaw rate

    # Start by sending NOPs so that we have known start state
    # Useful for debugging and safety
    t = 0
    while t < 3:
        t += 1.0 / freq
        [ p.publish(msg) for p in publishers ]
        if round(t, 2) % 1.0 == 0:
            rospy.loginfo("Sending zeros at t = {}".format(round(t,2)))
        rate.sleep()

    """
    THIS WILL NOT AUTOMATICALLY CAUSE THE ROBOT TO DO ANYTHING!!
    YOU MUST PAIR THIS WITH MODIFIED CRAZYFLIE_CONTROLLER/SRC/CONTROLLER.CPP
    AND USE JOYSTICK TO SWITCH TO MODQUAD MODE FOR THESE COMMANDS TO WORK
    """
    t = 0
    roll   = 0.0
    pitch  = 0.0
    yaw    = 0.0
    thrust = 60000

    # Update message content
    msg.linear.x  = pitch  # pitch [-30, 30] deg
    msg.linear.y  = roll   # roll [-30, 30] deg
    msg.linear.z  = thrust # Thrust ranges 10000 - 60000
    msg.angular.z = yaw    # yaw rate

    rospy.loginfo("Start Control")
    rospy.loginfo("Send thrust {}".format(thrust))
    while not rospy.is_shutdown() and t < 20.0:
        # Update time
        t += 1.0/freq

        # Send control message
        [ p.publish(msg) for p in publishers ]

        # The sleep preserves sending rate
        rate.sleep()
    landing()

def landing():
    prefix = 'modquad'
    num_robot = rospy.get_param("num_robots", 4)

    publishers = [ rospy.Publisher('/modquad{:02d}/cmd_vel'.format(mid), Twist, queue_size=100) for mid in range (start_id, start_id + num_robot) ]

    # First few msgs will be zeros
    msg = Twist()
    msg.linear.x = 0 # roll [-30, 30] deg
    msg.linear.y = 0 # pitch [-30, 30] deg
    msg.linear.z = 0 # Thrust ranges 10000 - 60000
    msg.angular.z = 0 # yaw rate

    timer = rospy.Rate(1)

    for t in [30000, 20000, 10000, 0]:
        msg.linear.z = t
        [p.publish(msg) for p in publishers]
        timer.sleep()

def test_shape_with_waypts(num_struc, wayptset, speed=1, test_id="", 
        doreform=False, max_fault=1, rand_fault=False):

    global traj_func, t, start_id

    traj_func = min_snap_trajectory
    traj_vars = traj_func(0, speed, None, wayptset)

    rospy.on_shutdown(landing)

    rospy.init_node('modrotor_simulator')

    run(speed=speed, traj_vars=traj_vars)

if __name__ == '__main__':
    print("starting simulation")

    # The place, according to mocap, where robot will start
    x = 6.68
    y = 0.64
    z = 0.00

    num_struc = 3
    results = test_shape_with_waypts(
                       num_struc, 
                       waypt_gen.waypt_set([[x    , y    , 0.0],
                                            [x    , y    , 0.1],
                                            [x    , y    , 0.5]
                                           ]),
                       speed=0.05, test_id="controls", 
                       doreform=True, max_fault=1, rand_fault=False)
    print("---------------------------------------------------------------")
