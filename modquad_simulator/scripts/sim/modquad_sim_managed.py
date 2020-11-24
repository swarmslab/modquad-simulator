#!/usr/bin/env python


import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from numpy import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import sys
import copy

from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import circular_trajectory, simple_waypt_trajectory, \
    min_snap_trajectory

from modsim import params
from modsim.attitude import attitude_controller
# from modsim.plot.drawer_vispy import Drawer

from modsim.datatype.structure import Structure
from modsim.datatype.structure_manager import StructureManager

from modsim.util.comm import publish_odom, publish_transform_stamped, publish_odom_relative, \
    publish_transform_stamped_relative
from modsim.util.state import init_state, state_to_quadrotor
from modquad_simulator.srv import Dislocation, DislocationResponse
from modsim.simulation.ode_integrator import simulation_step

from modquad_sched_interface.interface import convert_modset_to_struc
import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

fig = plt.figure()
fig2 = plt.figure()

# Control Input
thrust_newtons, roll, pitch, yaw = 0., 0., 0., 0.

dislocation_srv = (0., 0.)


# Control input callback
def control_input_listener(twist_msg):
    global thrust_newtons, roll, pitch, yaw
    # For more info, check:
    # https://github.com/whoenig/crazyflie_ros
    roll = twist_msg.linear.y
    pitch = twist_msg.linear.x
    yaw = twist_msg.angular.z
    thrust_pwm = twist_msg.linear.z

    c1, c2, c3 = -0.6709, 0.1932, 13.0652
    F_g = ((thrust_pwm / 60000. - c1) / c2) ** 2 - c3  # Force in grams
    if F_g<0:
        F_g = 0

    thrust_newtons = 9.81 * F_g / 1000.  # Force in Newtons


def dislocate(disloc_msg):
    global dislocation_srv
    dislocation_srv = (disloc_msg.x, disloc_msg.y)
    return DislocationResponse()  # Return nothing


def publish_structure_odometry(structure, x, odom_publishers, tf_broadcaster):
    ids, xx, yy = structure.ids, structure.xx, structure.yy

    # publish main robot
    main_id = ids[0]
    publish_odom(x, odom_publishers[main_id])
    publish_transform_stamped(main_id, x, tf_broadcaster)
    #print("\tPublish odom for {}".format(main_id))

    # show the other robots
    for robot_id, structure_x, structure_y in zip(ids, xx, yy)[1:]:
        publish_odom_relative(structure_x - xx[0], structure_y - yy[0], robot_id, main_id, odom_publishers[robot_id])
        publish_transform_stamped_relative(robot_id, main_id, structure_x - xx[0], structure_y - yy[0], tf_broadcaster)

def simulate(structures_list, trajectory_function,
        t_step=0.01, speed=1, loc=[1., .0, .0], 
        waypts=None, figind=1, filesuffix=""):
    global dislocation_srv 
    #global thrust_newtons, roll, pitch, yaw

    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]#, robot_id2] #[robot_id1, robot_id2, robot_id3, robot_id4]

    init_x = rospy.get_param('~init_x', 0.)
    init_y = rospy.get_param('~init_y', 0.)
    init_z = rospy.get_param('~init_z', 0.)
    demo_trajectory = rospy.get_param('~demo_trajectory', True)

    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'
    # cmd_vel_topic = rospy.get_param('~cmd_vel_topic', '/cmd_vel')  # '/cmd_vel2'

    # service for dislocate the robot
    rospy.Service('dislocate_robot', Dislocation, dislocate)

    tmax = 10.0#traj_vars.total_dist / speed

    # Subscribe to control input -- no effect when commented, what is this for??
    #[rospy.Subscriber('/' + robot_id + '/cmd_vel', Twist, control_input_listener) for robot_id in rids]

    # Odom publisher
    odom_publishers = {id_robot: 
        rospy.Publisher('/' + id_robot + odom_topic, Odometry, queue_size=0) 
        for structure in structures_list for id_robot in structure.ids }

    # TF publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    state_vector = init_state(loc, 0)

    svecs = [copy.deepcopy(state_vector) for _,_ in enumerate(structures_list)]
    for i,vec in enumerate(svecs):
        structures_list[i].state_vector = vec

    struc_mgr = StructureManager(structures_list)

    freq = 100  # 100hz
    rate = rospy.Rate(freq)
    t = 0
    overtime = 2.0
    #while not rospy.is_shutdown() or t < overtime*tmax + 1.0 / freq:
    while t < overtime*tmax + 1.0 / freq:
        rate.sleep()
        print("{} / {}".format(t, overtime*tmax))

        # Publish odometry
        [publish_structure_odometry(struc_mgr.strucs[i], struc_mgr.strucs[i].state_vector, odom_publishers, tf_broadcaster) for i,_ in enumerate(struc_mgr.strucs)]

        for structure in struc_mgr.strucs:
            #if i != 1: continue

            desired_state = trajectory_function(t, speed, structure.traj_vars)

            # Overwrite the control input with the demo trajectory
            [thrust_newtons, roll, pitch, yaw] = position_controller(
                                                    structure, desired_state)
            #thrust_newtons = round(thrust_newtons, 6)
            #roll = round(roll, 6)
            #pitch = round(pitch, 6)
            #yaw = round(yaw, 6)
            #if i == 2:
            #    print("\tDesire {} = {}".format(struc_mgr.strucs[i].ids, desired_state))
            #    print("in state = {}".format(struc_mgr.strucs[i].state_vector))
            #    print("t={}, Control inputs: {}".format(t, [thrust_newtons, roll, pitch, yaw]))

            # Control output based on crazyflie input
            F_single, M_single = attitude_controller((thrust_newtons, roll, pitch, yaw), 
                                                     structure.state_vector)
            #if i == 2:
            #    print("Control outputs: {}".format(F_single, M_single))
                
            # Control of Moments and thrust
            F_structure, M_structure, rotor_forces = modquad_torque_control(
                    F_single, M_single, structure, motor_sat=True)
            #if i == 2:
            #    print("Control moment, thrust: {}".format(F_structure, M_structure, rotor_forces))

            # Simulate
            structure.state_vector = simulation_step(
                    structure, structure.state_vector, F_structure, M_structure, 1. / freq)
            #if i == 2:
            #    print("\tState {} = {}".format(struc_mgr.strucs[i].ids, struc_mgr.strucs[i].state_vector))

        # Update time
        t += 1. / freq

def setup_and_run():#(mset, wayptset, speed=1, test_id="", doreform=False, max_fault=1):
    from modquad_sched_interface.interface import convert_modset_to_struc
    from compiled_scheduler.modset import modset
    from scheduler.gsolver import gsolve

    # Always use min-snap traj
    trajectory_function = min_snap_trajectory
    speed = 1.0

    # Generate structure 1
    struc1 = Structure(ids=['modquad01'], xx=[0.0], yy=[0.0])
    waypts1 = waypt_gen.spiral(3,3,3,1)
    struc1.traj_vars = trajectory_function(0, speed, None, waypts1)

    # Generate structure 2
    struc2 = Structure(ids=['modquad02'], xx=[0.0], yy=[0.0])
    waypts2 = waypt_gen.spiral(-3,-3,3,2)
    struc2.traj_vars = trajectory_function(0, speed, None, waypts2)

    # Generate structure 2
    struc3 = Structure(ids=['modquad03'], xx=[0.0], yy=[0.0])
    waypts3 = waypt_gen.zigzag_xy(6, 3)
    struc3.traj_vars = trajectory_function(0, speed, None, waypts3)

    # Simulate
    simulate([struc1,struc2,struc3], trajectory_function, 
            loc=[0,0,0], figind=1, speed=speed)

if __name__ == '__main__':
    print("starting simulation")
    setup_and_run()
