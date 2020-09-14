#!/usr/bin/env python
"""
This simulation is meant to test that multiple different structures
can be controlled independently of one another.
"""
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from numpy import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import math
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

# Control Input
thrust_newtons, roll, pitch, yaw = 0., 0., 0., 0.

num_mod = 9

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

# def publish_structure_odometry(structure, x, odom_publishers, tf_broadcaster):
#     # Shorten syntax
#     ids, xx, yy = structure.ids, structure.xx, structure.yy
# 
#     # publish main robot
#     if len(xx) == 1:
#         main_id = ids
#     else:
#         main_id = ids[0]
#     publish_odom(x, odom_publishers[main_id])
#     publish_transform_stamped(main_id, x, tf_broadcaster)
# 
#     # show the other robots
#     if len(xx) > 1:
#         for robot_id, structure_x, structure_y in zip(ids, xx, yy)[1:]:
#             publish_odom_relative(structure_x - xx[0], structure_y - yy[0], 
#                     robot_id, main_id, odom_publishers[robot_id])
#             publish_transform_stamped_relative(robot_id, main_id, structure_x - xx[0], 
#                     structure_y - yy[0], tf_broadcaster)

def simulate(structures, trajectory_function, 
        t_step=0.01, speed=1, loc=[1., .0, .0], 
        waypts=None, figind=1, filesuffix=""):
    global dislocation_srv, thrust_newtons, roll, pitch, yaw
    global thrust_newtons, roll, pitch, yaw

    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]#, robot_id2] #[robot_id1, robot_id2, robot_id3, robot_id4]

    state_log = []

    init_x = rospy.get_param('~init_x', 0.)
    init_y = rospy.get_param('~init_y', 0.)
    init_z = rospy.get_param('~init_z', 0.)
    demo_trajectory = rospy.get_param('~demo_trajectory', True)

    odom_topic = rospy.get_param('~odom_topic', '/odom')  # '/odom2'
    pos_topic = rospy.get_param('world_pos_topic', '/world_pos')  

    state_vector = init_state(loc, 0)
    for s in structures:
        s.state_vector = init_state(s.traj_vars.waypts[0,:],0)

    # Odom publisher
    odom_publishers = {'modquad{:02d}'.format(id_robot+1): rospy.Publisher(
        '/modquad{:02d}'.format(id_robot+1) + odom_topic, 
        Odometry, queue_size=0) for id_robot in range(num_mod)}

    struc_mgr = StructureManager(structures)

    pos_publishers = {id_robot: 
        rospy.Publisher('/' + id_robot + pos_topic, Odometry, queue_size=0) 
        for struc in struc_mgr.strucs for id_robot in struc.ids}

    # TF publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    tmax = max([s.traj_vars.total_dist / speed for s in structures])
    overtime = 1.5
    freq = 100  # 100hz
    rate = rospy.Rate(freq)
    t = 0
    while not rospy.is_shutdown() and t < overtime*tmax + 1.0 / freq:
        rate.sleep()
        t += 1. / freq
        struc_mgr.control_step(t, trajectory_function, speed, 
                odom_publishers, pos_publishers, tf_broadcaster)
    struc_mgr.make_plots()

def setup_and_run(speed=1.0):
    global num_mod
    trajectory_function = min_snap_trajectory
    radius = 2.5
    speed = 2.0
    trajs = [trajectory_function(0, speed, None, 
                waypt_gen.line( [i,i,0.0],
                    [radius*math.cos(math.pi / 4.0 * i),
                        radius*math.sin(math.pi / 4.0 * i),
                        i+1])) 
                for i in range(num_mod)]

    strucs = [Structure(['modquad{:02d}'.format(i+1)], xx=[0.0], yy=[0.0])
                for i in range(num_mod)]
    for i,struc in enumerate(strucs):
        struc.traj_vars = trajs[i]

    simulate(strucs, trajectory_function, loc=[0,0,0], speed=speed)

if __name__ == '__main__':
    setup_and_run()
