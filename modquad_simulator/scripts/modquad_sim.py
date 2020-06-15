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

from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import circular_trajectory, simple_waypt_trajectory, \
    min_snap_trajectory

from modsim import params
from modsim.attitude import attitude_controller
# from modsim.plot.drawer_vispy import Drawer

from modsim.datatype.structure import Structure

from modsim.util.comm import publish_odom, publish_transform_stamped, publish_odom_relative, \
    publish_transform_stamped_relative
from modsim.util.state import init_state, state_to_quadrotor
from modquad_simulator.srv import Dislocation, DislocationResponse
from modsim.simulation.ode_integrator import simulation_step

from modquad_sched_interface.interface import convert_modset_to_struc, convert_struc_to_mat
import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

from scheduler.gsolver import gsolve

fig = plt.figure()
fig2 = plt.figure()

## Control Input
#thrust_newtons, roll, pitch, yaw = 0., 0., 0., 0.
#
#dislocation_srv = (0., 0.)


## Control input callback
#def control_input_listener(twist_msg):
#    global thrust_newtons, roll, pitch, yaw
#    # For more info, check:
#    # https://github.com/whoenig/crazyflie_ros
#    roll = twist_msg.linear.y
#    pitch = twist_msg.linear.x
#    yaw = twist_msg.angular.z
#    thrust_pwm = twist_msg.linear.z
#
#    c1, c2, c3 = -0.6709, 0.1932, 13.0652
#    F_g = ((thrust_pwm / 60000. - c1) / c2) ** 2 - c3  # Force in grams
#    if F_g<0:
#        F_g = 0
#
#    thrust_newtons = 9.81 * F_g / 1000.  # Force in Newtons

def publish_for_attached_mods(robot_id, structure_x, structure_y, xx, yy, main_id, odom_publishers, tf_broadcaster):
    publish_odom_relative(structure_x - xx[0], structure_y - yy[0], robot_id, main_id, odom_publishers[robot_id])
    publish_transform_stamped_relative(robot_id, main_id, structure_x - xx[0], structure_y - yy[0], tf_broadcaster)

def publish_structure_odometry(structure, odom_publishers, tf_broadcaster):
    ids, xx, yy, x = structure.ids, structure.xx, structure.yy, structure.state_vector

    main_id = ids[0]
    publish_odom(x, odom_publishers[main_id])
    publish_transform_stamped(main_id, x, tf_broadcaster)

    # show the other robots
    [publish_for_attached_mods(robot_id, structure_x, structure_y, xx, yy,
        main_id, odom_publishers, tf_broadcaster)
        for robot_id, structure_x, structure_y in list(zip(ids, xx, yy))[1:]]

def simulate(structure, trajectory_function, 
        t_step=0.01, speed=1, figind=1, filesuffix=""):

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    print("Speed = {}".format(speed))
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use
    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]

    state_log = []
    forces_log = []
    pos_err_log = [0,0,0]

    demo_trajectory = rospy.get_param('~demo_trajectory', True)

    odom_topic = rospy.get_param('~odom_topic', '/odom')  

    tmax = structure.traj_vars.total_dist / speed

    # Plot bounds
    xmin = np.min(structure.traj_vars.waypts[:, 0])-3
    xmax = np.max(structure.traj_vars.waypts[:, 0])+3
    ymin = np.min(structure.traj_vars.waypts[:, 1])-3
    ymax = np.max(structure.traj_vars.waypts[:, 1])+3
    zmin = np.min(structure.traj_vars.waypts[:, 2])-3
    zmax = np.max(structure.traj_vars.waypts[:, 2])+3

    # Plotting coeffs
    overtime = 1.0
    lw=3
    alphaset = 0.8

    # 3D plot setup
    ax = fig2.add_subplot(1,1,1, projection='3d')
    ax.plot(structure.traj_vars.waypts[:,0], 
	    structure.traj_vars.waypts[:,1], 
	    structure.traj_vars.waypts[:,2], 
	    zdir='z', color='b', linewidth=lw, dashes=[3, 3])
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_zlim(zmin, zmax)

    # Odom publisher
    odom_publishers = {id_robot: 
            rospy.Publisher('/' + id_robot + odom_topic, Odometry, queue_size=0) 
            for id_robot in structure.ids}

    # TF publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    freq = 100  # 100hz
    rate = rospy.Rate(freq)
    t = 0
    ind = 0.0
    #while not rospy.is_shutdown() or t < overtime*tmax + 1.0 / freq:
    while t < overtime*tmax + 1.0 / freq:
        rate.sleep()
        t += 1. / freq
        print("{} / {}".format(t, overtime*tmax))

        # Publish odometry
        publish_structure_odometry(structure, odom_publishers, tf_broadcaster)

        desired_state = trajectory_function(t, speed, structure.traj_vars)
        if demo_trajectory:
            # Overwrite the control input with the demo trajectory
            [thrust_newtons, roll, pitch, yaw] = \
                    position_controller(structure, desired_state)

        # Control output based on crazyflie input
        F_single, M_single = \
                attitude_controller(structure, (thrust_newtons, roll, pitch, yaw))

        pos_err_log += np.power(desired_state[0] - structure.state_vector[:3], 2)

        # Control of Moments and thrust
        F_structure, M_structure, rotor_forces = \
                modquad_torque_control(F_single, M_single, structure, motor_sat=True)

        # Simulate
        structure.state_vector = simulation_step(structure, structure.state_vector, 
		F_structure, M_structure, 1. / freq)

        # Store data
        state_log.append(np.copy(structure.state_vector))
        forces_log.append(rotor_forces)
        ind += 1.0
        #print(rotor_forces)

    traj_vars = structure.traj_vars 
    pos_err_log /= ind
    pos_err_log = np.sqrt(pos_err_log)
    integral_val = np.sum(np.array(forces_log) ** 2) * (1.0 / freq)
    print("Final position = {}".format(structure.state_vector[:3]))

    if figind < 1:
        print("total integral={}".format(integral_val))
        return integral_val
    #ax.grid()

    state_log = np.array(state_log)
    ax.plot(state_log[:, 0], state_log[:, 1], state_log[:, 2], 
            zdir='z', color='r', linewidth=lw)
    ax.legend(["Planned Path", "Actual Path"])
    plt.savefig("figs/3d_{}.pdf".format(filesuffix))
    plt.sca(fig.gca())

    waypt_time_step = 1.0
    tstate = np.arange(0, tmax + 1.0/freq, 1.0/freq)
    twaypt = np.arange(0, tmax + waypt_time_step, waypt_time_step)

    # Generate legend
    legend_vals = ["Actual path", "Desired path"]
    # Shrink current axis's height by 10% on the bottom
    ax2 = plt.gca()
    box = ax.get_position()
    ax2.set_position([box.x0, box.y0 + box.height * 0.1,
                         box.width, box.height * 0.9])

    ylabelsize = 12
    # Plot first one so that we can do legend
    plt.subplot(4,1,figind+0)
    plt.plot(tstate, state_log[:, 0], color='r', linewidth=lw)
    plt.plot(twaypt, traj_vars.waypts[:,0], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("X position\n(m)", size=ylabelsize)
    plt.gca().set_ylim(xmin, xmax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    # Put a legend below current axis
    plt.figlegend(legend_vals, loc='upper center', ncol=2)#bbox_to_anchor=(0.5,  0.95),
                      #fancybox=True, shadow=True, ncol=2)

    plt.subplot(4,1,figind+1)
    plt.plot(tstate, state_log[:, 1], color='r', linewidth=lw)
    plt.plot(twaypt, traj_vars.waypts[:,1], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("Y position\n(m)", size=ylabelsize)
    plt.gca().set_ylim(ymin, ymax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    plt.subplot(4,1,figind+2)
    plt.plot(tstate, state_log[:, 2], color='r', linewidth=lw)
    plt.plot(twaypt, traj_vars.waypts[:,2], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("Z position\n(m)", size=ylabelsize)
    plt.gca().set_ylim(zmin, zmax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    # sum of the squared forces
    plt.subplot(4,1,figind+3)
    plt.xlabel("Time (sec)")
    plt.ylabel("Force\n(N)", size=ylabelsize)
    #forces_log = forces_log[5:]
    #plt.plot(tstate[5:], np.sum(np.array(forces_log) ** 2, axis=1), color='r', linewidth=lw)
    plt.plot(tstate[5:], np.array(forces_log[5:]) ** 2, color='r', linewidth=lw)
    plt.gca().set_ylim(0, 0.10)
    plt.grid()
    #strftime("%Y-%m-%d_%H:%M:%S", localtime()), 
    plt.savefig("figs/2d_{}.pdf".format(filesuffix))
    #print("total integral={}".format(np.sum(np.array(forces_log) ** 2) * t_step))
    plt.clf() # Clear figures
    plt.sca(ax)
    plt.clf()
    return integral_val, pos_err_log

def test_shape_with_waypts(mset, wayptset, speed=1, test_id="", 
        doreform=False, max_fault=1, rand_fault=False):
    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)
    loc=[0,0,0]
    state_vector = init_state(loc, 0)

    # Run Set 1
    if not doreform:
        gsolve(mset, waypts=traj_vars.waypts)

    faulty_rots = []
    random.seed(0)
    num_faults = 0
    while num_faults < max_fault:
        if rand_fault:
            newfault = (random.randint(0,mset.num_mod-1), random.randint(0,3))
            if newfault not in faulty_rots:
                faulty_rots.append(newfault)
                num_faults += 1	
        else:
            if num_faults < 4:
                newfault = (0, num_faults)
                faulty_rots.append(newfault)
            else:
                newfault = (1, num_faults % 4)
                faulty_rots.append(newfault)
        num_faults += 1

    print(faulty_rots)
    for f in faulty_rots:
        mset.fault_rotor(f[0], f[1])
    faulty_rots = [(f[0]+1, f[1]) for f in faulty_rots]

    if doreform:
        gsolve(mset, waypts=traj_vars.waypts)

    struc1 = convert_modset_to_struc(mset)
    struc1.state_vector = state_vector
    struc1.traj_vars = traj_vars
    print("Running test with faults: {} and reform: {}".format(struc1.motor_failure, doreform))
    pi = convert_struc_to_mat(struc1.ids, struc1.xx, struc1.yy)
    print("Structure Used: \n{}".format(pi.astype(np.int64)))

    if doreform:
        forces, pos_err = simulate(struc1, trajectory_function, figind=1,
                speed=speed, filesuffix="{}_f{}_reform".format(test_id, num_faults))
    else:
        forces, pos_err = simulate(struc1, trajectory_function, figind=1,
                speed=speed, filesuffix="{}_f{}_noreform".format(test_id, num_faults))
    return [forces, pos_err, mset.pi, faulty_rots]

if __name__ == '__main__':
    print("starting simulation")
    #print(structure_gen.airplane(5,5,3).struc)
    #sys.exit(0)
    results = test_shape_with_waypts(
                       #structure_gen.zero(4, 4), 
                       #structure_gen.plus(2, 1), 
                       structure_gen.rect(2, 2), 
                       #structure_gen.airplane(5,5,3),
                       #waypt_gen.helix(radius=2.5, rise=3, num_circ=2),
                       waypt_gen.line([0,0,0],[1,1,1]),
                       speed=1.25, test_id="2x1line", 
                       doreform=True, max_fault=0, rand_fault=False)
    print("Force used: {}".format(results[0]))
    print("RMSE Position Error: {}".format(np.mean(results[1])))
    #print("Faults: {}".format(results[3]))
