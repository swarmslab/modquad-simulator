#!/usr/bin/env python 

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
from numpy import copy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import sys
import json
from tf.transformations import euler_from_quaternion
import math

from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import circular_trajectory, simple_waypt_trajectory, \
    min_snap_trajectory

from modsim import params
from modsim.attitude import attitude_controller, enable_attitude_i_gain
# from modsim.plot.drawer_vispy import Drawer

from modsim.datatype.structure import Structure

# Functions to publish odom, transforms, and acc
from modsim.util.comm import publish_acc, publish_odom, \
                             publish_transform_stamped, \
                             publish_odom_relative,     \
                             publish_transform_stamped_relative, \
                             publish_structure_acc,     \
                             publish_acc_for_attached_mods, \
                             publish_structure_odometry, \
                             publish_odom_for_attached_mods

from modsim.util.state import init_state, state_to_quadrotor
from modquad_simulator.srv import Dislocation, DislocationResponse
from modsim.simulation.ode_integrator import simulation_step

from modsim.util.thrust import convert_thrust_pwm_to_newtons
from modsim.util.flight import sim_takeoff, sim_land

# Fault detection functions
from modsim.util.fault_detection import fault_exists,               \
                                        fault_exists_real,          \
                                        get_faulty_quadrant_rotors, \
                                        update_ramp_rotors,         \
                                        update_ramp_factors,        \
                                        form_groups,                \
                                        update_rotmat

from modsim.util.fault_injection import inject_faults

from modquad_sched_interface.interface import convert_modset_to_struc, \
                                              convert_struc_to_mat   , \
                                              rotpos_to_mat

import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

#from scheduler.gsolver import gsolve
from modquad_sched_interface.simple_scheduler import lin_assign

fig = plt.figure()
fig2 = plt.figure()
fig.patch.set_facecolor('#E0E0E0')
fig.patch.set_alpha(0.7)
fig2.patch.set_facecolor('#E0E0E0')
fig2.patch.set_alpha(0.7)

def recompute_velocities(new_state, old_state, dt):
    state_vec = new_state

    # Instantaneous linear velocities
    vels = (new_state[:3] - old_state[:3]) / (0.0+dt)
    state_vec[3] = vels[0]
    state_vec[4] = vels[1]
    state_vec[5] = vels[2]

    # Instantaneous angular velocities
    prev_angs = euler_from_quaternion(old_state[6:10])
    new_angs  = euler_from_quaternion(new_state[6:10])

    prev_angs = np.array([prev_angs[0], prev_angs[1], prev_angs[2]])
    new_angs = np.array([new_angs[0], new_angs[1], new_angs[2]])

    vels = (new_angs - prev_angs) / (0.0 + dt)

    state_vec[-3] = vels[0]
    state_vec[-2] = vels[1]
    state_vec[-1] = vels[2]

    return state_vec

def simulate(structure, trajectory_function, sched_mset, speed=1, figind=1):
    global faulty_rots, fmod, frot

    rospy.init_node('modrotor_simulator', anonymous=True)
    params.init_params(speed, is_sim=True)

    state_log = []
    forces_log = []
    pos_err_log = [0,0,0]

    odom_topic = rospy.get_param('~odom_topic', '/odom')  
    imu_topic  = rospy.get_param('~imu_topic', '/imu')  

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
            rospy.Publisher('/' + id_robot + odom_topic, Odometry, queue_size=10) 
            for id_robot in structure.ids}

    # TF publisher
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # Logging
    tlog = []
    desired_cmd_log = []
    M_log = []
    single_log = []
    struct_log = []
    desx = []
    desy = []
    desz = []

    # Params tracked during sim
    freq = 100  # 100hz
    rate = rospy.Rate(freq)
    t = 0
    ind = 0.0

    # Params for fault tracking and IDing
    ramp_rotor_set  = [[], []]
    ramp_factors    = [1, 0]
    diagnose_mode   = False
    faults_injected = False
    residual        = []
    quadrant        = []
    rotmat          = []
    groups          = []
    inject_time     = 0

    thrust_newtons, roll, pitch, yaw = 0.0, 0.0, 0.0, 0.0

    enable_attitude_i_gain()
    sim_takeoff(structure, freq, odom_publishers, tf_broadcaster)

    while not rospy.is_shutdown() and t < 30: #overtime*tmax + 1.0 / freq:

        # Publish odometry
        publish_structure_odometry(structure, odom_publishers, tf_broadcaster)

        desired_state = trajectory_function(t, speed, structure.traj_vars)
        desx.append(desired_state[0][0])
        desy.append(desired_state[0][1])
        desz.append(desired_state[0][2])

        # Compute control inputs
        [thrust_pwm, roll, pitch, yawrate] = \
                position_controller(structure, desired_state, 1.0 / freq)
        yaw_des = 0 # Currently unchangeable for trajectories we run
        thrust_newtons = convert_thrust_pwm_to_newtons(thrust_pwm)

        # Control output based on crazyflie input
        F_single, M_single = attitude_controller( structure, 
                    (thrust_newtons, roll, pitch, yawrate), 
                    yaw_des)

        en_motor_sat = True

        # Control of Moments and thrust with rotor faults DISabled
        en_motor_fail= False
        F_structure_est, M_structure_est, rotor_forces_est = \
                modquad_torque_control( F_single, M_single, 
					structure, en_motor_sat, en_motor_fail, fail_type=2)

        # No noise added to estimate because this assumes "perfectness"
        # TODO: Verify this is reasonable, I think it is but want to be sure

        # Simulate, obtain new state and state derivative assuming no failures
        est_state_vector = simulation_step( structure, 
                                            structure.state_vector, 
		                                    F_structure_est, 
                                            M_structure_est, 
                                            1.0 / freq             )

        # Control of Moments and thrust with rotor faults ENabled
        en_motor_fail= True
        F_structure, M_structure, rotor_forces = \
                modquad_torque_control( F_single, M_single, 
					structure, en_motor_sat, en_motor_fail, fail_type=2)

        # Inject some noise to rotor operation to make more realistic
        F_structure += np.random.normal(loc=0, scale=0.25,
                                        size=F_structure.shape)

        # Simulate, obtain new state and state derivative with failures
        structure.state_vector = simulation_step( structure, 
                                            structure.state_vector, 
		                                    F_structure, 
                                            M_structure, 
                                            1.0 / freq             )

        # Compute residual
        residual = np.array(desired_state[0]) - structure.state_vector[:3]
                    # - est_state_vector

        #if inject_time > 0 and t - inject_time > 1.5:
        #    import pdb; pdb.set_trace()

        # Check for faults
        if fault_exists(residual[:3]) and not diagnose_mode:
            rospy.loginfo("Fault detected, enter diagnosis mode")
            diagnose_mode = True
            quadrant = get_faulty_quadrant_rotors(residual, structure)
            rotmat = rotpos_to_mat(structure, quadrant)
            groups = form_groups(quadrant, rotmat)
            rospy.loginfo("Groups = {}".format(groups))
            next_diag_t = 0

        # Store data
        single_log.append([F_single, M_single[0], M_single[1], M_single[2]])
        struct_log.append([F_structure, M_structure[0], M_structure[1], M_structure[2]])
        pos_err_log += np.power(desired_state[0] - structure.state_vector[:3], 2)
        tlog.append(t)
        state_log.append(np.copy(structure.state_vector))
        desired_cmd_log.append([thrust_newtons, roll, pitch, yawrate])
        M_log.append(M_structure)
        forces_log.append(rotor_forces)
        ind += 1.0

        # Check if we need to inject faults
        if ( t >= 8.0 and not faults_injected ):
            max_faults = 1
            faulty_rots = inject_faults(structure, max_faults, 
                                        sched_mset, faulty_rots,
                                        fmod, frot)
            faults_injected = True
            inject_time = t
            #print("Residual = {}".format(residual))

        # Sleep so that we can maintain a 100 Hz update rate
        rate.sleep()
        t += 1. / freq

    sim_land(structure, freq, odom_publishers, tf_broadcaster)

    rospy.loginfo("PLOTTING")

    traj_vars = structure.traj_vars 
    pos_err_log /= ind
    pos_err_log = np.sqrt(pos_err_log)
    integral_val = np.sum(np.array(forces_log) ** 2) * (1.0 / freq)

    single_log = np.array(single_log)
    struct_log = np.array(struct_log)

    if figind < 1:
        return integral_val

    state_log = np.array(state_log)
    ax.plot(state_log[:, 0], state_log[:, 1], state_log[:, 2], 
            zdir='z', color='r', linewidth=lw)
    ax.legend(["Desired", "Actual"])
    plt.sca(fig.gca())

    waypt_time_step = 1.0
    tstate = np.arange(0, tmax + 1.0/freq, 1.0/freq)
    twaypt = np.arange(0, tmax + waypt_time_step, waypt_time_step)

    # Generate legend
    legend_vals = ["Actual", "Desired"]
    # Shrink current axis's height by 10% on the bottom
    ax2 = plt.gca()
    box = ax.get_position()
    ax2.set_position([box.x0, box.y0 + box.height * 0.1,
                         box.width, box.height * 0.9])

    ylabelsize = 12
    # Plot first one so that we can do legend
    plt.subplot(4,2,figind+0)
    plt.plot(tlog, state_log[:, 0], color='r', linewidth=lw)
    plt.plot(tlog, desx, alpha=alphaset, color='g', linewidth=lw)
    plt.axvline(inject_time, color='grey')
    plt.ylabel("X (m)", size=ylabelsize)
    plt.gca().set_ylim(xmin, xmax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    # Put a legend below current axis
    plt.figlegend(legend_vals, loc='upper center', ncol=2)

    plt.subplot(4,2,figind+2)
    plt.plot(tlog, state_log[:, 1], color='r', linewidth=lw)
    plt.plot(tlog, desy, alpha=alphaset, color='g', linewidth=lw)
    plt.axvline(inject_time, color='grey')
    plt.ylabel("Y (m)", size=ylabelsize)
    plt.gca().set_ylim(ymin, ymax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    plt.subplot(4,2,figind+4)
    plt.plot(tlog, state_log[:, 2], color='r', linewidth=lw)
    plt.plot(tlog, desz, alpha=alphaset, color='g', linewidth=lw)
    plt.axvline(inject_time, color='grey')
    plt.ylabel("Z (m)", size=ylabelsize)
    plt.gca().set_ylim(zmin, zmax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    # sum of the squared forces
    ax = plt.subplot(4,2,figind+6)
    ax.set_xlabel("Time (sec)")
    ax.set_ylabel("Force (N)", size=ylabelsize)
    ax.plot(tlog, np.array(forces_log) ** 2, color='r', linewidth=lw)
    ax.axvline(inject_time, color='grey')
    ax.set_ylim(0, 0.10)
    ax.grid()

    desired_cmd_log = np.array(desired_cmd_log)
    atts = [euler_from_quaternion(e) for e in state_log[:, 6:10]]
    atts = np.array(
            [ [math.degrees(e[0]), math.degrees(e[1]), math.degrees(e[2])]
             for e in atts]
           )
    M_log = np.array(M_log)

    # thrust
    ax_t = ax.twinx()
    ax_t.plot(tlog, desired_cmd_log[:, 0], 'c')
    ax_t.set_ylabel(r"Thrust (N) cyan")

    # roll
    ax2 = plt.subplot(4,2,figind+1)
    ax2.set_ylabel(r"$\phi$ (deg)", size=ylabelsize)
    ax2.plot(tlog, atts[:, 0], color='r', linewidth=lw)
    ax2.plot(tlog, desired_cmd_log[:, 1], color='g', linewidth=lw)
    ax2.axvline(inject_time, color='grey')
    ax2.set_ylim(-10, 10)
    ax2.grid()

    # Mx - moment about x axis, i.e., for pitch because of CF frame
    ax3 = ax2.twinx()
    ax3.plot(tlog, M_log[:, 0], 'c')
    ax3.set_ylabel(r"$M_x$ (N.m) cyan")

    # pitch
    ax0 = plt.subplot(4,2,figind+3)
    ax0.set_ylabel(r"$\theta$ (deg)", size=ylabelsize)
    ax0.plot(tlog, atts[:, 1], color='r', linewidth=lw)
    ax0.plot(tlog, desired_cmd_log[:, 2], color='g', linewidth=lw)
    ax0.axvline(inject_time, color='grey')
    ax0.set_ylim(-10, 10)
    ax0.grid()

    # My - moment about y axis, i.e., for pitch
    ax1 = ax0.twinx()
    ax1.plot(tlog, M_log[:, 1], 'c')
    ax1.set_ylabel(r"$M_y$ (N.m) cyan")

    # Yaw Rate
    ax4 = plt.subplot(4,2,figind+5)
    ax4.set_ylabel(r"$\dot{\psi}$ (deg/s)", size=ylabelsize)
    ax4.plot(tlog, state_log[:, -1], color='r', linewidth=lw)
    ax4.plot(tlog, desired_cmd_log[:, 3], color='g', linewidth=lw)
    ax4.axvline(inject_time, color='grey')
    ax4.grid()

    # Mz
    ax5 = ax4.twinx()
    ax5.plot(tlog, M_log[:, 2], 'c')
    ax5.set_ylabel(r"$M_z (N.m)$ cyan")

    # Single and Structure plots
    ax6 = plt.subplot(4,2,figind+7)
    ax6.plot(tlog, single_log[:, 0], 'c', linewidth=lw)
    ax6.plot(tlog, struct_log[:, 0], 'c--', linewidth=lw)
    ax6.set_ylabel(r"$F (N)$ cyan")

    # Mz
    ax7 = ax6.twinx()
    ax7.set_xlabel("Time (sec)")
    ax7.set_ylabel(r"$M$ (N.m)", size=ylabelsize)
    ax7.plot(tlog, single_log[:, 1], color='b', linewidth=lw)
    ax7.plot(tlog, single_log[:, 2], color='m', linewidth=lw)
    ax7.plot(tlog, struct_log[:, 1], 'b--', linewidth=lw)
    ax7.plot(tlog, struct_log[:, 2], 'm--', linewidth=lw)
    ax7.axvline(inject_time, color='grey')
    ax7.grid()


    plt.show()

    plt.clf() # Clear figures
    try:
        plt.sca(ax)
        plt.clf()
    except:
        pass

    return integral_val, pos_err_log

def test_shape_with_waypts(mset, wayptset, speed=1):
    # Initialize the min snap trajectory
    trajectory_function = min_snap_trajectory
    traj_vars = trajectory_function(0, speed, None, wayptset)
    loc=[0,0,0]
    state_vector = init_state(loc, 0)

    # Generate the structure
    lin_assign(mset)
    struc1 = convert_modset_to_struc(mset)
    struc1.state_vector = state_vector
    struc1.traj_vars = traj_vars

    pi = convert_struc_to_mat(struc1.ids, struc1.xx, struc1.yy)
    print("Structure Used: \n{}".format(pi.astype(np.int64)))

    # Run the simulation
    simulate(struc1, trajectory_function, mset, figind=1, speed=speed)

if __name__ == '__main__':
    global faulty_rots, fmod, frot
    # Hard-coding module 1, rotor 1 to be faulty
    faulty_rots = []
    fmod = 1
    frot = 1
    random.seed(1)
    structure = structure_gen.plus(3, 3)
    waypts = waypt_gen.helix(radius=0.5, rise=0.6, num_circ=2, start_pt=[0,0,0.5])
    #waypts = waypt_gen.hover_line(rise=0.5)
    results = test_shape_with_waypts( structure, waypts, speed=0.10 )
    print("---------------------------------------------------------------")
