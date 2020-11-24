#!/usr/bin/env python 

from __future__ import print_function
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
from modsim.attitude import attitude_controller
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

# Fault detection functions
from modsim.util.fault_detection import fault_exists,               \
                                        get_faulty_quadrant_rotors, \
                                        update_ramp_rotors,         \
                                        update_ramp_factors,        \
                                        form_groups,                \
                                        update_rotmat,              \
                                        compute_residual

from modsim.util.fault_injection import inject_faults
from modsim.util.thrust import convert_thrust_pwm_to_newtons

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

<<<<<<< HEAD:modquad_simulator/scripts/sim/alt_toggle_search_sim.py
def simulate(structure, trajectory_function, sched_mset,
        t_step=0.01, speed=1, figind=1, 
        filesuffix="", max_faults=1):

    global fmod, frot, faulty_rots, noise, rfname

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    rospy.set_param('is_modquad_sim', True)
=======
def init_params(speed):
    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)

    # So that modquad_torque_control knows which mapping to use
    rospy.set_param('rotor_map', 1) 

    rospy.set_param('is_modquad_sim', True)

def recompute_velocities(new_state, old_state, dt):
    state_vec = new_state

    # Instantaneous linear velocities
    vels = (new_state[:3] - old_state[:3]) / (0.0+dt)
    state_vec[3] = vels[0]
    state_vec[4] = vels[1]
    state_vec[5] = vels[2]
>>>>>>> origin/modulardynamics_python2:modquad_simulator/scripts/sim/simple_traj_sim.py

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

def simulate(structure, trajectory_function, sched_mset,
        t_step=0.01, speed=1, figind=1, 
        filesuffix="", max_faults=1):

    rospy.init_node('modrotor_simulator', anonymous=True)
    init_params(speed)

    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]

    state_log = []
    forces_log = []
    pos_err_log = [0,0,0]

    demo_trajectory = rospy.get_param('~demo_trajectory', False)

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

<<<<<<< HEAD:modquad_simulator/scripts/sim/alt_toggle_search_sim.py
    can_pub_imu = False
    faults_injected = False
    freq = 300  # 100hz
=======
    freq = 100  # 100hz
>>>>>>> origin/modulardynamics_python2:modquad_simulator/scripts/sim/simple_traj_sim.py
    rate = rospy.Rate(freq)
    t = 0
    ind = 0.0

<<<<<<< HEAD:modquad_simulator/scripts/sim/alt_toggle_search_sim.py
    est = []
    isForStateEst = True

    # Whether we are diagnosing a fault
    diagnose_mode = False

    # ramp_mode ranges [-1, 1] - sign indicates ramp down (-) or up (+)
    # Used to compute ramp_factor
    ramp_mode = -1 # Init to ramping down, will grow to 0

    # The time at which to perform next diagnostic check
    next_diag_t  = 0
    inject_time  = 0
=======
    tlog = []
    desired_cmd_log = []
    M_log = []
>>>>>>> origin/modulardynamics_python2:modquad_simulator/scripts/sim/simple_traj_sim.py

    _takeoff(structure, freq, odom_publishers, tf_broadcaster)

    thrust_newtons, roll, pitch, yaw = 0.0, 0.0, 0.0, 0.0

<<<<<<< HEAD:modquad_simulator/scripts/sim/alt_toggle_search_sim.py
    #while not rospy.is_shutdown() or t < overtime*tmax + 1.0 / freq:
    while not rospy.is_shutdown(): #t < overtime*tmax + 1.0 / freq:
        t += 1. / freq
        if ( t % 10 < 0.01 ):
            print("{:.01f} / {:.01f} - Residual = {}".format(t, overtime*tmax, residual))
=======
    while not rospy.is_shutdown() and t < overtime*tmax + 1.0 / freq:
>>>>>>> origin/modulardynamics_python2:modquad_simulator/scripts/sim/simple_traj_sim.py

        # Publish odometry
        publish_structure_odometry(structure, odom_publishers, tf_broadcaster)

        desired_state = trajectory_function(t, speed, structure.traj_vars)

<<<<<<< HEAD:modquad_simulator/scripts/sim/alt_toggle_search_sim.py
        # Overwrite the control input with the demo trajectory
        [thrust_newtons, roll, pitch, yaw] = \
                position_controller(structure, desired_state)
=======
        # Compute control inputs
        [thrust_pwm, roll, pitch, yawrate] = \
                position_controller(structure, desired_state, 1.0 / freq)
        yaw_des = 0 # This is something currently unchangeable for trajectories we are running

        thrust_newtons = convert_thrust_pwm_to_newtons(thrust_pwm)
>>>>>>> origin/modulardynamics_python2:modquad_simulator/scripts/sim/simple_traj_sim.py

        # Control output based on crazyflie input
        F_single, M_single = \
                attitude_controller(structure, (thrust_newtons, roll, pitch, yawrate), yaw_des)

        en_motor_sat = True

        # Control of Moments and thrust
        F_structure, M_structure, rotor_forces = \
<<<<<<< HEAD:modquad_simulator/scripts/sim/alt_toggle_search_sim.py
                modquad_torque_control(
                        F_single, M_single, structure,
                        en_motor_sat, en_fail_rotor_act, 
                        ramp_rotor_set, ramp_factor)

        # The sleep is to make the simulation look like it would in real life
        rate.sleep()

        # Simulate, obtain new state and state derivative
        F_structure  = np.array(F_structure) # Add Gaussian noise
        F_structure += np.random.normal(loc=0, scale=noise, size=F_structure.shape)
        structure.state_vector = simulation_step( structure, 
                                                  structure.state_vector, 
		                                          F_structure, 
                                                  M_structure, 
                                                  1.0 / freq             )

        # Compute residuals for error detection
        des_yaw = 0 # yaw-variable above, confusingly, is actually yaw_rate
        residual = compute_residual(structure.state_vector, desired_state,
                                    roll, pitch, des_yaw)
        #print("Residual = {}".format(residual))

        # Process the residual - i.e. check for failed rotors via thresholding
        # of residuals
        if fault_exists(residual) and not diagnose_mode:
            print("Enter Fault Diagnosis Mode")
            diagnose_mode = True
            quadrant = get_faulty_quadrant_rotors(residual, structure)
            rotmat = rotpos_to_mat(structure, quadrant)
            groups = form_groups(quadrant, rotmat)
            print("Groups = {}".format(groups))
            next_diag_t = 0

        # If we are in the diagnose_mode, then we need to iteratively turn off
        # the rotors in the quadrant and see what state error goes to
        if diagnose_mode:
            if t >= next_diag_t: # Update rotor set
                # We found the faulty rotor
                if (abs(residual[-3] < 0.03) and \
                    abs(residual[-2]) < 0.03) and \
                    len(ramp_rotor_set[0]) > 0:
                #{
                    print("State Est = {}".format(est))
                    print("Residual = {}".format(residual[-3:-1]))

                    # Recurse over set if not already single rotor
                    if (len(ramp_rotor_set[0]) == 1): # Single rotor
                        print("[\N{GREEK CAPITAL LETTER DELTA}t = {:.03f}] Injected ({}, {}), ID'd {}".format(
                                t-inject_time, fmod, frot, ramp_rotor_set[0][0]),
                                file=sys.stderr)
                        with open(rfname, "a+") as f:
                            print("The faulty rotor is {}".format(ramp_rotor_set[0]))
                            f.write("{}-Mod: [\N{GREEK CAPITAL LETTER DELTA}t = {:5.2f}] Inject ({}, {}), ID'd: {} \n".format(
                                    len(structure.xx), t - inject_time, fmod, frot,
                                    ramp_rotor_set[0]) 
                            )
                        sys.exit(0)

                    print("The faulty rotor is in set {}".format(ramp_rotor_set[0]))
                    assert len(ramp_rotor_set[0]) > 1

                    rotmat = update_rotmat(ramp_rotor_set[0], rotmat)

                    # Form smaller subgroups
                    groups = form_groups(ramp_rotor_set[0], rotmat)
                    quadrant_idx = 0 # Reset
                    print("New Groups: {}".format(groups))
                    ramp_rotor_set = [[], ramp_rotor_set[0]]
                #}
                else:
                    ramp_rotor_set, quadrant_idx = \
                                    update_ramp_rotors(
                                            structure,
                                            t, next_diag_t,
                                            groups, quadrant_idx,
                                            rotmat,
                                            ramp_rotor_set)
                next_diag_t = t + fdd_interval
                print("New Ramp Rotor Set = {}".format(ramp_rotor_set))
                print("t = {:03f}, next_check = {:03f}".format(t, next_diag_t))
                print("------------------------------------------------")
            else: # Update ramping factors
                ramp_factor = update_ramp_factors(t, next_diag_t, ramp_factor)
=======
                modquad_torque_control( F_single, M_single, 
					structure, en_motor_sat)

        # Simulate, obtain new state and state derivative
        new_state_vector = simulation_step( structure, 
                                            structure.state_vector, 
		                            F_structure, 
                                            M_structure, 
                                            1.0 / freq             )

        # Compute velocities manually
        structure.state_vector = new_state_vector #\
            #recompute_velocities(new_state_vector, structure.state_vector, 1.0 / freq)
>>>>>>> origin/modulardynamics_python2:modquad_simulator/scripts/sim/simple_traj_sim.py

        # Store data
        pos_err_log += np.power(desired_state[0] - structure.state_vector[:3], 2)
        tlog.append(t)
        state_log.append(np.copy(structure.state_vector))
        desired_cmd_log.append([thrust_newtons, roll, pitch, yawrate])
        M_log.append(M_structure)
        forces_log.append(rotor_forces)
        ind += 1.0

<<<<<<< HEAD:modquad_simulator/scripts/sim/alt_toggle_search_sim.py
        # Inject faults
        if ( t >= tmax/10.0 and not faults_injected ):
            faulty_rots = inject_faults(structure, max_faults, 
                                        sched_mset, faulty_rots,
                                        fmod, frot)
            faults_injected = True
            inject_time = t
            #import pdb; pdb.set_trace()
            print("Residual = {}".format(residual))

=======
>>>>>>> origin/modulardynamics_python2:modquad_simulator/scripts/sim/simple_traj_sim.py
        # Sleep so that we can maintain a 100 Hz update rate
        rate.sleep()
        t += 1. / freq


    rospy.loginfo("PLOTTING")

    traj_vars = structure.traj_vars 
    pos_err_log /= ind
    pos_err_log = np.sqrt(pos_err_log)
    integral_val = np.sum(np.array(forces_log) ** 2) * (1.0 / freq)
    #print("Final position = {}".format(structure.state_vector[:3]))

    if figind < 1:
        #print("total integral={}".format(integral_val))
        return integral_val
    #ax.grid()

    state_log = np.array(state_log)
    ax.plot(state_log[:, 0], state_log[:, 1], state_log[:, 2], 
            zdir='z', color='r', linewidth=lw)
    ax.legend(["Desired", "Actual"])
    #plt.savefig("figs/3d_{}.pdf".format(filesuffix))
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
    plt.plot(twaypt, traj_vars.waypts[:,0], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("X (m)", size=ylabelsize)
    plt.gca().set_ylim(xmin, xmax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    # Put a legend below current axis
    plt.figlegend(legend_vals, loc='upper center', ncol=2)#bbox_to_anchor=(0.5,  0.95),
                      #fancybox=True, shadow=True, ncol=2)

    plt.subplot(4,2,figind+2)
    plt.plot(tlog, state_log[:, 1], color='r', linewidth=lw)
    plt.plot(twaypt, traj_vars.waypts[:,1], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("Y (m)", size=ylabelsize)
    plt.gca().set_ylim(ymin, ymax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    plt.subplot(4,2,figind+4)
    plt.plot(tlog, state_log[:, 2], color='r', linewidth=lw)
    plt.plot(twaypt, traj_vars.waypts[:,2], alpha=alphaset, color='g', linewidth=lw)
    plt.ylabel("Z (m)", size=ylabelsize)
    plt.gca().set_ylim(zmin, zmax)
    plt.gca().xaxis.set_ticklabels([])
    plt.grid()

    # sum of the squared forces
    ax = plt.subplot(4,2,figind+6)
    ax.set_xlabel("Time (sec)")
    ax.set_ylabel("Force (N)", size=ylabelsize)
    ax.plot(tlog, np.array(forces_log) ** 2, color='r', linewidth=lw)
    ax.set_ylim(0, 0.10)
    ax.grid()
    #plt.savefig("figs/2d_{}.pdf".format(filesuffix))

    desired_cmd_log = np.array(desired_cmd_log)
    atts = [euler_from_quaternion(e) for e in state_log[:, 6:10]]
    atts = np.array([[math.degrees(e[0]), math.degrees(e[1]), math.degrees(e[2])] for e in atts])
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
    ax0.set_ylim(-10, 10)
    ax0.grid()

    # My - moment about y axis, i.e., for pitch
    ax1 = ax0.twinx()
    ax1.plot(tlog, M_log[:, 1], 'c')
    ax1.set_ylabel(r"$M_y$ (N.m) cyan")

    # Yaw Rate
    ax4 = plt.subplot(4,2,figind+5)
    ax4.set_xlabel("Time (sec)")
    ax4.set_ylabel(r"$\dot{\psi}$ (deg/s)", size=ylabelsize)
    ax4.plot(tlog, state_log[:, -1], color='r', linewidth=lw)
    ax4.plot(tlog, desired_cmd_log[:, 3], color='g', linewidth=lw)
    ax4.grid()

    # Mz
    ax5 = ax4.twinx()
    ax5.plot(tlog, M_log[:, 2], 'c')
    ax5.set_ylabel(r"$M_z (N.m)$ cyan")

    plt.show()

    plt.clf() # Clear figures
    try:
        plt.sca(ax)
        plt.clf()
    except:
        pass

    return integral_val, pos_err_log

def _takeoff(structure, freq, odom_publishers, tf_broadcaster):
    global start_id

    rate = rospy.Rate(freq)

    # Publish to robot
    msg = Twist()

    # TAKEOFF
    taken_off = False

    # Message init for takeoff
    msg.linear.x  = 0  # pitch [-30, 30] deg
    msg.linear.y  = 0  # roll [-30, 30] deg
    msg.linear.z  = 0  # Thrust ranges 10000 - 60000
    msg.angular.z = 0  # yaw rate
    yaw_des = 0
    pidz_ki = 3500
    rospy.loginfo("Start Control")
    rospy.loginfo("TAKEOFF")
    roll, pitch, yawrate, thrust_pwm = 0,0,0,0
    while not taken_off:
        # Publish odometry
        publish_structure_odometry(structure, odom_publishers, tf_broadcaster)

        if structure.state_vector[2] > 0.05 or msg.linear.z > 50000:
            structure.pos_accumulated_error = msg.linear.z / pidz_ki
            taken_off = True

        # Convert thrust to PWM range
        thrust_pwm += 10000 * (1.0/freq)
        thrust_newtons = convert_thrust_pwm_to_newtons(thrust_pwm)

        # Control output based on crazyflie input
        F_single, M_single = \
                attitude_controller(structure, (thrust_newtons, roll, pitch, yawrate), yaw_des)

        en_motor_sat = True
        en_fail_rotor_act = True

        # Control of Moments and thrust
        F_structure, M_structure, rotor_forces = \
                modquad_torque_control(
                        F_single, M_single, structure,
                        en_motor_sat, en_fail_rotor_act)

        # Simulate, obtain new state and state derivative
        structure.state_vector = simulation_step( structure, 
                                                  structure.state_vector, 
		                                  F_structure, 
                                                  M_structure, 
                                                  1.0 / freq             )

        # The sleep preserves sending rate
        rate.sleep()
    rospy.loginfo("COMPLETED TAKEOFF")


def test_shape_with_waypts(mset, wayptset, speed=1, test_id="", 
        doreform=False, max_fault=1, rand_fault=False):

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

<<<<<<< HEAD:modquad_simulator/scripts/sim/alt_toggle_search_sim.py
    simulate(struc1, trajectory_function, mset, 
             figind=1, speed=speed, 
             filesuffix="{}_f{}_reform".format(test_id, max_fault), 
             max_faults=max_fault)

if __name__ == '__main__':
    global fmod, frot, faulty_rots, noise, rfname

    faulty_rots = []
    fmod = int(sys.argv[1])
    frot = int(sys.argv[2])
    n_idx= int(sys.argv[3])
    ftype= int(sys.argv[4])
    shape= int(sys.argv[5])

    noise_arr = [ 0, 0.25, 0.5, 0.75, 1.0 ]
    noise = noise_arr[n_idx]

    structure = None
    if shape == 1:
        structure = structure_gen.plus(3, 3)
    elif shape == 2:
        structure = structure_gen.zero(3, 3)
    elif shape == 3:
        structure = structure_gen.rect(4, 3)
    else:
        raise Exception("Unknown shape num")

    waypts = waypt_gen.helix(radius=2.5, rise=15, num_circ=15)

    rospy.set_param("fdd_group_type", "log4")
    random.seed(1)

    rfname = "/home/arch/catkin_ws/src/modquad-simulator/" + \
             "modquad_simulator/alt_fdd_toggle_results/"       + \
             "n{:.01f}_fail{}.txt".format(noise, ftype)

    print("starting simulation")
    results = test_shape_with_waypts( structure, waypts,
                       speed=3.5, test_id="3x3zero_2.5x1x2helix", 
=======
    forces, pos_err = simulate(struc1, trajectory_function, mset, 
            figind=1, speed=speed, 
            filesuffix="{}_f{}_reform".format(test_id, max_fault), 
            max_faults=max_fault)

    return [forces, pos_err, mset.pi]

if __name__ == '__main__':
    rospy.loginfo("starting simulation")
    random.seed(1)
    results = test_shape_with_waypts(
                       structure_gen.rect(4, 4), 
                       waypt_gen.helix(radius=0.5, rise=0.6, num_circ=2, start_pt=[0,0,0.5]),
                       speed=0.15, test_id="4x4rect_2.5x1x2helix", 
>>>>>>> origin/modulardynamics_python2:modquad_simulator/scripts/sim/simple_traj_sim.py
                       doreform=True, max_fault=1, rand_fault=False)
    print("---------------------------------------------------------------")
