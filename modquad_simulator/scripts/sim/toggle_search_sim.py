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
                                        update_rotmat

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

faulty_rots = []

fmod = sys.argv[1]
frot = sys.argv[2]

def simulate(structure, trajectory_function, sched_mset,
        t_step=0.01, speed=1, figind=1, 
        filesuffix="", max_faults=1):

    global faulty_rots

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)

    # So that modquad_torque_control knows which mapping to use
    rospy.set_param('rotor_map', 2) 

    rospy.init_node('modrotor_simulator', anonymous=True)
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

    can_pub_imu = False
    faults_injected = False
    freq = 100  # 100hz
    rate = rospy.Rate(freq)
    t = 0
    ind = 0.0

    est = []
    isForStateEst = True

    # Whether we are diagnosing a fault
    diagnose_mode = False

    # ramp_mode ranges [-1, 1] - sign indicates ramp down (-) or up (+)
    # Used to compute ramp_factor
    ramp_mode = -1 # Init to ramping down, will grow to 0

    # The time at which to perform next diagnostic check
    next_diag_t  = 0

    # Which rotor we are testing currently
    quadrant_idx = 0
    
    # List of tuples of (mod_id, rot_id) rotors where fault is
    quadrant = []
    rotmat = []
    groups = []

    # Rotor ramping variables for fault detection
    ramp_rotor_set = [[], []]
    ramp_factor = [1, 0]

    # Ramp up times
    rospy.set_param("fault_det_time_interval", 5.0)

    fdd_interval = rospy.get_param("fault_det_time_interval")

    residual = []

    thrust_newtons, roll, pitch, yaw = 0.0, 0.0, 0.0, 0.0

    #while not rospy.is_shutdown() or t < overtime*tmax + 1.0 / freq:
    while t < overtime*tmax + 1.0 / freq:
        t += 1. / freq
        if ( t % 10 < 0.01 ):
            print("{:.01f} / {:.01f} - Residual = {}".format(t, overtime*tmax, residual))

        # Publish odometry
        #import pdb; pdb.set_trace()
        publish_structure_odometry(structure, odom_publishers, tf_broadcaster)

        desired_state = trajectory_function(t, speed, structure.traj_vars)

        #if demo_trajectory:
        # Overwrite the control input with the demo trajectory
        [thrust_pwm, roll, pitch, yaw] = \
                position_controller(structure, desired_state, 1.0 / freq)

        thrust_newtons = 6.0 * convert_thrust_pwm_to_newtons(thrust_pwm)
        rospy.loginfo('thrust_newtons = {}'.format(thrust_newtons))

        # Control output based on crazyflie input
        F_single, M_single = \
                attitude_controller(structure, (thrust_newtons, roll, pitch, yaw))

        pos_err_log += np.power(desired_state[0] - structure.state_vector[:3], 2)

        en_motor_sat = True
        en_fail_rotor_act = True

        # Control of Moments and thrust
        F_structure, M_structure, rotor_forces = \
                modquad_torque_control(
                        F_single, M_single, structure,
                        en_motor_sat, en_fail_rotor_act, 
                        ramp_rotor_set, ramp_factor)

        # Add noise to F_structure, M_structure
        #print(F_structure)
        #F_structure += np.random.normal(loc=0, scale=0.025, size=F_structure.shape)
        #M_structure += np.random.normal(loc=0, scale=0.025, size=F_structure.shape)

        en_fail_rotor_act = False

        # Compute in case of no failures for state estimation
        F_structure_est, M_structure_est, rotor_forces_est = \
                modquad_torque_control(
                        F_single, M_single, structure,
                        en_motor_sat, en_fail_rotor_act, 
                        ramp_rotor_set, ramp_factor)

        # Perform state estimation for next time step
        est = simulation_step(structure, structure.state_vector, 
		                      F_structure_est, M_structure_est, 1. / freq)

        # The sleep is to make the simulation look like it would in real life
        rate.sleep()

        # Simulate, obtain new state and state derivative
        structure.state_vector = simulation_step( structure, 
                                                  structure.state_vector, 
		                                          F_structure, 
                                                  M_structure, 
                                                  1.0 / freq             )

        # Compute residuals for error detection
        residual = structure.state_vector - est

        # Process the residual - i.e. check for failed rotors via thresholding
        # of residuals
        if fault_exists(residual) and not diagnose_mode:
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
                if (abs(residual[-3] < 0.05) and abs(residual[-2]) < 0.05):
                    print("State Est = {}".format(est))
                    print("Residual = {}".format(residual[-3:-1]))

                    # Recurse over set if not already single rotor
                    if (len(ramp_rotor_set[0]) == 1): # Single rotor
                        print("The faulty rotor is {}".format(ramp_rotor_set[0]))
                        break
                        sys.exit(0)

                    print("The faulty rotor is in set {}".format(ramp_rotor_set[0]))

                    rotmat = update_rotmat(ramp_rotor_set[0], rotmat)

                    # Form smaller subgroups
                    groups = form_groups(ramp_rotor_set[0], rotmat)
                    quadrant_idx = 0 # Reset
                    print("New Groups: {}".format(groups))
                    ramp_rotor_set = [[], ramp_rotor_set[0]]
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

        # Store data
        state_log.append(np.copy(structure.state_vector))
        forces_log.append(rotor_forces)
        ind += 1.0

        # Inject faults
        if ( t >= tmax/10.0 and not faults_injected ):
            faulty_rots = inject_faults(structure, max_faults, 
                                        sched_mset, faulty_rots,
                                        fmod, frot)
            faults_injected = True
            print("Residual = {}".format(residual))

        # Sleep so that we can maintain a 100 Hz update rate
        rate.sleep()

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

    plt.show()

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

    # Generate the structure
    #gsolve(mset, waypts=traj_vars.waypts)
    lin_assign(mset)
    struc1 = convert_modset_to_struc(mset)
    struc1.state_vector = state_vector
    struc1.traj_vars = traj_vars

    #print("Starting fault inject exp with no faults.")
    pi = convert_struc_to_mat(struc1.ids, struc1.xx, struc1.yy)
    print("Structure Used: \n{}".format(pi.astype(np.int64)))
    #print("Mset:\n{}".format(mset.pi))

    if doreform:
        forces, pos_err = simulate(struc1, trajectory_function, mset, 
                figind=1, speed=speed, 
                filesuffix="{}_f{}_reform".format(test_id, max_fault), 
                max_faults=max_fault)
    else:
        forces, pos_err = simulate(struc1, trajectory_function, mset,
                figind=1, speed=speed, 
                filesuffix="{}_f{}_noreform".format(test_id, max_fault),
                max_faults=max_fault)

    #print(struc1.ids)
    #print(struc1.xx)
    #print(struc1.yy)
    return [forces, pos_err, mset.pi]

if __name__ == '__main__':
    print("starting simulation")
    #print(structure_gen.airplane(5,5,3).struc)
    #sys.exit(0)
    rospy.set_param("fdd_group_type", "log4")
    random.seed(1)
    results = test_shape_with_waypts(
                       #structure_gen.zero(4, 4), 
                       #structure_gen.plus(2, 1), 
                       structure_gen.rect(2, 2), 
                       #structure_gen.airplane(5,5,3),
                       waypt_gen.helix(radius=2.5, rise=1, num_circ=2),
                       #waypt_gen.line([0,0,0],[1,1,1]),
                       speed=2.0, test_id="4x4rect_2.5x1x2helix", 
                       doreform=True, max_fault=1, rand_fault=False)
    #print("Force used: {}".format(results[0]))
    #print("RMSE Position Error: {}".format(np.mean(results[1])))
    #print("Faults: {}".format(faulty_rots))
    print("---------------------------------------------------------------")
