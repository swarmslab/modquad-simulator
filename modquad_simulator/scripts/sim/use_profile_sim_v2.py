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

from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import circular_trajectory, simple_waypt_trajectory, \
    min_snap_trajectory

from modsim import params
from modsim.attitude import attitude_controller
# from modsim.plot.drawer_vispy import Drawer

from modsim.datatype.structure import Structure

from modsim.util.comm import publish_acc, publish_odom, publish_transform_stamped, publish_odom_relative, \
    publish_transform_stamped_relative
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
                                        find_suspects_by_profile,   \
                                        find_suspects_by_profile_prefix,   \
                                        expand_from_epictr

from modquad_sched_interface.interface import convert_modset_to_struc, \
                                              convert_struc_to_mat   , \
                                              rotpos_to_mat

import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen
from modquad_sched_interface.simple_scheduler import lin_assign

# ENABLE if you want to use the more complex Gurobi-based module-to-pos mapper
#from scheduler.gsolver import gsolve

# fig = plt.figure()
# fig2 = plt.figure()

# faulty_rots = []
# 
# fmod = sys.argv[1]
# frot = sys.argv[2]

# profile_fname = \
#"/home/arch/catkin_ws/src/modquad-simulator/modquad_simulator/profiles/noisy_lowrange_fail.json"
#"/home/arch/catkin_ws/src/modquad-simulator/modquad_simulator/profiles/fprof_ranged.json"
#"/home/arch/catkin_ws/src/modquad-simulator/modquad_simulator/profiles/fprof_ranged_lower_thrust_range.json"
#"/home/arch/catkin_ws/src/modquad-simulator/modquad_simulator/profiles/fprof_ranged_lower_thrust_cap.json"

# Publish ODOMETRY
def publish_odom_for_attached_mods(robot_id, structure_x, structure_y, xx, yy, main_id, odom_publishers, tf_broadcaster):
    publish_odom_relative(structure_x - xx[0], structure_y - yy[0], robot_id, main_id, odom_publishers[robot_id])
    publish_transform_stamped_relative(robot_id, main_id, structure_x - xx[0], structure_y - yy[0], tf_broadcaster)

def publish_structure_odometry(structure, odom_publishers, tf_broadcaster):
    ids, xx, yy, x = structure.ids, structure.xx, structure.yy, structure.state_vector

    main_id = ids[0]
    publish_odom(x, odom_publishers[main_id])
    publish_transform_stamped(main_id, x, tf_broadcaster)

    # show the other robots
    [publish_odom_for_attached_mods(robot_id, structure_x, structure_y, xx, yy,
        main_id, odom_publishers, tf_broadcaster)
        for robot_id, structure_x, structure_y in list(zip(ids, xx, yy))[1:]]

# Publish ACCELERATION
def publish_acc_for_attached_mods(robot_id, structure_x, structure_y, xx, yy, 
				  main_id, acc_publishers, tf_broadcaster):
	ids, xx, yy, x = structure.ids, structure.xx, structure.yy, structure.state_vector
	main_id = ids[0]
	return

def publish_structure_acc(structure, state_log, tdelta):
    vel1 = state_log[-1][3:6]
    vel2 = state_log[-2][3:6]
    acc = (vel2 - vel1) / tdelta

    pub = rospy.Publisher('/struc' + str(structure.struc_id) + '/imu', 
                            Imu, queue_size=1) 

    # This will publish to structure topic
    publish_acc(structure.state_vector, acc, pub)
    return

def inject_faults(mq_struc, max_faults, mset):
    global fmod, frot, faulty_rots
    faulty_rots = []
    num_faults = 0
    while num_faults < max_faults:
        #newfault = (random.randint(0,mset.num_mod-1), random.randint(0,3))
        newfault = (int(fmod), int(frot))
        if newfault not in faulty_rots:
            faulty_rots.append(newfault)
            num_faults += 1	
        num_faults += 1

    print("\tInjecting faults: {}".format(faulty_rots))
    for f in faulty_rots:
        mq_struc.motor_failure.append(f)
    #faulty_rots = [(f[0]+1, f[1]) for f in faulty_rots]
    faulty_rots = [(f[0], f[1]) for f in faulty_rots]

def simulate(structure, trajectory_function, sched_mset, speed=1, 
             fail_type=1, noise=0.0, fname="illegal", rfname="illegal"):
    """
    :param structure: structure obj from structure.py
    :param trajectory_function: a trajectory planner function
    :param sched_mset: the matrix representation of the structure
    :param speed: what speed have we planned mission at (m/s)
    :param fail_type: complete, 75%, or 50% failure
    :param noise: How much Gaussian noise to add (std_dev)
    :param fname: prefix for noise profile files
    :param rfname: file to store results in
    """

    if fname == "illegal":
        raise Exception("Illegal file prefix for profile")
    if rfname == "illegal":
        raise Exception("Illegal file name for results")

    num_mod = len(structure.xx)

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    rospy.set_param('is_modquad_sim', True)
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use
    rospy.init_node('modrotor_simulator', anonymous=True)
    robot_id1 = rospy.get_param('~robot_id', 'modquad01')
    rids = [robot_id1]

    state_log = []
    forces_log = []
    pos_err_log = [0,0,0]

    demo_trajectory = rospy.get_param('~demo_trajectory', True)

    odom_topic = rospy.get_param('~odom_topic', '/odom')  
    imu_topic  = rospy.get_param('~imu_topic', '/imu')  

    tmax = structure.traj_vars.total_dist / speed

    # Plotting coeffs
    overtime = 1.5

    # Odom publisher
    odom_publishers = {id_robot: 
            rospy.Publisher('/' + id_robot + odom_topic, Odometry, queue_size=0) 
            for id_robot in structure.ids}

    # Imu publisher
    imu_publishers = {id_robot: 
            rospy.Publisher('/' + id_robot + imu_topic, Imu, queue_size=1) 
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
    inject_time  = 0
    sus_del_time = 0

    # Which rotor we are testing currently
    quadrant_idx = 0
    
    # List of tuples of (mod_id, rot_id) rotors where fault is
    quadrant = []
    rotmat = []
    groups = []
    suspects = dict()

    # Rotor ramping variables for fault detection
    ramp_rotor_set = [[], []]
    ramp_factor = [1, 0]

    # Ramp up times
    rospy.set_param("fault_det_time_interval", 5.0)

    fdd_interval = rospy.get_param("fault_det_time_interval")
    fdetect_time = 0

    residual = []
    residual_log = []

    #while not rospy.is_shutdown() or t < overtime*tmax + 1.0 / freq:
    while t < overtime*tmax + 1.0 / freq:
        t += 1. / freq
        if ( t % 10 < 0.01 ):
            print("{:.01f} / {:.01f} - Residual = {}".format(t, overtime*tmax, residual))

        # Publish odometry
        publish_structure_odometry(structure, odom_publishers, tf_broadcaster)

        desired_state = trajectory_function(t, speed, structure.traj_vars)

        # Overwrite the control input with the demo trajectory
        [thrust_newtons, roll, pitch, yaw] = \
                position_controller(structure, desired_state)

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
                        ramp_rotor_set, ramp_factor, fail_type)

        # Compute in case of NO FAILURES for state estimation
        en_fail_rotor_act = False
        F_structure_est, M_structure_est, rotor_forces_est = \
                modquad_torque_control(
                        F_single, M_single, structure,
                        en_motor_sat, en_fail_rotor_act, 
                        ramp_rotor_set, ramp_factor, fail_type)

        # Perform state estimation for next time step
        est = simulation_step(structure, structure.state_vector, 
		                      F_structure_est, M_structure_est, 1. / freq)

        # The sleep is to make the simulation look like it would in real life
        rate.sleep()

        # Add noise to make things a bit more realistic
        F_structure  = np.array(F_structure)
        F_structure += np.random.normal(loc=0, scale=noise, size=F_structure.shape)

        # Simulate, obtain new state and state derivative
        structure.state_vector = simulation_step( structure, 
                                                  structure.state_vector, 
		                                          F_structure, 
                                                  M_structure, 
                                                  1.0 / freq             )

        # Compute residuals for error detection
        residual = structure.state_vector - est
        residual_log.append(residual)

        # Process the residual - i.e. check for failed rotors via thresholding
        # of residuals
        if fault_exists(residual) and not diagnose_mode:
            print("Enter fault diagnosis mode")
            diagnose_mode = True
            quadrant = get_faulty_quadrant_rotors(residual, structure)
            rotmat = rotpos_to_mat(structure, quadrant)
            groups = form_groups(quadrant, rotmat)

            # Provide some time for stabilization
            next_diag_t = t + 2.0 # 2 sec to collect data, allow fault to stabilize
            fdetect_time = t

        # If we are in the diagnose_mode, then we need to iteratively turn off
        # the rotors in the quadrant and see what state error goes to
        if diagnose_mode:
            if t >= next_diag_t: # Update rotor set
                # We found the faulty rotor
                if (abs(residual[-3] < 0.03) and abs(residual[-2]) < 0.03):
                    print("State Est = {}".format(est))
                    print("Residual = {}".format(residual[-3:-1]))

                    # Recurse over set if not already single rotor
                    if (len(ramp_rotor_set[0]) == 1): # Single rotor
                        with open(rfname, "a+") as f:
                            print("The faulty rotor is {}".format(ramp_rotor_set[0]))
                            f.write("2 {}-Mod: [\N{GREEK CAPITAL LETTER DELTA}t = {:5.2f}] Inject ({}, {}), ID'd: {} | [\N{GREEK CAPITAL LETTER DELTA}t = {:5.2f}] Suspects: {}\n".format(
                                    num_mod, t - inject_time, fmod, frot,
                                    ramp_rotor_set[0], sus_del_time, suspects), 
                            )
                        return

                    print("The faulty rotor is in set {}".format(
                                                            ramp_rotor_set[0]))

                    rotmat = update_rotmat(ramp_rotor_set[0], rotmat)

                    # Form smaller subgroups
                    groups = form_groups(ramp_rotor_set[0], rotmat)
                    quadrant_idx = 0 # Reset
                    print("New Groups: {}".format(groups))
                    ramp_rotor_set = [[], ramp_rotor_set[0]]
                else:
                    if next_diag_t <= t: #fdetect_time + fdd_interval:
                        best = [0, 0]
                        mag = 0
                        for entry in residual_log[-10:]:
                            new_mag = entry[-3]**2 + entry[-2]**2 
                            if new_mag > mag:
                                best = entry
                                mag = new_mag
                        suspects = find_suspects_by_profile_prefix( structure, 
                                                                    residual_log[-10:],
                                                                    fname               )

                        # Filter by whether suspect is in the detected quadrant
                        # TODO: Not hard code the failure types
                        suslist = set()
                        [suslist.add(sus) for ftype in [0,1,2] for sus in
suspects[ftype] if sus in quadrant]
                        suspects = list(suslist)
                        import pdb; pdb.set_trace()

                        sus_del_time = t - inject_time

                        if len(suspects) == 1:
                            with open(rfname, "a+") as f:
                                f.write("1 {}-Mod: [\N{GREEK CAPITAL LETTER DELTA}t = {:5.2f}] Inject ({}, {}), Suspects: {}\n".format(
                                            num_mod, sus_del_time, fmod, frot, suspects)
                                )
                            return

                        assert len(suspects) > 0

                        rotmat = rotpos_to_mat(structure, suspects)
                        groups = form_groups(suspects, rotmat)
                        print("New Groups: {}".format(groups))
                        if len(ramp_rotor_set[0]) > 0:
                            ramp_rotor_set = [[], ramp_rotor_set[0]]
                        else:
                            ramp_rotor_set = [groups[0], []]
                    else:
                        ramp_rotor_set, quadrant_idx = \
                                    expand_from_epictr(
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

        # Publish the acceleration data
        if not can_pub_imu:
            # Need at least three log entries to get the acceleration
            if (ind > 2):
                can_pub_imu = True
        else:
            publish_structure_acc(structure, state_log, 1.0/freq)

        # Inject faults
        if ( t >= tmax/10.0 and not faults_injected ):
            print("[t = {:.03f}]".format(t))
            max_faults = 1
            inject_faults(structure, max_faults, sched_mset)
            faults_injected = True
            inject_time = t

def test_shape_with_waypts( mset, wayptset, speed=1, fail_type=1, noise=0.0, fname="illegal", rfname='illegal'):

    if fname == "illegal":
        raise Exception("Need another profile fname")
    if rfname == "illegal":
        raise Exception("Need another results fname")

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

    simulate(struc1, trajectory_function, mset, speed=speed, fail_type=fail_type, noise=noise, fname=fname, rfname=rfname)

if __name__ == '__main__':
    global fmod, frot, faulty_rots, noise, rfname

    faulty_rots = []
    fmod = int(sys.argv[1])
    frot = int(sys.argv[2])
    n_idx= int(sys.argv[3])
    ftype= int(sys.argv[4])
    shape= int(sys.argv[5])

    noise_arr = [ 0.05, 0.15, 0.25 ]
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

    # File to store the results in
    rfname = "/home/arch/catkin_ws/src/modquad-simulator/" + \
             "modquad_simulator/fdd_prof_results/"       + \
             "n{:.01f}_fail{}.txt".format(noise, ftype)

    # Profile PREFIX, since we don't know type of failure ahead of time
    # However, we can make noise estimate, e.g., if system indoors + no HVAC
    pfprefix = "/home/arch/catkin_ws/src/modquad-simulator/" + \
               "modquad_simulator/profiles_v3/"       + \
               "n{:.01f}_fail".format(noise)

    print("starting simulation")
    results = test_shape_with_waypts(   structure, waypts, speed=3.5, 
                                        fail_type=ftype, noise=noise, 
                                        fname=pfprefix, rfname=rfname     )
    print("---------------------------------------------------------------")
