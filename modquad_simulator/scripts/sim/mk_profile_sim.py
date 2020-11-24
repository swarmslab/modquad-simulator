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
                                        compute_residual

from modquad_sched_interface.interface import convert_modset_to_struc, \
                                              convert_struc_to_mat   , \
                                              rotpos_to_mat

import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

#from scheduler.gsolver import gsolve, lin_assign
from modquad_sched_interface.simple_scheduler import lin_assign

#profile_fname = \
#"/home/arch/catkin_ws/src/modquad-simulator/modquad_simulator/profiles/noisy_lowrange_fail.json"
#"/home/arch/catkin_ws/src/modquad-simulator/modquad_simulator/profiles/noisy_lowcap_fail.json"
#"/home/arch/catkin_ws/src/modquad-simulator/modquad_simulator/profiles/noisy_full_fail.json"

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
    global faulty_rots, fmod, frot
    faulty_rots = []
    num_faults = 0
    while num_faults < max_faults:
        newfault = (int(fmod), int(frot))
        if newfault not in faulty_rots:
            faulty_rots.append(newfault)
            num_faults += 1	
        num_faults += 1

    print("Injecting faults: {}".format(faulty_rots))
    for f in faulty_rots:
        mq_struc.motor_failure.append(f)
    #faulty_rots = [(f[0]+1, f[1]) for f in faulty_rots]
    faulty_rots = [(f[0], f[1]) for f in faulty_rots]

def ros_param_setup(speed):
    rospy.set_param("fdd_group_type", "log4")
    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    rospy.set_param('rotor_map', 2) # So that modquad_torque_control knows which mapping to use
    rospy.set_param("fault_det_time_interval", 5.0)
    rospy.set_param('is_modquad_sim', True)
    rospy.init_node('modrotor_simulator', anonymous=True)

def simulate(structure, trajectory_function, sched_mset, speed=1, fail_type=1, noise=0.0, fname="illegal"):
    global fmod, frot, faulty_rots

    if fname == "illegal":
        raise Exception("Need a profile fname")

    ros_param_setup(speed)

    state_log = []
    forces_log = []
    pos_err_log = [0,0,0]

    odom_topic = rospy.get_param('~odom_topic', '/odom')  
    imu_topic  = rospy.get_param('~imu_topic', '/imu')  

    tmax = structure.traj_vars.total_dist / speed

    # Plotting coeffs
    overtime = 1.0

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
    fdd_interval = rospy.get_param("fault_det_time_interval")

    residual = []
    residual_log = np.zeros((0,3))

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

        en_fail_rotor_act = False

        # Compute in case of no failures for state estimation
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

        # Simulate, obtain new state and state derivative
        F_structure  = np.array(F_structure)
        F_structure += np.random.normal(loc=0, scale=noise, size=F_structure.shape)
        structure.state_vector = simulation_step( structure, 
                                                  structure.state_vector, 
		                                          F_structure, 
                                                  M_structure, 
                                                  1.0 / freq             )

        # Compute residuals for error detection
        residual = structure.state_vector - est

        # [del x, del y, del z, del vx, del vy, del vz]
                    #compute_residual(structure.state_vector, desired_state)

        # Store data
        if faults_injected:
            residual_log = np.vstack((residual_log, residual[-3:].reshape(1,3)))
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
            max_faults=1
            inject_faults(structure, max_faults, sched_mset)
            faults_injected = True
            print("Residual = {}".format(residual))

    # Collect all errors that are large out of last ten
    #best = [0, 0]
    #mag = best[0]**2 + best[1]**2
    entries = []
    for entry in residual_log[-1000:]:
        # new_pmag = entry[0]**2 + entry[1]**2 + entry[2]**2
        # new_vmag = entry[3]**2 + entry[4]**2 + entry[5]**2
        pqr_mag = entry[0]**2 + entry[1]**2 + entry[2]**2
        # Filter out when it does well by chance
        if pqr_mag > 0.05:
            entries.append(entry)
    entries = np.array(entries)

    min_entry = np.min(entries, axis=0)
    max_entry = np.max(entries, axis=0)
    entries = np.array([min_entry, max_entry])

    print("Chosen residuals:\n{}".format(entries))

    # Store the residual in a file
    pdata = {}

    try:
        with open(fname, "r+") as f:
            pass
    except:
        # Needed to prevent weird first-write bug where not all data written
        with open(fname, "w") as f: # overwrite the file
            json.dump(pdata, f, indent=4)
    
    with open(fname, "r+") as f:
        hashstr = structure.gen_hashstring(en_fail_motor=False)
        try:
            pdata = json.load(f)
            if type(pdata) == type(""):
                pdata = json.loads(pdata)
            if hashstr not in pdata:
                pdata[hashstr] = {}
            modstr = "{}".format(fmod)
            if modstr not in pdata[hashstr]:
                pdata[hashstr][modstr] = {}
            rotstr = "{}".format(frot)

            if rotstr not in pdata[hashstr][modstr]:
                pdata[hashstr][modstr][rotstr] = []    

            pdata[hashstr][modstr][rotstr] = entries.tolist()

        except: # File is empty
            fault_entry = {hashstr: {fmod: {frot: residual[-3:-1].tolist()}}}
            pdata = json.dumps(fault_entry)
            print("New json file being written")

    with open(fname, "w") as f: # overwrite the file
        json.dump(pdata, f, indent=4)

def test_shape_with_waypts( mset, wayptset, speed=1, fail_type=1, noise=0.0, fname="illegal"):
    global fmod, frot, faulty_rots

    if fname == "illegal":
        raise Exception("Need a profile fname")

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

    simulate(struc1, trajectory_function, mset, speed=speed, fail_type=fail_type, noise=noise, fname=fname)

if __name__ == '__main__':
    global fmod, frot, faulty_rots, noise, ftype

    random.seed(1)
    spd = 3.5
    waypts = waypt_gen.helix(radius=2.5, rise=2, num_circ=2)

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

    rfname = "/home/arch/catkin_ws/src/modquad-simulator/" + \
             "modquad_simulator/profiles_v3/"       + \
             "n{:.01f}_fail{}.txt".format(noise, ftype)

    print("starting simulation")
    results = test_shape_with_waypts( structure, waypts, 
                                        speed=spd, fail_type=ftype, 
                                        noise=noise, fname=rfname)
    print("---------------------------------------------------------------")
    print("Sim complete!")
