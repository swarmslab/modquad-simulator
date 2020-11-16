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
                                        update_rotmat

from modquad_sched_interface.interface import convert_modset_to_struc, \
                                              convert_struc_to_mat   , \
                                              rotpos_to_mat

import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

from scheduler.gsolver import gsolve, lin_assign

fig = plt.figure()
fig2 = plt.figure()

faulty_rots = []

fmod = sys.argv[1]
frot = sys.argv[2]

profile_fname = \
"/home/arch/catkin_ws/src/modquad-simulator/modquad_simulator/profiles/fprof_ranged.json"

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
    global faulty_rots
    faulty_rots = []
    num_faults = 0
    while num_faults < max_faults:
        #newfault = (random.randint(0,mset.num_mod-1), random.randint(0,3))
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

def simulate(structure, trajectory_function, sched_mset,
        t_step=0.01, speed=1, figind=1, 
        filesuffix="", max_faults=1):

    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    #print("Speed = {}".format(speed))
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
    rospy.set_param("fault_det_time_interval", 5.0)

    fdd_interval = rospy.get_param("fault_det_time_interval")

    residual = []

    residual_log = np.zeros((0,2))

    #while not rospy.is_shutdown() or t < overtime*tmax + 1.0 / freq:
    while t < overtime*tmax + 1.0 / freq:
        t += 1. / freq
        if ( t % 10 < 0.01 ):
            print("{:.01f} / {:.01f} - Residual = {}".format(t, overtime*tmax, residual))

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

        en_motor_sat = True
        en_fail_rotor_act = True

        # Control of Moments and thrust
        F_structure, M_structure, rotor_forces = \
                modquad_torque_control(
                        F_single, M_single, structure,
                        en_motor_sat, en_fail_rotor_act, 
                        ramp_rotor_set, ramp_factor)

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

        # Store data
        residual_log = np.vstack((residual_log, residual[-3:-1].reshape(1,2)))
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
            inject_faults(structure, max_faults, sched_mset)
            faults_injected = True
            print("Residual = {}".format(residual))

    # Collect all errors that are large out of last ten
    best = [0, 0]
    mag = best[0]**2 + best[1]**2
    entries = []
    for entry in residual_log[-100:]:
        new_mag = entry[0]**2 + entry[1]**2 
        # Filter out when it does well by chance
        if new_mag > 0.1:
            entries.append(entry)
    entries = np.array(entries)

    min_entry = [np.min(entries[:, 0]), np.min(entries[:, 1])]
    max_entry = [np.max(entries[:, 0]), np.max(entries[:, 1])]
    entries = np.array([min_entry, max_entry])

    # Take something in the middle as the profile
    #try:
    #    best = [round(np.mean(entries[:, 0]),2), round(np.mean(entries[:, 1]),2)]
    #except:
    #    import pdb
    #    pdb.set_trace()

    print("Chosen residuals:\n{}".format(entries))

    # Store the residual in a file
    pdata = {}
    with open(profile_fname, "r") as f:
        hashstr = structure.gen_hashstring(en_fail_motor=False)
        try:
            pdata = json.load(f)
            if type(pdata) == type(""):
                pdata = json.loads(pdata)
            #print("Loaded from file: {}".format(pdata))
            if hashstr not in pdata:
                pdata[hashstr] = {}
            modstr = "{}".format(fmod)
            if modstr not in pdata[hashstr]:
                pdata[hashstr][modstr] = {}
            rotstr = "{}".format(frot)

            if rotstr not in pdata[hashstr][modstr]:
                pdata[hashstr][modstr][rotstr] = []    

            #val = [round(x, 2) for x in best]
            
            pdata[hashstr][modstr][rotstr] = entries.tolist()

        except: # File is empty
            fault_entry = {hashstr: {fmod: {frot: residual[-3:-1].tolist()}}}
            pdata = json.dumps(fault_entry)
            print("New json file being written")
        #print(pdata)
        #print(ndata)

    with open(profile_fname, "w") as f:
        json.dump(pdata, f, indent=4)

    # Process the final residual - i.e. check for failed rotors via thresholding
    # of residuals
    #residual = structure.state_vector - est
    #print("[{:.02f}] Residual: {}".format(t, ["{:.02f}".format(entry) for entry in residual[-3:-1]]))

    # traj_vars = structure.traj_vars 
    # pos_err_log /= ind
    # pos_err_log = np.sqrt(pos_err_log)
    # integral_val = np.sum(np.array(forces_log) ** 2) * (1.0 / freq)
    # #print("Final position = {}".format(structure.state_vector[:3]))

    # if figind < 1:
    #     #print("total integral={}".format(integral_val))
    #     return integral_val
    # #ax.grid()

    # state_log = np.array(state_log)
    # ax.plot(state_log[:, 0], state_log[:, 1], state_log[:, 2], 
    #         zdir='z', color='r', linewidth=lw)
    # ax.legend(["Planned Path", "Actual Path"])
    # plt.savefig("figs/3d_{}.pdf".format(filesuffix))
    # plt.sca(fig.gca())

    # waypt_time_step = 1.0
    # tstate = np.arange(0, tmax + 1.0/freq, 1.0/freq)
    # twaypt = np.arange(0, tmax + waypt_time_step, waypt_time_step)

    # # Generate legend
    # legend_vals = ["Actual path", "Desired path"]
    # # Shrink current axis's height by 10% on the bottom
    # ax2 = plt.gca()
    # box = ax.get_position()
    # ax2.set_position([box.x0, box.y0 + box.height * 0.1,
    #                      box.width, box.height * 0.9])

    # ylabelsize = 12
    # # Plot first one so that we can do legend
    # plt.subplot(4,1,figind+0)
    # plt.plot(tstate, state_log[:, 0], color='r', linewidth=lw)
    # plt.plot(twaypt, traj_vars.waypts[:,0], alpha=alphaset, color='g', linewidth=lw)
    # plt.ylabel("X position\n(m)", size=ylabelsize)
    # plt.gca().set_ylim(xmin, xmax)
    # plt.gca().xaxis.set_ticklabels([])
    # plt.grid()

    # # Put a legend below current axis
    # plt.figlegend(legend_vals, loc='upper center', ncol=2)#bbox_to_anchor=(0.5,  0.95),
    #                   #fancybox=True, shadow=True, ncol=2)

    # plt.subplot(4,1,figind+1)
    # plt.plot(tstate, state_log[:, 1], color='r', linewidth=lw)
    # plt.plot(twaypt, traj_vars.waypts[:,1], alpha=alphaset, color='g', linewidth=lw)
    # plt.ylabel("Y position\n(m)", size=ylabelsize)
    # plt.gca().set_ylim(ymin, ymax)
    # plt.gca().xaxis.set_ticklabels([])
    # plt.grid()

    # plt.subplot(4,1,figind+2)
    # plt.plot(tstate, state_log[:, 2], color='r', linewidth=lw)
    # plt.plot(twaypt, traj_vars.waypts[:,2], alpha=alphaset, color='g', linewidth=lw)
    # plt.ylabel("Z position\n(m)", size=ylabelsize)
    # plt.gca().set_ylim(zmin, zmax)
    # plt.gca().xaxis.set_ticklabels([])
    # plt.grid()

    # # sum of the squared forces
    # plt.subplot(4,1,figind+3)
    # plt.xlabel("Time (sec)")
    # plt.ylabel("Force\n(N)", size=ylabelsize)
    # #forces_log = forces_log[5:]
    # #plt.plot(tstate[5:], np.sum(np.array(forces_log) ** 2, axis=1), color='r', linewidth=lw)
    # plt.plot(tstate[5:], np.array(forces_log[5:]) ** 2, color='r', linewidth=lw)
    # plt.gca().set_ylim(0, 0.10)
    # plt.grid()
    # #strftime("%Y-%m-%d_%H:%M:%S", localtime()), 
    # plt.savefig("figs/2d_{}.pdf".format(filesuffix))
    # #print("total integral={}".format(np.sum(np.array(forces_log) ** 2) * t_step))
    # plt.clf() # Clear figures
    # plt.sca(ax)
    # plt.clf()


    # # Plot the residuals
    # plt.figure()
    # plt.plot(residual_log[:, 0], 'b.' , linewidth=3.0)
    # plt.plot(residual_log[:, 1], 'r--', linewidth=2.5)
    # plt.savefig("figs/residual_{}.pdf".format(filesuffix))
    # plt.clf()

    # # Plot the ang vel over time
    # plt.figure
    # plt.plot(tstate, state_log[:, -3], 'b-', label=r"$\dot{\phi}$")
    # plt.plot(tstate, state_log[:, -3] - residual_log[:, 0], 'b--', label=r"$\hat{\dot{\phi}}$")
    # plt.plot(tstate, state_log[:, -2], 'r-', label=r"$\dot{\theta}$")
    # plt.plot(tstate, state_log[:, -2] - residual_log[:, 1], 'r--', label=r"$\hat{\dot{\theta}}$")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Ang Vel")
    # plt.legend()
    # plt.savefig("figs/angvel_{}.pdf".format(filesuffix))
    # plt.clf()

    integral_val = -1
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

    forces, pos_err = simulate(struc1, trajectory_function, mset, 
            figind=1, speed=speed, 
            filesuffix="{}_m{}r{}_profile".format(test_id, fmod, frot), 
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
    spd = 5.0
    results = test_shape_with_waypts(
                       #structure_gen.rect(5, 5), 
                       #structure_gen.plus(3, 3), 
                       structure_gen.zero(4, 4), 
                       #structure_gen.airplane(5,5,3),
                       waypt_gen.helix(radius=2.5, rise=1, num_circ=1),
                       #waypt_gen.line([0,0,0],[5,-5,5]),
                       #waypt_gen.zigzag_xy(10, 5, num_osc=8.0, start_pt=[0,0,0]),
                       speed=spd, 
                       test_id="4x4zero_2.5x1x1helix_spd{}_".format(spd), 
                       #test_id="5x5sq_(0,0,0)-(5,-5,5)_spd{}_".format(spd), 
                       doreform=True, max_fault=1, rand_fault=False)
    #print("Force used: {}".format(results[0]))
    #print("RMSE Position Error: {}".format(np.mean(results[1])))
    #print("Faults: {}".format(faulty_rots))
    print("---------------------------------------------------------------")
    print("Sim complete!")
