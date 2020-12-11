#!/usr/bin/env python3

import numpy as np
import time
import matplotlib.pyplot as plt
import math

import rospy
import tf2_ros

from geometry_msgs.msg  import Twist
from std_msgs.msg       import Int8MultiArray
from std_srvs.srv       import Empty, EmptyRequest
from crazyflie_driver.srv import UpdateParams

from tf.transformations import euler_from_quaternion

from modsim.datatype.structure import Structure

from modsim.controller  import position_controller
from modsim.trajectory  import min_snap_trajectory

from modsim             import params
from modsim.params      import RunType
from modsim.util.state  import init_state, state_to_quadrotor
from modsim.util.thrust import convert_thrust_newtons_to_pwm

from modsim.util.fault_detection import fault_exists_real,      \
                                        real_find_suspects,     \
                                        get_faulty_quadrant_rotors_real,\
                                        update_ramp_rotors, update_ramp_factors, \
                                        form_groups, update_rotmat

from dockmgr.datatype.PoseManager import PoseManager
#from dockmgr.datatype.ImuManager import ImuManager

from modquad_sched_interface.interface import convert_modset_to_struc, \
                                              convert_struc_to_mat,    \
                                              rotpos_to_mat

import modquad_sched_interface.waypt_gen     as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

from modquad_sched_interface.simple_scheduler import lin_assign

# Set up for Structure Manager
structure = None
t = 0.0
traj_func = min_snap_trajectory
start_id = 16 # 1 indexed

logs = {
	't':     [], 'sthrust': [],                 # time, desired thrust
	'x':     [], 'y':       [], 'z':        [], # measured position
	'vx':    [], 'vy':      [], 'vz':       [], # measured linear velocity
	'roll':  [], 'pitch':   [], 'yaw':      [], # measured attitude
	'vroll': [], 'vpitch':  [], 'vyaw':     [], # measured angular velocity
	'sroll': [], 'spitch':  [], 'syawrate': [], # desired roll, pitch, yaw rate
	'desx':  [], 'desy':    [], 'desz':     [], # desired position
	'desvx': [], 'desvy':   [], 'desvz':    []  # desired linear velocity
}

def switch_estimator_to_kalman_filter(start_id, num_robot):
    srv_name_set = ['/modquad{:02d}/switch_to_kalman_filter'.format(mid) for mid in range(start_id, start_id+num_robot)]
    switch_set = [ rospy.ServiceProxy(srv_name, Empty) for srv_name in srv_name_set ]
    rospy.loginfo('Wait for all switch_to_kalman_filter services')
    [rospy.wait_for_service(srv_name) for srv_name in srv_name_set]
    rospy.loginfo('Found all switch_to_kalman_filter services')
    msg = EmptyRequest()
    [switch(msg) for switch in switch_set]

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

def update_logs(t, state_vector, desired_state, thrust, roll, pitch, yawrate):
    global logs

    # Add to logs
    logs['t'].append(t)

    logs['x'].append(state_vector[0])
    logs['y'].append(state_vector[1])
    logs['z'].append(state_vector[2])

    #vel = structure.state_vector[3:6]
    logs['vx'].append(state_vector[3])
    logs['vy'].append(state_vector[4])
    logs['vz'].append(state_vector[5])


    # Orientation for a single rigid body is constant throughout the body
    euler = euler_from_quaternion(state_vector[6:10])
    logs['roll'].append(math.degrees(euler[0]))
    logs['pitch'].append(math.degrees(euler[1]))
    logs['yaw'].append(math.degrees(euler[2]))

    # Measured angular velocities 
    logs['vroll'].append(state_vector[-3])
    logs['vpitch'].append(state_vector[-2])
    logs['vyaw'].append(state_vector[-1])

    # Add to logs
    logs['desx'].append(desired_state[0][0])
    logs['desy'].append(desired_state[0][1])
    logs['desz'].append(desired_state[0][2])

    logs['desvx'].append(desired_state[1][0])
    logs['desvy'].append(desired_state[1][1])
    logs['desvz'].append(desired_state[1][2])

    logs['sthrust'].append(thrust)
    logs['sroll'].append(roll)
    logs['spitch'].append(pitch)
    logs['syawrate'].append(yawrate)

def init_params(speed):
    #rospy.set_param("kalman/resetEstimation", 1)
    rospy.set_param("fault_det_time_interval",    5.0) # Time interval for fault detection groups
    rospy.set_param("fdd_group_type",         "indiv") # FDD Group Size = 1 Robot
    rospy.set_param('opmode',                'normal')
    rospy.set_param('structure_speed',          speed)
    rospy.set_param('rotor_map',                    3) # So that modquad_torque_control knows which mapping to use
    rospy.set_param('is_modquad_sim',           False) # For controller.py
    rospy.set_param('is_modquad_bottom_framed', False)
    rospy.set_param('is_modquad_unframed',      False)
    rospy.set_param('is_strong_rots',           False) # For controller.py
    rospy.loginfo("!!READY!!")
    np.set_printoptions(precision=2)

def update_state(pose_mgr, structure, freq): # freq computed as 1 / (t - prev_t)
    # Convert the individual new states into structure new state
    # As approximant, we let the first modules state be used
    new_states = pose_mgr.get_new_states() # From OPTITRACK
    new_pos = np.array([state[:3] for state in new_states])
    new_pos = np.mean(new_pos, axis=0).tolist()

    if np.all(structure.prev_state_vector == 0):
        structure.prev_state_vector = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0])
    else:
        structure.prev_state_vector = structure.state_vector

    # Update position to be centroid of structure
    structure.state_vector = np.array(pose_mgr.get_new_state(1))
    structure.state_vector[0] = new_pos[0]
    structure.state_vector[1] = new_pos[1]
    structure.state_vector[2] = new_pos[2]

    # compute instantaneous velocities
    vels = (structure.state_vector[:3] - structure.prev_state_vector[:3]) / (1.0 / freq)
    structure.state_vector[3] = vels[0] # np.mean(velbuf[:, 0]) # vx
    structure.state_vector[4] = vels[1] # np.mean(velbuf[:, 1]) # vy
    structure.state_vector[5] = vels[2] # np.mean(velbuf[:, 2]) # vz

    # compute instantaneous angular velocities
    vels = [0.0, 0.0, 0.0]
    if np.all(structure.prev_state_vector == 0):
        # compute euler angles - RPY roll pitch yaw
        prev_angs = euler_from_quaternion(structure.prev_state_vector[6:10])
        curr_angs = euler_from_quaternion(structure.state_vector[6:10])

        prev_angs = np.array([prev_angs[0], prev_angs[1], prev_angs[2]])
        curr_angs = np.array([curr_angs[0], curr_angs[1], curr_angs[2]])

        # compute angular velocities
        vels = (curr_angs - prev_angs) / (1.0 / freq)
    
    # Update state vector with smoothed linear/angular velocities
    structure.state_vector[-3] = vels[0] # np.mean(velbuf[:, 3]) # vroll 
    structure.state_vector[-2] = vels[1] # np.mean(velbuf[:, 4]) # vpitch
    structure.state_vector[-1] = vels[2] # np.mean(velbuf[:, 5]) # vyaw

def run(traj_vars, t_step=0.01, speed=1):
    global logs
    global t, traj_func, start_id, structure

    freq = 100.0  # 100hz
    rate = rospy.Rate(freq)
    t = 0
    ind = 0
    num_robot = 4

    worldFrame = rospy.get_param("~worldFrame", "/world")
    init_params(speed) # Prints "!!READY!!" to log

    pose_mgr = PoseManager(num_robot, '/modquad', start_id=start_id-1) #0-indexed
    pose_mgr.subscribe()
 
    """ Publish here to control
    crazyflie_controller/src/controller.cpp has been modified to subscribe to
    this topic, and if we are in the ModQuad state, then the Twist message
    from mq_cmd_vel will be passed through to cmd_vel
    TODO: modify so that we publish to all modules in the struc instead of
    single hardcoded one """
    publishers = [ rospy.Publisher('/modquad{:02d}/mq_cmd_vel'.format(mid), 
                    Twist, queue_size=100) 
                    for mid in range (start_id, start_id + num_robot) ]

    # Publish to robot
    msg = Twist()

    # First few msgs will be zeros
    msg.linear.x = 0 # roll [-30, 30] deg
    msg.linear.y = 0 # pitch [-30, 30] deg
    msg.linear.z = 0 # Thrust ranges 10000 - 60000
    msg.angular.z = 0 # yaw rate

    # shutdown
    rospy.on_shutdown(_simple_landing)

    # Update pose
    rospy.sleep(1)

    # Start by sending NOPs so that we have known start state
    # Useful for debugging and safety
    t = 0
    while t < 3:
        t += 1.0 / freq
        [ p.publish(msg) for p in publishers ]
        if round(t, 2) % 1.0 == 0:
            rospy.loginfo("Sending zeros at t = {}".format(round(t,2)))
        rate.sleep()
        update_state(pose_mgr, structure, freq)

    # Update for the 2x2 structure
    update_att_ki_gains(start_id, num_robot)
    structure.update_firmware_params()
    #import pdb; pdb.set_trace()
    #switch_estimator_to_kalman_filter(start_id, num_robot)

    for mid in range(start_id, start_id + num_robot):
        rospy.loginfo(
            "setup complete for /modquad{:02d}".format(mid))


    _takeoff(pose_mgr, structure, freq, publishers)

    """
    THIS WILL NOT AUTOMATICALLY CAUSE THE ROBOT TO DO ANYTHING!!
    YOU MUST PAIR THIS WITH MODIFIED CRAZYFLIE_CONTROLLER/SRC/CONTROLLER.CPP
    AND USE JOYSTICK TO SWITCH TO MODQUAD MODE FOR THESE COMMANDS TO WORK
    """
    tstart = rospy.get_time()
    t = 0
    prev_t = t
    while not rospy.is_shutdown() and t < 90.0:
        # Update time
        prev_t = t
        t = rospy.get_time() - tstart
        dt = t - prev_t

        update_state(pose_mgr, structure, 1.0/dt)

        # Get new desired state
        desired_state = traj_func(t, speed, traj_vars)

        # Get new control inputs
        [thrust, roll, pitch, yaw] = \
                position_controller(structure, desired_state, dt)

        des_pos  = np.array(desired_state[0])
        is_pos   = structure.state_vector[:3]
        residual = des_pos - is_pos

        update_logs(t, structure.state_vector, desired_state, thrust, roll, pitch, yaw)
 
        # Update message content
        msg.linear.x  = pitch  # pitch [-30, 30] deg
        msg.linear.y  = roll   # roll [-30, 30] deg
        msg.linear.z  = thrust # Thrust ranges 10000 - 60000
        msg.angular.z = yaw    # yaw rate

        if round(t, 2) % 0.5 == 0:
            rospy.loginfo("[{}] {}".format(round(t, 1), 
                                        np.array([thrust, roll, pitch, yaw])))
            rospy.loginfo("     Des={}, Is={}".format(
                                            np.array(desired_state[0]), 
                                            np.array(structure.state_vector[:3])))
            rospy.loginfo("     Fpwm={}".format( thrust ))
            rospy.loginfo("")

        # Send control message
        [ p.publish(msg) for p in publishers ]

        # The sleep preserves sending rate
        rate.sleep()

    _landing(pose_mgr, structure, freq, publishers, msg.linear.z)

    make_plots()
    rospy.loginfo("DONE")

def make_plots():
    global logs

    plt.figure()
    ax0 = plt.subplot(3,3,1)
    ax1 = ax0.twinx()
    ax0.plot(logs['t'], logs['spitch'], 'c')
    ax1.plot(logs['t'], logs['x'], 'r', label='xpos')
    ax1.plot(logs['t'], logs['desx'], 'k', label='desx')
    ax1.set_ylabel("X (m)")
    ax1.legend(loc='lower right')
    ax1.set_ylim(-10, 10)
    ax0.set_ylim(-5, 5)
    ax0.set_ylabel("Pitch (deg)")

    ax2 = plt.subplot(3,3,2)
    ax3 = ax2.twinx()
    ax2.plot(logs['t'], logs['spitch'], 'c')
    ax3.plot(logs['t'], logs['vx'], 'r', label='xvel')
    ax3.plot(logs['t'], logs['desvx'], 'k', label='desvx')
    ax3.legend(loc='lower right')
    ax3.set_ylabel("X (m/s)")
    ax2.set_ylabel("Pitch (deg)")
    #ax3.set_ylim(-200, 200)
    ax2.set_ylim(-5, 5)

    ax4 = plt.subplot(3,3,3)
    ax5 = ax4.twinx()
    ax4.plot(logs['t'], logs['spitch'], 'c', label='des pitch')
    ax4.legend(loc='lower right')
    ax4.set_ylabel("Pitch (deg)")
    ax5.plot(logs['t'], logs['pitch'], 'm')
    ax5.set_ylabel("Pitch (deg)")
    ax4.set_ylim(-20, 20)
    ax5.set_ylim(-5, 5)

    ax6 = plt.subplot(3,3,4)
    ax7 = ax6.twinx()
    ax6.plot(logs['t'], logs['sroll'], 'c' )
    ax7.plot(logs['t'], logs['y'], 'g', label='ypos')
    ax7.plot(logs['t'], logs['desy'], 'k', label='desy')
    ax7.legend(loc='lower right')
    ax7.set_ylabel("Y (m)")
    ax6.set_ylabel("Roll (deg)")
    ax6.set_ylim(-5, 5)
    ax7.set_ylim(-10, 10)

    ax8 = plt.subplot(3,3,5)
    ax9 = ax8.twinx()
    ax8.plot(logs['t'], logs['sroll'], 'c' )
    ax9.plot(logs['t'], logs['vy'], 'g', label='vy')
    ax9.plot(logs['t'], logs['desvy'], 'k', label='des_vy')
    ax9.legend(loc='lower right')
    ax9.set_ylabel("Y (m/s)")
    ax8.set_ylabel("Roll (deg)")
    ax8.set_ylim(-5, 5)
    #ax9.set_ylim(-200, 200)

    ax10 = plt.subplot(3,3,6)
    ax10.plot(logs['t'], logs['sroll'], 'c', label='des_roll')
    ax10.set_ylabel("Roll (deg), cyan")
    ax11 = ax10.twinx()
    ax11.plot(logs['t'], logs['roll'], 'm', label='meas_roll')
    ax11.set_ylabel("Meas Roll (deg), magenta")
    ax10.set_ylim(-5, 5)
    ax11.set_ylim(-20, 20)

    ax12 = plt.subplot(3,3,7)
    ax13 = ax12.twinx()
    ax12.plot(logs['t'], logs['sthrust'], 'c' )
    ax13.plot(logs['t'], logs['z'], 'b', label='zpos')
    ax13.plot(logs['t'], logs['desz'], 'k', label='desz')
    ax13.legend(loc='lower right')
    ax13.set_ylabel("Z (m)")
    ax12.set_ylabel("Thrust (PWM)")
    ax12.set_ylim(5000, 61000)
    ax13.set_ylim(-0.1, 1.0)
    ax12.set_xlabel('Time (sec)')

    ax14 = plt.subplot(3,3,8)
    ax15 = ax14.twinx()
    ax14.plot(logs['t'], logs['sthrust'], 'c' )
    ax15.plot(logs['t'], logs['vz'], 'b', label='vz')
    ax15.plot(logs['t'], logs['desvz'], 'k', label='desvz')
    ax15.legend(loc='lower right')
    ax15.set_ylabel("Z (m/s)")
    ax14.set_ylabel("Thrust (PWM)")
    ax14.set_ylim(5000, 61000)
    #ax15.set_ylim(-20, 20)
    ax14.set_xlabel('Time (sec)')

    ax16 = plt.subplot(3,3,9)
    ax17 = ax16.twinx()
    ax16.plot(logs['t'], [0 for _ in range(len(logs['t']))], 'k' )
    ax16.plot(logs['t'], logs['syawrate'], 'c', label=r'Sent $\dot{\psi}$')
    ax16.plot(logs['t'], logs['vyaw'], 'm', label=r'Meas $\dot{\psi}$')
    ax17.plot(logs['t'], logs['yaw'], 'b')
    ax16.set_ylabel("Yaw Rate (deg/s), cyan")
    ax17.set_ylabel("Meas Yaw (deg), magenta")
    ax16.set_ylim(-210, 210)
    ax17.set_ylim(-250, 250)
    ax16.set_xlabel('Time (sec)')
    ax15.legend(loc='lower right')

    plt.subplots_adjust(left=0.11, bottom=0.09, right=0.9, top=0.99, wspace=0.36, hspace=0.26)

    plt.show()
    plt.close()

def _takeoff(pose_mgr, structure, freq, publishers):
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
    pidz_ki = 3500
    rospy.loginfo("Start Control")
    rospy.loginfo("TAKEOFF")
    while not taken_off:
        update_state(pose_mgr, structure, freq)

        if structure.state_vector[2] > 0.05 or msg.linear.z > 50000:
            #msg.linear.z = 0
            structure.pos_accumulated_error = msg.linear.z / pidz_ki
            taken_off = True

        # Convert thrust to PWM range
        msg.linear.z += 10000 * (1.0/freq)

        [ p.publish(msg) for p in publishers ]

        # The sleep preserves sending rate
        rate.sleep()
    rospy.loginfo("COMPLETED TAKEOFF")

def _landing(pose_mgr, structure, freq, publishers, cur_thrust):
    global start_id

    rospy.loginfo("LANDING")
    rate = rospy.Rate(freq)

    # Publish to robot
    msg = Twist()

    # LANDING
    landed = False

    # Message init for takeoff
    msg.linear.x  = 0  # pitch [-30, 30] deg
    msg.linear.y  = 0  # roll [-30, 30] deg
    msg.angular.z = 0  # yaw rate
    msg.linear.z  = cur_thrust
    while not landed:
        update_state(pose_mgr, structure, freq)

        if structure.state_vector[2] <= 0.02 or msg.linear.z < 11000:
            landed = True

        msg.linear.z -= 10000 * (1.0/freq)

        [ p.publish(msg) for p in publishers ]

        # The sleep preserves sending rate
        rate.sleep()

    msg.linear.z = 0
    [ p.publish(msg) for p in publishers ]

def _simple_landing():
    global start_id

    rospy.loginfo("SIMPLE LANDING")
    freq = 80.0 # Hz
    rate = rospy.Rate(freq)

    prefix = 'modquad'
    num_robot = rospy.get_param("num_robots", 4)

    publishers = [ rospy.Publisher('/modquad{:02d}/mq_cmd_vel'.format(mid), Twist, queue_size=100) for mid in range (start_id, start_id + num_robot) ]

    # Publish to robot
    msg = Twist()

    # LANDING
    landed = False

    # Message init for takeoff
    msg.linear.x  = 0  # pitch [-30, 30] deg
    msg.linear.y  = 0  # roll [-30, 30] deg
    msg.angular.z = 0  # yaw rate
    while not landed:

        if msg.linear.z < 11000:
            landed = True

        msg.linear.z -= 10000 * (1.0/freq)

        [ p.publish(msg) for p in publishers ]

        # The sleep preserves sending rate
        rate.sleep()

    msg.linear.z = 0
    [ p.publish(msg) for p in publishers ]

def test_shape_with_waypts(num_struc, wayptset, speed=1, test_id="", 
        doreform=False, max_fault=1, rand_fault=False):

    global traj_func, t, start_id, structure
    # Need to call before reset_docking to ensure it gets right start_id
    start_mod_id = start_id-1 # 0-indexed
    rospy.set_param('start_mod_id', start_mod_id) # 0-indexed

    traj_func = min_snap_trajectory
    traj_vars = traj_func(0, speed, None, wayptset)
    loc=[0,0,0]
    state_vector = init_state(loc, 0)

    mset = structure_gen.rect(2, 2)
    lin_assign(mset, start_id=start_mod_id, reverse=True) # 0-indexed
    #print(mset.pi)
    structure = convert_modset_to_struc(mset, start_mod_id)
    structure.state_vector = state_vector
    structure.traj_vars = traj_vars

    # Verify this is the correct structure
    pi = convert_struc_to_mat(structure.ids, structure.xx, structure.yy)
    print("Structure Used: ")
    print("{}".format(pi.astype(np.int64)))

    rospy.on_shutdown(_simple_landing)

    rospy.init_node('modrotor_simulator')

    time.sleep(2)

    run(speed=speed, traj_vars=traj_vars)

if __name__ == '__main__':
    print("starting simulation")

    # The place, according to mocap, where robot will start
    x = -0.239 # 6.68 # 4.9#  6.3
    y =  5.219 # 0.64 #-0.9# -1.0
    z =   0.0 # 0.0  # 0.5

    num_struc = 1
    results = test_shape_with_waypts(
                       num_struc, 
                       #waypt_gen.zigzag_xy(2.0, 1.0, 6, start_pt=[x-1,y-0.5,0.5]),
                       waypt_gen.helix(radius=0.5, 
                                       rise=0.5, 
                                       num_circ=4, 
                                       start_pt=[x, y, 0.0]),
                       #waypt_gen.waypt_set([[x-0.0  , y+0.00  , 0.0],
                       #                     [x-0.0  , y+0.00  , 0.1],
                       #                     [x-0.0  , y+0.00  , 0.2],
                       #                     [x-0.0  , y+0.00  , 0.6]
                       #                     #[x+1  , y    , 0.5]
                       #                    ]),
                       #waypt_gen.waypt_set([[x    , y    , 0.0],
                       #                     [x    , y    , 0.1],
                       #                     [x    , y    , 0.3],
                       #                     [x-1.0, y-0.2, 0.5],
                       #                     [x-1.0, y+0.2, 0.5],
                       #                     [x+1.0, y+0.2, 0.5],
                       #                     [x+1.0, y-0.2, 0.5],
                       #                     [x-1.0, y-0.2, 0.5],
                       #                     [x-1.0, y+0.2, 0.5],
                       #                     [x+1.0, y+0.2, 0.5],
                       #                     [x+1.0, y-0.2, 0.5],
                       #                     [x-1.0, y-0.2, 0.5],
                       #                     [x-1.0, y+0.2, 0.5],
                       #                     [x+1.0, y+0.2, 0.5],
                       #                     [x+1.0, y-0.2, 0.5],
                       #                     [x    , y    , 0.5],
                       #                     [x    , y    , 0.2]
                       #                    ]
                       #                   ),
                       speed=0.3, test_id="controls", 
                       doreform=True, max_fault=1, rand_fault=False)
    print("---------------------------------------------------------------")
