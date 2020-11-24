#!/usr/bin/env python2

import numpy as np
import time
import matplotlib.pyplot as plt
import math

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Int8MultiArray
from std_srvs.srv import Empty, EmptyRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from modsim.datatype.structure import Structure
from modsim.datatype.structure_manager import StructureManager
from dockmgr.datatype.disassembly_manager import DisassemblyManager
from dockmgr.datatype.assembly_manager import AssemblyManager

from crazyflie_driver.srv import UpdateParams
from threading import Thread

from modsim.controller import position_controller, modquad_torque_control
from modsim.trajectory import min_snap_trajectory

from modsim import params
from modsim.params import RunType
from modsim.util.state import init_state, state_to_quadrotor
from modsim.util.thrust import convert_thrust_newtons_to_pwm

from modsim.util.fault_detection import fault_exists_real,      \
                                        real_find_suspects

from dockmgr.datatype.PoseManager import PoseManager
#from dockmgr.datatype.ImuManager import ImuManager

from modquad_sched_interface.interface import convert_modset_to_struc, \
                                              convert_struc_to_mat

import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen

from modquad_sched_interface.simple_scheduler import lin_assign

# Set up for Structure Manager
structure = None
t = 0.0
traj_func = min_snap_trajectory
start_id = 15 # 1 indexed

tlog = []
xlog = []
ylog = []
zlog = []

vxlog = []
vylog = []
vzlog = []

desx = []
desy = []
desz = []

desvx = []
desvy = []
desvz = []

s_thrustlog = []
s_rollog = []
s_pitchlog = []
s_yawlog = []

m_thrustlog = []
m_rollog = []
m_pitchlog = []
m_yawlog = []

desroll  = []
despitch = []
desyaw   = []

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
    global tlog, xlog, ylog, zlog, vxlog, vylog, vzlog
    global desx, desy, desz, desvx, desvy, desvz
    global s_thrustlog, s_rollog, s_pitchlog, s_yawlog # sent
    global m_thrustlog, m_rollog, m_pitchlog, m_yawlog # measured
    global desroll, despitch, desyaw

    # Add to logs
    tlog.append(t)

    xlog.append(state_vector[0])
    ylog.append(state_vector[1])
    zlog.append(state_vector[2])

    # Orientation for a single rigid body is constant throughout the body
    euler = euler_from_quaternion(state_vector[3:7])
    m_rollog.append(math.degrees(euler[0]))
    m_pitchlog.append(math.degrees(euler[1]))
    m_yawlog.append(math.degrees(euler[2]))

    # Quaternion is computed using orientation, so that also doesn't need
    # change

    # Add to logs
    desx.append(desired_state[0][0])
    desy.append(desired_state[0][1])
    desz.append(desired_state[0][2])

    desvx.append(desired_state[1][0])
    desvy.append(desired_state[1][1])
    desvz.append(desired_state[1][2])

    s_thrustlog.append(thrust)
    s_rollog.append(roll)
    s_pitchlog.append(pitch)
    s_yawlog.append(yawrate)

def init_params(speed):
    #rospy.set_param("kalman/resetEstimation", 1)
    rospy.set_param('opmode', 'normal')
    rospy.set_param('structure_speed', speed)
    rospy.set_param('rotor_map', 1) # So that modquad_torque_control knows which mapping to use
    rospy.set_param('is_modquad_sim', False) # For controller.py
    rospy.set_param('is_modquad_bottom_framed', False)
    rospy.set_param('is_modquad_unframed', True)
    rospy.set_param('is_strong_rots', False) # For controller.py
    rospy.loginfo("!!READY!!")
    np.set_printoptions(precision=1)

def update_state(pose_mgr, structure):
    # Convert the individual new states into structure new state
    # As approximant, we let the first modules state be used
    new_states = pose_mgr.get_new_states() # From OPTITRACK
    new_pos = np.array([state[:3] for state in new_states])
    new_pos = np.mean(new_pos, axis=0).tolist()

    # Update position to be centroid of structure
    structure.state_vector = pose_mgr.get_new_state(0)
    structure.state_vector[0] = new_pos[0]
    structure.state_vector[1] = new_pos[1]
    structure.state_vector[2] = new_pos[2]

def check_for_faults(fault_detected, structure):
    global desx, desy, desz

    if len(desx) > 0:
        des_pos = np.array([desx[-1], desy[-1], desz[-1]])
        cur_pos = np.array(structure.state_vector[:3])
    
        # Current pos is where we wanted next position to be
        residual= cur_pos - des_pos
    
        if not fault_detected and fault_exists_real(residual):
            print("FAULT DETECTED! COMMENCE SEARCH!")
            return True
        else:
            return fault_detected
    return False

def check_to_inject_fault(t, fault_injected, structure):
    # Test fault injection
    if t > 10.0 and not fault_injected:
        fault_injected = True
        rid = 0
        structure.single_rotor_toggle(
            [(structure.ids[1], structure.xx[1], structure.yy[1], rid)],
            rot_thrust_cap=0.0
        )
        rospy.loginfo("INJECT FAULT")
    return fault_injected

def run(traj_vars, t_step=0.01, speed=1):
    global tlog, xlog, ylog, zlog, vxlog, vylog, vzlog
    global desx, desy, desz, desvx, desvy, desvz
    global s_thrustlog, s_rollog, s_pitchlog, s_yawlog # sent
    global m_thrustlog, m_rollog, m_pitchlog, m_yawlog # measured

    global t, traj_func, start_id, structure

    freq = 150.0  # 100hz
    rate = rospy.Rate(freq)
    t = 0
    ind = 0
    num_robot = 1

    worldFrame = rospy.get_param("~worldFrame", "/world")
    init_params(speed) # Prints "!!READY!!" to log

    pose_mgr = PoseManager(num_robot, '/vrpn_client_node/modquad', start_id=start_id-1) #0-indexed
    pose_mgr.subscribe()
 
    # 0-indexed start val
    # imu_mgr = ImuManager(3, '/modquad', start_id=start_id-1)
    # imu_mgr.subscribe()

    # Publish here to control
    # crazyflie_controller/src/controller.cpp has been modified to subscribe to
    # this topic, and if we are in the ModQuad state, then the Twist message
    # from mq_cmd_vel will be passed through to cmd_vel
    # TODO: modify so that we publish to all modules in the struc instead of
    # single hardcoded one
    publishers = [ rospy.Publisher('/modquad{:02d}/mq_cmd_vel'.format(mid), Twist, queue_size=100) for mid in range (start_id, start_id + num_robot) ]

    pose_pub = rospy.Publisher("goal", PoseStamped, queue_size=1)

    # Publish to robot
    msg = PoseStamped()

    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 0
    quat = quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]

    # shutdown
    rospy.on_shutdown(landing)

    # Update pose
    rospy.sleep(1)

    # Start by sending NOPs so that we have known start state
    # Useful for debugging and safety
    t = 0
    while t < 3:
        t += 1.0 / freq
        pose_pub.publish(msg)
        if round(t, 2) % 1.0 == 0:
            rospy.loginfo("Sending zeros at t = {}".format(round(t,2)))
        rate.sleep()
        update_state(pose_mgr, structure)

    # Update for the 2x2 structure
    update_att_ki_gains(start_id, num_robot)
    #structure.update_firmware_params()
    #switch_estimator_to_kalman_filter(start_id, num_robot)

    for mid in range(start_id, start_id + num_robot):
        rospy.loginfo(
            "setup complete for /modquad{:02d}".format(mid))

    """
    THIS WILL NOT AUTOMATICALLY CAUSE THE ROBOT TO DO ANYTHING!!
    YOU MUST PAIR THIS WITH MODIFIED CRAZYFLIE_CONTROLLER/SRC/CONTROLLER.CPP
    AND USE JOYSTICK TO SWITCH TO MODQUAD MODE FOR THESE COMMANDS TO WORK
    """
    tstart = rospy.get_time()
    t = 0
    fault_injected = False
    fault_detected = False
    suspects_initd = False
    rospy.loginfo("Start Control")
    while not rospy.is_shutdown() and t < 20.0:
        # Update time
        t = rospy.get_time() - tstart
        #update_state(pose_mgr, structure)

        ## Get new desired state
        desired_state = traj_func(t, speed, traj_vars)


        #des_pos  = desired_state[0]
        #is_pos   = structure.state_vector[:3]
        #residual = des_pos - is_pos

        #suspects = find_suspects(residual)

        # Convert thrust to PWM range
        #thrust = convert_thrust_newtons_to_pwm(thrust_newtons)

        #update_logs(t, structure.state_vector, desired_state, thrust, roll, pitch, yaw)

        #if round(t, 2) % 0.5 == 0:
        #    rospy.loginfo("[{}] {}".format(round(t, 1), 
        #                                np.array([thrust, roll, pitch, yaw])))
        #    rospy.loginfo("     Des={}, Is={}".format(
        #                                    np.array(desired_state[0]), 
        #                                    np.array(structure.state_vector[:3])))
        #    rospy.loginfo("     Fn={}, Fp={}".format( thrust_newtons, thrust ))
        #    rospy.loginfo("")
        #    #if (np.sum(np.array(np.abs(structure.state_vector[:3]))) < 0.5):
        #    #    print("Position is not valid, showing close to origin")
        #    #    break
                
        pose_pub.publish(msg)
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = worldFrame
        msg.pose.position.x = 0
        msg.pose.position.y = 0
        msg.pose.position.z = 0.5
        quat = quaternion_from_euler(0, 0, 0)
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        #fault_injected = check_to_inject_fault(t, fault_injected, structure)

        # Based on residual, get a suspect list
        # if fault_injected and (not fault_detected) and (not suspects_initd):
        #     real_find_suspects(residual, roll, pitch, structure)
            

        # The sleep preserves sending rate
        rate.sleep()

    landing()

    #make_plots()

def make_plots():
    global tlog, xlog, ylog, zlog, vxlog, vylog, vzlog
    global desx, desy, desz, desvx, desvy, desvz
    global s_thrustlog, s_rollog, s_pitchlog, s_yawlog
    global m_rollog, m_pitchlog, m_yawlog

    xlog = np.array(xlog)
    ylog = np.array(ylog)
    zlog = np.array(zlog)
    desx = np.array(desx)
    desy = np.array(desy)
    desz = np.array(desz)

    max_ang = 30

    plt.figure()
    ax0 = plt.subplot(3,3,1)
    ax1 = ax0.twinx()
    ax0.plot(tlog, s_pitchlog, 'c')
    ax0.set_ylabel("Sent Pitch (deg) cyan")
    ax1.plot(tlog, np.zeros((len(tlog), 1)), 'gray', linestyle='--')
    ax1.plot(tlog, xlog, 'b')
    ax1.plot(tlog, desx, 'k')
    ax1.plot(tlog, desx-xlog, 'r')
    ax1.set_ylabel("X (m), blue, black, red")

    ax4 = plt.subplot(3,3,3)
    ax5 = ax4.twinx()
    ax4.plot(tlog, s_pitchlog, 'c')
    ax4.set_ylim((-max_ang, max_ang))
    ax4.set_ylabel("Sent Pitch (deg), cyan")
    ax5.plot(tlog, m_pitchlog, 'm')
    ax5.plot(tlog, np.zeros((len(tlog), 1)), 'gray', linestyle='--')
    ax5.set_ylim((-max_ang, max_ang))
    ax5.set_ylabel("Meas Pitch (deg), magenta")

    ax6 = plt.subplot(3,3,4)
    ax7 = ax6.twinx()
    ax6.plot(tlog, s_rollog, 'c' )
    ax6.set_ylabel("Sent Roll (deg), cyan")
    ax7.plot(tlog, np.zeros((len(tlog), 1)), 'gray', linestyle='--')
    ax7.plot(tlog, ylog, 'b')
    ax7.plot(tlog, desy, 'k')
    ax7.plot(tlog, desy-ylog, 'r')
    ax7.set_ylabel("Y (m), blue, black, red")

    ax10 = plt.subplot(3,3,6)
    ax10.plot(tlog, s_rollog, 'c')
    ax10.set_ylabel("Sent Roll (deg), cyan")
    ax10.set_ylim((-max_ang, max_ang))
    ax11 = ax10.twinx()
    ax11.plot(tlog, m_rollog, 'm')
    ax11.plot(tlog, np.zeros((len(tlog), 1)), 'gray', linestyle='--')
    ax11.set_ylim((-max_ang, max_ang))
    ax11.set_ylabel("Meas Roll (deg), magenta")

    ax12 = plt.subplot(3,3,7)
    ax13 = ax12.twinx()
    ax12.plot(tlog, s_thrustlog, 'c' )
    ax12.set_ylabel("Sent Thrust (PWM), cyan")
    ax13.plot(tlog, np.zeros((len(tlog), 1)), 'gray', linestyle='--')
    ax13.plot(tlog, zlog, 'b')
    ax13.plot(tlog, desz, 'k')
    ax13.plot(tlog, desz-zlog, 'r')
    ax13.set_ylabel("Z (m), blue, black, red")

    ax14 = plt.subplot(3,3,9)
    ax14.set_ylabel("Sent Yaw Rate (deg/s) cyan")
    ax15 = ax14.twinx()
    ax14.plot(tlog, s_yawlog, 'c' )
    ax15.plot(tlog, m_yawlog, 'm' )
    ax15.plot(tlog, np.zeros((len(tlog), 1)), 'gray', linestyle='--')
    ax15.set_ylabel("Meas Yaw (deg) magenta")
    ax15.set_ylim((-30, 30))
    ax14.set_ylim((-205, 205))

    plt.show()
    plt.close()

def landing():
    worldFrame = rospy.get_param("~worldFrame", "/world")
    msg = PoseStamped()

    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame
    msg.pose.position.x = 0
    msg.pose.position.y = 0
    msg.pose.position.z = 0
    quat = quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quat[0]
    msg.pose.orientation.y = quat[1]
    msg.pose.orientation.z = quat[2]
    msg.pose.orientation.w = quat[3]

    pose_pub = rospy.Publisher("goal", PoseStamped, queue_size=1)
    pose_pub.publish(msg)

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

    mset = structure_gen.rect(1, 1)
    lin_assign(mset, start_id=start_mod_id, reverse=True) # 0-indexed
    #print(mset.pi)
    structure = convert_modset_to_struc(mset, start_mod_id)
    structure.state_vector = state_vector
    structure.traj_vars = traj_vars

    # Verify this is the correct structure
    pi = convert_struc_to_mat(structure.ids, structure.xx, structure.yy)
    print("Structure Used: ")
    print("{}".format(pi.astype(np.int64)))

    rospy.on_shutdown(landing)


    # So that modquad_torque_control knows which mapping to use
    rospy.set_param('rotor_map', 2) 

    time.sleep(2)

    run(speed=speed, traj_vars=traj_vars)

if __name__ == '__main__':
    rospy.init_node('real_single_send', anonymous=True)
    print("starting simulation")

    # The place, according to mocap, where robot will start
    x =   0.0 # 6.68 # 4.9#  6.3
    y =   0.0 # 0.64 #-0.9# -1.0
    z =   0.0 # 0.0  # 0.5

    num_struc = 1
    results = test_shape_with_waypts(
                       num_struc, 
                       #waypt_gen.zigzag_xy(2.5, 1.0, 4, start_pt=[x,y,0.2]),
                       #waypt_gen.helix(radius=0.3, 
                       #                rise=1.0, 
                       #                num_circ=2, 
                       #                start_pt=[x, y, 0.0]),
                       waypt_gen.waypt_set([[x+0.0  , y+0.00  , 0.0],
                                            [x+0.0  , y+0.00  , 0.1],
                                            [x+0.0  , y+0.00  , 0.3],
                                            [x+0.0  , y+0.00  , 0.75]
                                            #[x+1  , y    , 0.5]
                                           ]),
                       #waypt_gen.waypt_set([[x    , y    , 0.0],
                       #                     [x    , y    , 0.1],
                       #                     [x    , y    , 0.5],
                       #                     [x    , y    , 0.7],
                       #                     [x    , y+0.1, 0.7],
                       #                     [x+0.5, y+0.1, 0.7],
                       #                     [x+1.0, y+0.1, 0.7],
                       #                     [x+0.5, y-0.1, 0.7],
                       #                     [x    , y    , 0.7],
                       #                     [x    , y+0.1, 0.7],
                       #                     [x+0.5, y+0.1, 0.7],
                       #                     [x+1.0, y+0.1, 0.7],
                       #                     [x+0.5, y-0.1, 0.7],
                       #                     [x    , y    , 0.7],
                       #                     [x    , y+0.1, 0.7],
                       #                     [x+0.5, y+0.1, 0.7],
                       #                     [x+1.0, y+0.1, 0.7],
                       #                     [x+0.5, y-0.1, 0.7],
                       #                     [x    , y    , 0.7],
                       #                     [x    , y    , 0.2]
                       #                    ]
                       #                   ),
                       speed=1.0, test_id="controls", 
                       doreform=True, max_fault=1, rand_fault=False)
    print("---------------------------------------------------------------")
