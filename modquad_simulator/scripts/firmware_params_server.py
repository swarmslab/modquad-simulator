#!/usr/bin/env python

import sys
import rospy

from std_srvs.srv import Empty, EmptyResponse

from crazyflie_driver.srv import UpdateParams

from modquad_simulator.srv import NewParams, NewParamsResponse
from modquad_simulator.srv import RotorToggle, RotorToggleResponse
from modquad_simulator.srv import SingleRotorToggle, SingleRotorToggleResponse
from modquad_simulator.srv import SetMotors, SetMotorsResponse

def handle_switch_to_kalman(msg):
    rospy.set_param("stabilizer/estimator", 2)
    try:
        update_params = rospy.ServiceProxy('update_params', UpdateParams)
        success = update_params( ['stabilizer/estimator'] )

        # THIS MUST SUCCEED
        if not success:
            rospy.loginfo("Setting estimator to Kalman failed!")
            assert success
        else:
            rospy.loginfo('Switched to Kalman Successfully')
        return EmptyResponse()

    except rospy.ServiceException as e:
        rospy.logerr(
            "Service Switch to Kalman firmware update failed: {}".format(e))

# Crazyflie dynamics parameters
def handle_zero_att_i_gains(msg):
    # msg is Empty, just used as a trigger
    rospy.set_param('pid_attitude/roll_ki', 0)
    rospy.set_param('pid_attitude/pitch_ki', 0)
    rospy.set_param('pid_attitude/yaw_ki', 0)

    rospy.set_param('pid_rate/roll_ki', 0)
    rospy.set_param('pid_rate/pitch_ki', 0)
    rospy.set_param('pid_rate/yaw_ki', 0)

    #rospy.set_param('ctrlINDI/yaw_kp', 0)

    rospy.loginfo("Wait for update_params service")
    rospy.wait_for_service('update_params')
    rospy.loginfo("Found update_params service")

    try:
        update_params = rospy.ServiceProxy('update_params', UpdateParams)
        success = update_params(
                    ['pid_attitude/roll_ki', 'pid_attitude/pitch_ki',
                     'pid_attitude/yaw_ki' , 'pid_rate/roll_ki',
                     'pid_rate/pitch_ki'   , 'pid_rate/yaw_ki',
                    ] )

        # THIS MUST SUCCEED
        if not success:
            rospy.loginfo("Setting I-Gains to 0 failed!")
            assert success
        else:
            rospy.loginfo('Attitude I-Gains Zeroed Successfully')
        return EmptyResponse()

    except rospy.ServiceException as e:
        rospy.logerr(
            "Service Zero Attitude I-Gains firmware update failed: {}".format(e))

def handle_change_single_rot_en(msg):
    rospy.loginfo("Changing rotor enable params")

    thrust_cap = msg.thrust_cap
    rot_id = msg.rotor_id

    rospy.set_param('rotor_toggle/enable_r{}'.format(rot_id), thrust_cap)

    #if perform_enable:
    #    rospy.loginfo("Request enable R{}".format(rot_id))
    #else:
    #    rospy.loginfo("Request disable R{}".format(rot_id))

    #rospy.loginfo("Wait for update_params service")
    #rospy.wait_for_service('update_params')
    #rospy.loginfo("Found update_params service")

    try:
        update_params = rospy.ServiceProxy('update_params', UpdateParams)
        success = update_params( ['rotor_toggle/enable_r{}'.format(rot_id)] )

        # THIS MUST SUCCEED
        if not success:
            print("Rotor toggling for single rotor in ModQuad structure failed!")
            assert success
        return SingleRotorToggleResponse()

    except rospy.ServiceException as e:
        rospy.logerr(
            "Service SingleRotorToggle firmware update failed: {}".format(e))

    return SingleRotorToggleResponse()

def handle_change_rot_en(msg):
    rospy.loginfo("Changing rotor enable params")
    en1 = msg.en_r1
    en2 = msg.en_r2
    en3 = msg.en_r3
    en4 = msg.en_r4
    rospy.set_param('rotor_toggle/enable_r1', en1)
    rospy.set_param('rotor_toggle/enable_r2', en2)
    rospy.set_param('rotor_toggle/enable_r3', en3)
    rospy.set_param('rotor_toggle/enable_r4', en4)

    rospy.loginfo("Request en1 = {}".format(en1))
    rospy.loginfo("Request en2 = {}".format(en2))
    rospy.loginfo("Request en3 = {}".format(en3))
    rospy.loginfo("Request en4 = {}".format(en4))

    rospy.loginfo("Wait for update_params service")
    rospy.wait_for_service('update_params')
    rospy.loginfo("Found update_params service")

    try:
        update_params = rospy.ServiceProxy('update_params', UpdateParams)
        success = update_params(
           ['rotor_toggle/enable_r1', 'rotor_toggle/enable_r2', 
            'rotor_toggle/enable_r3', 'rotor_toggle/enable_r4']
        )

        # THIS MUST SUCCEED
        if not success:
            print("Rotor toggling for ModQuad structure failed!")
            assert success
        return RotorToggleResponse()

    except rospy.ServiceException as e:
        rospy.logerr("Service RotorToggle firmware update failed: %s" % e)

    return RotorToggleResponse()

def handle_change_dym(msg):
    rospy.loginfo("change_dynamics service callback initiated")
    P1 = msg.S_x1 # * msg.Cy
    P2 = msg.S_x2 # * msg.Cy
    P3 = msg.S_x3 # * msg.Cy
    P4 = msg.S_x4 # * msg.Cy
    R1 = msg.S_y1 # * msg.Cx
    R2 = msg.S_y2 # * msg.Cx
    R3 = msg.S_y3 # * msg.Cx
    R4 = msg.S_y4 # * msg.Cx
    # T=5000

    rospy.set_param('var/pitch1', P1)
    rospy.set_param('var/pitch2', P2)
    rospy.set_param('var/pitch3', P3)
    rospy.set_param('var/pitch4', P4)
    rospy.set_param('var/roll1', R1)
    rospy.set_param('var/roll2', R2)
    rospy.set_param('var/roll3', R3)
    rospy.set_param('var/roll4', R4)
    rospy.set_param('var/czz', 1.2)
    # rospy.set_param('/crazy01/var/thrust', T)

    rospy.loginfo("Wait for update_params service")
    rospy.wait_for_service('update_params')
    rospy.loginfo("Found update_params service")
    # print("change_dynamics updates PITCH = {}".format([P1, P2, P3, P4]))
    # print("change_dynamics updates ROLL  = {}".format([R1, R2, R3, R4]))
    try:
        update_params = rospy.ServiceProxy('update_params', UpdateParams)
        success = update_params(
           ['var/pitch1', 'var/pitch2', 'var/pitch3', 'var/pitch4',
            'var/roll1' , 'var/roll2' , 'var/roll3' , 'var/roll4' ,
            'var/czz'
           ])

        # THIS MUST SUCCEED
        if not success:
            print("Param update for ModQuad structure failed!")
            assert success
        return NewParamsResponse()

    except rospy.ServiceException as e:
        rospy.logerr("Service firmware update failed: %s" % e)

    return NewParamsResponse()


def set_motors_srv(msg):
    # time =

    rospy.set_param('motorPowerSet/m1', msg.m1)
    rospy.set_param('motorPowerSet/m2', msg.m2)
    rospy.set_param('motorPowerSet/m3', msg.m3)
    rospy.set_param('motorPowerSet/m4', msg.m4)
    rospy.set_param('motorPowerSet/motor_timer', msg.timer)

    # rospy.wait_for_service('update_params')
    try:
        update_params = rospy.ServiceProxy('update_params', UpdateParams)
        update_params(['motorPowerSet/m1', 'motorPowerSet/m2',
'motorPowerSet/m3', 'motorPowerSet/m4'])
        update_params(['motorPowerSet/motor_timer'])
        return SetMotorsResponse()
    except rospy.ServiceException as e:
        rospy.logerr("Service motor failed: %s" % e)

    return SetMotorsResponse()


def acquire_params():
    rospy.init_node('firmware_params_server')

    rospy.loginfo("Advertise zero_att_i_gains service")
    rospy.Service('zero_att_i_gains', Empty, handle_zero_att_i_gains)

    rospy.loginfo("Advertise switch_to_kalman_filter service")
    rospy.Service('switch_to_kalman_filter', Empty, handle_switch_to_kalman)

    rospy.loginfo("Advertise change_dynamics service")
    rospy.Service('change_dynamics', NewParams, handle_change_dym)

    rospy.loginfo("Advertise toggle_rotors service")
    rospy.Service('toggle_rotors', RotorToggle, handle_change_rot_en)

    rospy.loginfo("Advertise toggle_single_rotor service")
    rospy.Service('toggle_single_rotor', SingleRotorToggle, handle_change_single_rot_en)

    rospy.loginfo("Advertise motor_timer service")
    rospy.Service('motor_timer', SetMotors, set_motors_srv)

    rospy.spin()


if __name__ == '__main__':
    acquire_params()
