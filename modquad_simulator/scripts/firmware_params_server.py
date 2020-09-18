#!/usr/bin/env python

import sys
import rospy
from crazyflie_driver.srv import UpdateParams
from modquad_simulator.srv import NewParams, NewParamsResponse
from modquad_simulator.srv import SetMotors, SetMotorsResponse


# Crazyflie dynamics parameters


def handle_change_dym(msg):
    rospy.loginfo("change_dynamics service callback initiated")
    P1 = msg.S_x1 * msg.Cy
    P2 = msg.S_x2 * msg.Cy
    P3 = msg.S_x3 * msg.Cy
    P4 = msg.S_x4 * msg.Cy
    R1 = msg.S_y1 * msg.Cx
    R2 = msg.S_y2 * msg.Cx
    R3 = msg.S_y3 * msg.Cx
    R4 = msg.S_y4 * msg.Cx
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
    try:
        update_params = rospy.ServiceProxy('update_params', UpdateParams)
        update_params(
            ['var/pitch1', 'var/pitch2', 'var/pitch3', 'var/pitch4',
             'var/roll1' , 'var/roll2' , 'var/roll3' , 'var/roll4' ,
             'var/czz'
            ])
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

    rospy.loginfo("Advertise change_dynamics service")
    rospy.Service('change_dynamics', NewParams, handle_change_dym)

    rospy.loginfo("Advertise motor_timer service")
    rospy.Service('motor_timer', SetMotors, set_motors_srv)

    rospy.spin()


if __name__ == '__main__':
    acquire_params()
