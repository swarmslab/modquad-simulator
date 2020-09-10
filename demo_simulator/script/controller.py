#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty
from geometry_msgs.msg import Twist

from modsim.controller import position_controller
from modsim.trajectory import min_snap_trajectory, circular_trajectory

from modsim import params
from modsim.params import RunType
from modsim.util.state import init_state, state_to_quadrotor

from dockmgr.datatype.OdometryManager import OdometryManager

import modquad_sched_interface.waypt_gen as waypt_gen
import modquad_sched_interface.structure_gen as structure_gen
from modquad_sched_interface.simple_scheduler import lin_assign
from modquad_sched_interface.interface import convert_modset_to_struc, \
                                              convert_struc_to_mat


# Control input callback
def convert_to_thrust_pwm(thrust_newtons):
    # To prevent imaginary thrust
    if thrust_newtons < 0:
        thrust_newtons = 0

    F_g = thrust_newtons * (1000.0 / 9.81)
    thrust_pwm = 60000.0 * ((math.sqrt(F_g + c3) * c2) + c1)

def goal_to_pose(x, y, z, yaw):
    goal = PoseStamped()
    goal.header.seq = 0
    goal.header.frame_id = '/world'

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z
    quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
    goal.pose.orientation.x = quaternion[0]
    goal.pose.orientation.y = quaternion[1]
    goal.pose.orientation.z = quaternion[2]
    goal.pose.orientation.w = quaternion[3]
    return goal



class Controller():
    def __init__(self, use_controller, joy_topic):
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

        if use_controller:
            rospy.loginfo("waiting for land service")
            rospy.wait_for_service('land')
            rospy.loginfo("found land service")
            self._land = rospy.ServiceProxy('land', Empty)

            rospy.loginfo("waiting for takeoff service")
            rospy.wait_for_service('takeoff')
            rospy.loginfo("found takeoff service")
            self._takeoff = rospy.ServiceProxy('takeoff', Empty)
        else:
            self._land = None
            self._takeoff = None

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)

        rospy.on_shutdown(self.land)

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1 and self._land != None:
                    rospy.loginfo("Request land")
                    self._land()
                if i == 1 and data.buttons[i] == 1:
                    self._emergency()
                if i == 2 and data.buttons[i] == 1 and self._takeoff != None:
                    rospy.loginfo("Request takeoff")
                    self._takeoff()
                if i == 4 and data.buttons[i] == 1:
                    self.circular_motion()
                    pass

        self._buttons = data.buttons

    def circular_motion():
        # Generate a trajectory
        x =  6.7
        y = -1.0
        z =  1.5

        mset = structure_gen.rect(1, 1), 
        wayptset = waypt_gen.waypt_set([[x,y,0],[x,y,z],
                             [x,y,z-1],[x,y-0.5,z-0.5],
                             [x+1,y,z],[x,y+1,z],
                             [x,y,z],[x,y,0.1]     ]
                           ),

        state_vector = init_state(loc, 0)

        # Generate the structure
        lin_assign(mset)
        struc1 = convert_modset_to_struc(mset)
        struc1.state_vector = state_vector
        struc1.traj_vars = traj_vars

        trajectory_function = min_snap_trajectory
        speed = .5
        traj_vars = trajectory_function(0, speed, None, wayptset)
        loc=[x,y,0]
        state_vector = init_state(loc, 0)
    
        publisher = rospy.Publisher('/modquad13/cmd_vel', Twist, queue_size=1)

        # To read new state
        odom_mgr = OdometryManager(1, '/modquad', start_id=12) # 0-indexed start val
        odom_mgr.subscribe()

        # Publish to robot
        msg = Twist()

        # Time counter
        t = 1.
        s = 100.

        start = rospy.Time.now()
        t = 0

        # Circle loop
        while not rospy.is_shutdown():
            theta = t / s + i * 2 * math.pi / n
            cur_state = odom_mgr.get_new_state(0)
    
            desired_state = trajectory_function(t, speed, structure.traj_vars)

            # Get new control inputs
            [thrust_newtons, roll, pitch, yaw] = \
                    position_controller(struc1, desired_state)

            thrust_pwm = convert_to_thrust_pwm(thrust_newtons)

            msg.linear.x = roll # roll [-30, 30] deg
            msg.linear.y = pitch # pitch [-30, 30] deg
            msg.linear.z = thrust_pwm # Thrust ranges 10000 - 60000
            msg.angular.z = yaw # yaw rate

            publisher.publish(msg)

            t += 1
            rospy.sleep(.1)

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    use_controller = rospy.get_param("~use_crazyflie_controller", False)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    rospy.loginfo("Starting controller")
    rospy.set_param("/modquad13/commander/enHighLevel", 0)
    controller = Controller(use_controller, joy_topic)
    rospy.spin()
