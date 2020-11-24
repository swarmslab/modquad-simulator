import rospy

import numpy as np
import itertools

from modsim import params

from modquad_simulator.srv import NewParams, NewParamsRequest
from modquad_simulator.srv import RotorToggle, RotorToggleRequest
from modquad_simulator.srv import SingleRotorToggle, SingleRotorToggleRequest


class Structure:
    max_id = 0

    def __init__(self, ids=['modquad01'], xx=[0], yy=[0], motor_failure=[]):
        """
        :param ids: robot ids
        :param xx: module locations in the structure frame (x-coordinates)
        :param yy: module locations in the structure frame (y-coordinates)
        :param motor_failure: motor failures as a set of tuples, (module from 0 to n-1, rotor number from 0 to 3)
        """
        self.struc_id = Structure.max_id
        Structure.max_id += 1
        self.ids = ids
        self.xx = np.array(xx)
        self.yy = np.array(yy)
        self.motor_failure = motor_failure
        self.motor_roll = [[0, 0, 0, 0], [0, 0, 0, 0]]
        self.motor_pitch = [[0, 0, 0, 0], [0, 0, 0, 0]]
        
        # Previously global params in modquad_sim.py now held in structure
        self.thrust_newtons = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        np.set_printoptions(formatter={'float': lambda x: "{0:0.3f}".format(x)})
        ##
        self.n = len(self.ids)  # Number of modules
        if self.n == 0:
            import sys
            print('-----')
            raise ValueError("ERROR: Cannot make structure of 0 modules")

        # x-coordinates with respect to the center of mass
        self.xx = np.array(self.xx) - np.average(self.xx)

        # y-coordinates with respect to the center of mass
        self.yy = np.array(self.yy) - np.average(self.yy)

        # Equation (4) of the Modquad paper
        self.inertia_tensor = self.n * np.array(params.I) + params.mass * np.diag([
             np.sum(self.yy ** 2),
             np.sum(self.xx ** 2),
             np.sum(self.yy ** 2) + np.sum(self.xx ** 2)
         ])

        self.pos_accumulated_error = np.array([0.0, 0.0, 0.0])
        self.att_accumulated_error = np.array([0.0, 0.0, 0.0])
        self.traj_vars         = None # Populate this
        self.state_vector      = []   # Populate this
        self.prev_state_vector = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0]) # Populate this

        # error in [x,y,z,phi,theta,yaw]
        self.error      = np.array([0,0,0,0]) # Populate this
        self.prev_error = np.array([0,0,0,0]) # Populate this
        self.prev_time  = 0

        # self.inertia_tensor = np.array(params.I)
        try:
            self.inverse_inertia = np.linalg.inv(self.inertia_tensor)
        except:
            print("Inverse inertia calculation error: {}".format(
                    sys.exc_info()[0]))
            print(self.inertia_tensor)
            print("There are {} robots in structure".format(self.n))
            print("Inertia param is {}".format(params.I))
            print(self.ids)
            print(self.xx)
            print(self.yy)
            print(self.motor_failure)
            raise Exception("Inverting inertia exception")

    def gen_hashstring(self, en_fail_motor=True):
        """ 
        This is for reconfig, where we need to determine whether to split structure based on
        the shape of it and the faults it contains
        """
        # Import here in case something else using structure does not need mqscheduler package
        from modquad_sched_interface.interface import convert_struc_to_mat

        pi = convert_struc_to_mat([int(mid[7:]) for mid in self.ids], self.xx, self.yy)
        R = [r for r in range(pi.shape[0])]
        C = [c for c in range(pi.shape[1])]
        RC = [p for p in itertools.product(R,C)]
        shape2 = []
        for r in R:
            for c in C:
                if pi[r,c] != -1:
                    shape2.append('1')
                else:
                    shape2.append('0')
            if r < R[-1]:
                shape2.append(';')
        shape = ''.join(shape2)
        # NOTE: range(4) should be range(num_rotor) for however many rotors the system has, we just use quadrotors
        rotorstat = ''
        if en_fail_motor:
            rotorstat = ','.join(
                            ''.join('%d' % int((int(mod[7:]), rot) not in self.motor_failure) 
                            for rot in range(4)) 
                        for mod in sorted(self.ids))
        else:
            return shape
        return shape + '_' + rotorstat

    def update_control_params(self, thrust_newtons, roll, pitch, yaw):
        self.thrust_newtons = thrust_newtons
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def update_firmware_params(self):
        rospy.loginfo("Starting firmware param update")
        # Intrinsic parameters for Cx, Cy, and Cz.
        d = 0.0346 # manhattan distance from center of module mass to rotors (m)
        nc = self.n  # number of robots in the structure

        # Send new parameter to each robot
        rospy.loginfo("Need to update firmware for {}".format(self.ids))
        rospy.loginfo("----")
        rospy.loginfo("Mods in struc")
        rospy.loginfo(self.ids)
        rospy.loginfo(self.xx)
        rospy.loginfo(self.yy)
        for id_robot, xi, yi in list(zip(self.ids, self.xx, self.yy)):
            # Compute P_i 
            # (pitch)
            Sx = np.sign(yi + d * np.array([1, -1, -1, 1]))
            rospy.loginfo("{}: xi = {}, new_Sx = {}".format(id_robot, xi, Sx))
            # modquad13:  0.058 + [0.0346, -0.0346, -0.0346, 0.0346]
            # modquad14: -0.058 + [0.0346, -0.0346, -0.0346, 0.0346]

            # The minus is added because the crazyflie frame is different.
            # (roll)
            Sy = np.sign(-xi + d * np.array([-1, -1, 1, 1]))  
            rospy.loginfo("{}: yi = {}, new_Sy = {}".format(id_robot, yi, Sy))
            # modquad13: 0.0 + [-0.0346, -0.0346, 0.0346, 0.0346]

            # Send to dynamic attitude parameters
            rospy.loginfo('Wait for service /{}/change_dynamics'.format(id_robot))
            service_name = '/{}/change_dynamics'.format(id_robot)
            rospy.wait_for_service(service_name)
            rospy.loginfo('Found service /{}/change_dynamics'.format(id_robot))

            try:
                change_dynamics = rospy.ServiceProxy(service_name, NewParams)
                msg = NewParamsRequest()

                # Update...?
                msg.Cx = 1 #2  # Cx - unused
                msg.Cy = 1 #2  # Cy - unused
                msg.Cz = 1 #nc  # Cz

                # Update roll constants
                msg.S_y1 = Sy[0]
                msg.S_y2 = Sy[1]
                msg.S_y3 = Sy[2]
                msg.S_y4 = Sy[3]

                # Update pitch constants
                msg.S_x1 = Sx[0]
                msg.S_x2 = Sx[1]
                msg.S_x3 = Sx[2]
                msg.S_x4 = Sx[3]

                rospy.loginfo('Updating attitude params using: ' + service_name)
                change_dynamics(msg)
                rospy.loginfo('Params updated: PITCH:' + str(Sx) + ", ROLL:" + str(Sy))

                rospy.loginfo("change_dynamics update PITCH = {}".format(
                    [msg.S_x1, msg.S_x2, msg.S_x3, msg.S_x4]
                ))
                rospy.loginfo("change_dynamics update ROLL = {}".format(
                    [msg.S_y1, msg.S_y2, msg.S_y3, msg.S_y4]
                ))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))

    def toggle_rotors(self, rot_list, disable_rots):

        """
        :param rot_list: list of tuples of ("modquadXY", x_pos, y_pos, rotor_id) to (dis/en)able
        :param disable_rots: if True, disable rotors in rot_list, else enable
        
        Crazyflie Axes are different!!
            x^ FWD OF CF
             |
        <--------> y RIGHT SIDE OF CF
             |
             v
        """
        if type(disable_rots) is not bool:
            raise Exception("Expect disable_rots to be boolean, was {}".format(
                                                            type(disable_rots)))
        if (len(rot_list) == 0):
            rospy.loginfo("No rotors in list to (dis/en)able")
            return

        if disable_rots:
            rospy.loginfo("Structure DISabling rotors {}".format(rot_list))
        else:
            rospy.loginfo("Structure ENabling rotors {}".format(rot_list))

        # Send new parameter set to each robot
        for id_robot, xi, yi, rid in rot_list:
            enables = [0 if disable_rots else 1 for i in range(4)]

            # Send to dynamic attitude parameters
            rospy.loginfo('Wait for service /{}/toggle_rotors'.format(id_robot))
            service_name = '/{}/toggle_rotors'.format(id_robot)
            rospy.wait_for_service(service_name)
            rospy.loginfo('Found service /{}/toggle_rotors'.format(id_robot))

            try:
                toggle_rotors = rospy.ServiceProxy(service_name, RotorToggle)
                msg = RotorToggleRequest()

                # Update rotor toggle constants
                msg.en_r1 = enables[0]
                msg.en_r2 = enables[1]
                msg.en_r3 = enables[2]
                msg.en_r4 = enables[3]

                rospy.loginfo('Updating attitude params using: ' + service_name)
                toggle_rotors(msg)
                rospy.loginfo('RotTog:' + str(enables))

                rospy.loginfo("toggle_rotors update R1-4 = {}".format(
                    [msg.en_r1, msg.en_r2, msg.en_r3, msg.en_r4]
                ))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))

    def single_rotor_toggle(self, rot_list, rot_thrust_cap):

        """
        :param rot_list: list of tuples of ("modquadXY", x_pos, y_pos, rotor_id) to (dis/en)able
        :param rot_thrust_cap: if True, change thrust cap
        
        Crazyflie Axes are different!!
            x^ FWD OF CF
             |
        <--------> y RIGHT SIDE OF CF
             |
             v
        """
        if rot_thrust_cap < 0 or rot_thrust_cap > 1:
            raise Exception("rot_thrust_cap range [0,1], was {}".format(rot_thrust_cap))

        if (len(rot_list) == 0):
            rospy.loginfo("No rotors in list to (dis/en)able")
            return

        # if not enable_rots:
        #     rospy.loginfo("Structure DISabling rotors {}".format(rot_list))
        # else:
        #     rospy.loginfo("Structure ENabling rotors {}".format(rot_list))

        # Send new parameter set to each robot
        for id_robot, xi, yi, rid in rot_list:
            # Send to dynamic attitude parameters
            #rospy.loginfo('Wait for service /{}/toggle_single_rotor'.format(id_robot))
            service_name = '/{}/toggle_single_rotor'.format(id_robot)
            #rospy.wait_for_service(service_name)
            #rospy.loginfo('Found service /{}/toggle_single_rotor'.format(id_robot))

            try:
                toggle_single_rotor = rospy.ServiceProxy(service_name, SingleRotorToggle)
                msg = SingleRotorToggleRequest()

                # Update rotor toggle constants
                msg.thrust_cap = rot_thrust_cap
                msg.rotor_id = rid + 1 # 0-index to 1-index

                rospy.loginfo('Toggle Single Rotor with: ' + service_name)
                toggle_single_rotor(msg)
                rospy.loginfo("toggle_single_rotor: new cap {} -> R{}".format(
                                msg.thrust_cap, msg.rotor_id ) )
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))
