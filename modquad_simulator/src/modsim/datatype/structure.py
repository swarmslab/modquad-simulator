import rospy

import numpy as np
import itertools

from modsim import params

from modquad_simulator.srv import NewParams, NewParamsRequest


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
        # FIXME inertia with parallel axis theorem is not working. Temporary multiplied by zero
        #self.inertia_tensor = self.n * np.array(params.I) + 1.0 * params.mass * np.diag([
        #     np.sum(self.yy ** 2),
        #     np.sum(self.xx ** 2),
        #     np.sum(self.yy ** 2) + np.sum(self.xx ** 2)
        # ])
        self.inertia_tensor = 0.5 * self.n * np.array(params.I) + 0.0 * params.mass * np.diag([
            np.sum(self.yy ** 2),
            np.sum(self.xx ** 2),
            np.sum(self.yy ** 2) + np.sum(self.xx ** 2)
        ])

        self.pos_accumulated_error = np.array([0.0, 0.0, 0.0])
        self.att_accumulated_error = np.array([0.0, 0.0, 0.0])
        self.traj_vars = None    # Populate this
        self.state_vector = [] # Populate this

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
        d = 0.0346
        nc = len(self.xx)  # number of robots in the component

        # Send new parameter to each robot
        print("Need to update firmware for {}".format(self.ids))
        #import pdb; pdb.set_trace()
        for id_robot, xi, yi in list(zip(self.ids, self.xx, self.yy)):
            # Compute P_i
            Sx = np.sign(xi + d * np.array([1, -1, -1, 1]))

            # The minus is added because the crazyflie frame is different.
            Sy = np.sign(yi + d * np.array([-1, -1, 1, 1]))  

            # Send to dynamic attitude parameters
            rospy.loginfo('Wait for service /{}/change_dynamics'.format(id_robot))
            service_name = '/{}/change_dynamics'.format(id_robot)
            rospy.wait_for_service(service_name)
            rospy.loginfo('Found service /{}/change_dynamics'.format(id_robot))

            try:
                change_dynamics = rospy.ServiceProxy(service_name, NewParams)
                msg = NewParamsRequest()
                msg.Cx = 2  # Cx
                msg.Cy = 2  # Cy
                msg.Cz = nc  # Cz

                msg.S_y1 = Sy[0]
                msg.S_y2 = Sy[1]
                msg.S_y3 = Sy[2]
                msg.S_y4 = Sy[3]

                msg.S_x1 = Sx[0]
                msg.S_x2 = Sx[1]
                msg.S_x3 = Sx[2]
                msg.S_x4 = Sx[3]

                rospy.loginfo('Updating attitude params using: ' + service_name)
                change_dynamics(msg)
                rospy.loginfo('Params updated: ' + str(Sx) + ", " + str(Sy))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: {}".format(e))
