import numpy as np

from modsim import params


class Structure:

    def __init__(self, ids=['modquad01'], xx=[0], yy=[0], motor_failure=[]):
        """

        :param ids: robot ids
        :param xx: module locations in the structure frame (x-coordinates)
        :param yy: module locations in the structure frame (y-coordinates)
        :param motor_failure: motor failures as a set of tuples, (module from 0 to n-1, rotor number from 0 to 3)
        """
        self.ids = ids
        self.xx = xx
        self.yy = yy
        self.motor_failure = motor_failure
        self.motor_roll = [[0, 0, 0, 0], [0, 0, 0, 0]]
        self.motor_pitch = [[0, 0, 0, 0], [0, 0, 0, 0]]

        ##
        self.n = len(self.xx)  # Number of modules
        self.xx = np.array(self.xx) - np.average(self.xx)  # x-coordinates with respect to the center of mass
        self.yy = np.array(self.yy) - np.average(self.yy)  # y-coordinates with respect to the center of mass

        # Equation (4) of the Modquad paper
        # FIXME inertia with parallel axis theorem is not working. Temporary multiplied by zero
        self.inertia_tensor = self.n * np.array(params.I) + 0. * params.mass * np.diag([
            np.sum(self.yy ** 2),
            np.sum(self.xx ** 2),
            np.sum(self.yy ** 2) + np.sum(self.xx ** 2)
        ])

        # self.inertia_tensor = np.array(params.I)
        self.inverse_inertia = np.linalg.inv(self.inertia_tensor)

