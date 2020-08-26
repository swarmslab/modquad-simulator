import numpy as np
from modsim.datatype.quad import Quad
from modsim import params


class Structure:

    def __init__(self, ids=['modquad01'], quads=[Quad()], xx=[0], yy=[0], motor_failure=[]):
        # print len(ids), len(quads), len(xx), len(yy)
        """
        :param ids: robot ids
        :param quads: robot types with tilting propellers
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
        self.quads = quads
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

        # A matrix for the structure
        e3 = np.array([0, 0, 1])
        A_list = [np.zeros([6, 4])]*self.n
        L = params.arm_length
        ctau = params.ctau

        for i in range(self.n):
            # print i
            p = []
            Rp = self.quads[i].getRp()

            # the position of the propellers
            p.append(np.array([self.xx[i] + L, self.yy[i] - L, 0]))
            p.append(np.array([self.xx[i] - L, self.yy[i] - L, 0]))
            p.append(np.array([self.xx[i] - L, self.yy[i] + L, 0]))
            p.append(np.array([self.xx[i] + L, self.yy[i] + L, 0]))

            A_list[i] = np.concatenate((np.vstack([Rp.dot(e3)]*4).T, np.vstack([
                np.cross(p[0], Rp.dot(e3)) + ctau*Rp.dot(e3),
                np.cross(p[1], Rp.dot(e3)) - ctau*Rp.dot(e3),
                np.cross(p[2], Rp.dot(e3)) + ctau*Rp.dot(e3),
                np.cross(p[3], Rp.dot(e3)) - ctau*Rp.dot(e3)]).T), axis=0)

        self.A = np.concatenate(A_list, axis=1)
        self.rankA = np.linalg.matrix_rank(self.A)
