import numpy as np
from modsim import params
import math

class Quad:
    """
    quadrotor type that has tilted propellers specified with row and pitch angles
    """
    def __init__(self, row=0, pitch=0, id="modquad01", failure=[]):
        """
        :param row: row angle of the propellers
        :param pitch: pitch angle of the propellers
        :param id: robot id
        :param failure: rotor failure indices
        """
        self._row = row
        self._pitch = pitch
        self._id = id
        self._failure = failure

        Rx = np.array([[1, 0, 0],
                       [0, math.cos(self._row), -math.sin(self._row)],
                       [0, math.sin(self._row), math.cos(self._row)]])

        Ry = np.array([[math.cos(self._pitch), 0, math.sin(self._pitch)],
                       [0, 1, 0],
                       [-math.sin(self._pitch), 0, math.cos(self._pitch)]])
        self._Rp = Ry.dot(Rx)
        #
        # print Rx.dot(np.array([0,0,1]))
        # print Ry.dot(np.array([0, 0, 1]))

    def getRp(self):
        """
        :return Rp: the rotation matrix of the tilting propellers
        """
        return self._Rp
