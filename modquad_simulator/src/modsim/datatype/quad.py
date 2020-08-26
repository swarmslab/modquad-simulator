import numpy as np
from modsim import params

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

    def getRp(self):
        """
        :return Rp: the rotation matrix of the tilting propellers
        """
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(self._row), -np.sin(self._row)],
                       [0, np.sin(self._row), np.cos(self._row)]])

        Ry = np.array([[np.cos(self._pitch), 0, np.sin(self._pitch)],
                       [0, 1, 0],
                       [-np.sin(self._pitch), 0, np.cos(self._pitch)]])

        Rp = Ry.dot(Rx)
        return Rp
