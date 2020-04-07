import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


class Drawer(object):
    def __init__(self):
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection='3d')
        plt.ion()
        plt.show()

        self.counter = 0

    def plot(self, x, y, z, traj_x=[], traj_y=[], traj_z=[]):
        self.counter += 1
        if not self.counter % 10:
            return

        # self.ax.cle
        self.ax.plot([-5, 5], [-5, 5], [-5, 5], '.')
        self.ax.plot(traj_x, traj_y, traj_z, '-')
        self.ax.plot([x], [y], [z], 'o')
        plt.draw()
        plt.pause(0.00001)
        self.ax.cla()
