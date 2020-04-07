import matplotlib.pyplot as plt

plt.ion()


class Plotter(object):
    def __init__(self, min_y=0, max_y=10):
        # Suppose we know the x range


        # Set up plot
        self.figure, self.ax = plt.subplots()
        self.lines, = self.ax.plot([], [], 'o')
        # Autoscale on unknown axis and known lims on the other
        # self.ax.set_autoscaley_on(True)
        self.ax.set_ylim(min_y, max_y)
        # Other stuff
        self.ax.grid()

    def update(self, xdata, ydata):
        # Update data (with the new _and_ the old points)
        self.lines.set_xdata(xdata)
        self.lines.set_ydata(ydata)
        # Need both of these in order to rescale
        self.ax.relim()
        self.ax.autoscale_view()
        # We need to draw *and* flush
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()


def test():
    import numpy as np
    import time

    p = Plotter()

    xdata = []
    ydata = []
    for x in np.arange(0, 10, 0.5):
        xdata.append(x)
        ydata.append(np.exp(-x ** 2) + 10 * np.exp(-(x - 7) ** 2))
        p.update(xdata, ydata)
        time.sleep(1)
