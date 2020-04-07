import threading

from vispy import app
import vispy
import numpy as np
from vispy import scene
from vispy.scene import visuals
import vispy.visuals as visualsplot

from modsim.plot.graphicquad import GraphicQuad


class Drawer(object):
    def __init__(self, quads=[], refresh_rate=10):
        self.quads = quads
        self.trajectories = []

        def draw():
            canvas = scene.SceneCanvas(keys='interactive', show=True)
            view = canvas.central_widget.add_view()

            # add a colored 3D axis for orientation
            axis = visuals.XYZAxis(parent=view.scene)

            self.graphic_robots = []
            self.graphic_trajectories = []
            for _ in self.quads:
                self.graphic_robots.append(GraphicQuad(view))

                # Trtajectory
                Plot3D = scene.visuals.create_visual_node(visualsplot.LinePlotVisual)
                pos = np.c_[[0], [0], [0]]
                gtraj = Plot3D(pos, width=2.0, color=(1, 0, 0, .9),
                               face_color=(0., 0., 0, 0.),
                               parent=view.scene)
                self.graphic_trajectories.append(gtraj)

            # Initial drawing for robots
            for i, q in enumerate(self.quads):
                self.graphic_robots[i].draw(q)

            cam = scene.TurntableCamera(elevation=30, azimuth=45)
            cam.set_range((-3, 3), (-3, 3), (-3, 3))
            view.camera = cam

            view.camera = 'arcball'  # or try 'turntable'

            def update(ev):
                # Draw robots
                for i, q in enumerate(self.quads):
                    self.graphic_robots[i].draw(q)

                # Draw trajectories
                for traj, gtraj in zip(self.trajectories, self.graphic_trajectories):
                    x, y, z = traj
                    pos = np.c_[x, y, z]
                    gtraj.set_data(pos, marker_size=0)

            timer = app.Timer(interval=1 / refresh_rate, connect=update, start=True)
            vispy.app.run()

        t = threading.Thread(target=draw)
        t.start()

    def plot(self, quads, trajectories=None):
        self.quads = quads

        if trajectories is not None:
            self.trajectories = trajectories

# Drawer()
