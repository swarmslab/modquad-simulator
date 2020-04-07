import numpy as np
from vispy import scene
from vispy.scene import visuals
import vispy.visuals as visualsplot

from modsim import params
from math import sqrt

from transforms3d import euler


class GraphicQuad(object):
    def __init__(self, view):
        self.view = view
        Plot3D = scene.visuals.create_visual_node(visualsplot.LinePlotVisual)

        comp_points = {}

        Lx = 1 * params.arm_length * sqrt(2) / 2
        cage_width = params.cage_width
        motors = np.array([[Lx, Lx, -Lx, -Lx], [Lx, -Lx, -Lx, Lx], [0, 0, 0, 0]]).T

        # Arms
        points_arm1 = motors[:3:2, :]
        arm1 = Plot3D(points_arm1, width=4.0, color='blue',
                      face_color=(0., 0., 0, 0.),
                      parent=view.scene)

        comp_points[arm1] = points_arm1

        points_arm2 = motors[1:4:2, :]
        arm2 = Plot3D(points_arm2, width=4.0, color='blue',
                      face_color=(0., 0., 0, 0.),
                      parent=view.scene)

        comp_points[arm2] = points_arm2

        # Motors
        motor_points = visuals.Markers()
        motor_points.set_data(motors, edge_color=None, face_color=(1, 1, 1, .5), size=15)
        view.add(motor_points)
        comp_points[motor_points] = motors

        # Front tag
        # Centroid
        scatter = visuals.Markers()
        pos = np.array([[Lx / 2], [0], [0]]).T
        scatter.set_data(pos, edge_color=None, face_color=(1, 0, 0, .5), size=10)
        view.add(scatter)
        comp_points[scatter] = pos

        # Cage down side
        l2c = cage_width / abs(Lx) / 2  # Lx to cage width
        cage_color = (0, 1, 0, .5)
        cage_d_points = np.vstack((motors, motors[0, :])) * l2c
        cage_d_points[:, 2] -= .3 * Lx
        cage_d = Plot3D(cage_d_points, width=1.0, color=cage_color,
                        face_color=(0., 0., 0, 0.),
                        parent=view.scene, marker_size=0)
        comp_points[cage_d] = cage_d_points
        # Cage top
        cage_t_points = np.vstack((motors, motors[0, :])) * l2c
        cage_t_points[:, 2] += .7 * Lx
        cage_t = Plot3D(cage_t_points, width=1.0, color=cage_color,
                        face_color=(0., 0., 0, 0.),
                        parent=view.scene, marker_size=0)
        comp_points[cage_t] = cage_t_points

        # lageral
        for i in range(4):
            cage_li_points = np.vstack((l2c * motors[i, :] - np.array([[0, 0, .3 * Lx], [0, 0, .3 * Lx]]),
                                        l2c * motors[i, :] + np.array([[0, 0, .7 * Lx], [0, 0, .7 * Lx]])))
            cage_li = Plot3D(cage_li_points, width=1.0, color=cage_color,
                             face_color=(0., 0., 0, 0.),
                             parent=view.scene, marker_size=0)

            comp_points[cage_li] = cage_li_points

        self._comp_points = comp_points


    def draw(self, quad):
        location = quad.pos

        # Rotation Matrix
        R = euler.euler2mat(quad.euler[0], quad.euler[1], quad.euler[2])

        for comp, points in self._comp_points.iteritems():
            # transform points
            tpoints = []
            for p in points:
                # Rotate
                trans_p = np.dot(R, p)

                # Translate
                trans_p += np.array(location)

                tpoints.append(trans_p)

            if isinstance(comp, visualsplot.LinePlotVisual):
                comp.set_data(np.array(tpoints), marker_size=0)
            else:
                comp.set_data(np.array(tpoints))
