#!/usr/bin/env python
from time import sleep

from modsim.attitude import geo_attitude_controller
from modsim.controller import geo_position_controller, modquad_torque_control
from modsim.trajectory import circular_trajectory

from modsim.util.state import init_state
from modsim.datatype.quad import Quad

import numpy as np

from modsim.simulation.ode_integrator import simulation_step

from modsim import params
from modsim.datatype.structure import Structure
import matplotlib.pyplot as plt


def simulate(structure, trajectory_function, t_step=0.005, tmax=5, loc=[1., .0, .0]):
    """

    :param structure:
    :param trajectory_function:
    :param t_step: time step
    :param tmax: maximum time
    """
    state_vector = init_state(loc, 0)
    state_log = []
    forces_log = []

    # For every time step
    for t in np.arange(0, tmax, t_step):
        # print t, state_vector
        ##### Trajectory
        desired_state = trajectory_function(t % 10, tmax)
        # Controller for a single robot
        f_des = geo_position_controller(state_vector, desired_state)
        tau_des = geo_attitude_controller(f_des, state_vector, desired_state)

        # Structure control
        F_structure, M_structure, rotor_forces = modquad_torque_control(f_des, tau_des, state_vector, structure)
        forces_log.append(rotor_forces)

        # Simulate
        new_state_vector = simulation_step(structure, state_vector, F_structure, M_structure, t_step)
        if new_state_vector is None:
            break
        state_vector = new_state_vector
        state_log.append(np.copy(state_vector))

    print "end simulation"
    state_log = np.array(state_log)
    # Show trajectory x-y
    plt.plot(state_log[:, 0], state_log[:, 1])
    plt.grid()
    plt.show()

    # sum of the squared forces
    plt.plot(np.sum(np.array(forces_log) ** 2, axis=1))
    plt.grid()
    plt.show()

    print "total integral=", np.sum(np.array(forces_log) ** 2) * t_step


if __name__ == '__main__':
    structure = Structure(ids=['modquad01', 'modquad02'], quads=[Quad()]*2, xx=[0, params.cage_width], yy=[0, 0], motor_failure=[])

    # w = params.cage_width
    # structure = Structure(ids=['1', '2', '3', '4'], xx=[0., 0., -w, -w], yy=[0., -w, -w, 0.])
    # structure = Structure()
    structure4 = Structure(ids=['modquad01', 'modquad02', 'modquad03', 'modquad04'],
                           quads=[Quad(), Quad(), Quad(), Quad()],
                           xx=[0, params.cage_width, 0, params.cage_width],
                           yy=[0, 0, params.cage_width, params.cage_width],
                           motor_failure=[(1, 2)])

    trajectory_function = circular_trajectory

    simulate(structure4, trajectory_function)
