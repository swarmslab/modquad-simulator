#!/usr/bin/env python



from modsim.controller import control_handle
from modsim.trajectory import circular_trajectory

from modsim.simulation.motion import control_output, modquad_torque_control
from modsim.util.state import init_state

import numpy as np

from modsim.simulation.ode_integrator import simulation_step

from modsim import params
from modsim.datatype.structure import Structure


def simulate(structure, trajectory_function, t_step = 0.01,tmax = 5, loc=[0.,.0,.0]):
    """

    :param structure:
    :param trajectory_function:
    :param t_step: time step
    :param tmax: maximum time
    """
    freq = 1./t_step  # frequency
    state_vector = init_state(loc, 0)
    state_log = []
    forces_log = []

    # For every time step
    for t in np.arange(0, tmax, t_step):
        print t, state_vector[0]
        ##### Trajectory
        desired_state = trajectory_function(t % 10, tmax)
        # Position controller for a single robot
        F, M = control_output(t, state_vector, desired_state, control_handle)
        # Structure control
        F_structure, M_structure, rotor_forces = modquad_torque_control(F, M, structure)
        forces_log.append(rotor_forces)

        # Simulate
        state_vector = simulation_step(structure, state_vector, F_structure, M_structure, freq)
        state_log.append(np.copy(state_vector))

    print "end simulation"





if __name__ == '__main__':
    structure = Structure(ids=['modquad01', 'modquad02'], xx=[0, -params.cage_width], yy=[0, 0])
    trajectory_function = circular_trajectory

    simulate(structure, trajectory_function)