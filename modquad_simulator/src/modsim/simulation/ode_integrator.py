from scipy.integrate import ode
from modsim.simulation.motion import state_derivative


def simulation_step(structure, state_vector, F, M, time_step, dx):
    """
    Integrates the state of the qudrotor based on the control input.
    :param structure:
    :param state_vector:
    :param F:
    :param M:
    :param time_step:
    :return:
    """
    dx.shape = (13, 1)
    state_vector.shape = (13, 1)
    # ## Derivative of the robot dynamics
    f_dot = lambda s: dx
    #
    # Solve the differential equation of motion
    r = ode(f_dot).set_integrator('dopri5', nsteps=5000)
    r.set_initial_value(state_vector, 0)
    r.integrate(time_step, step=True)

    #r = odeint(dx, state_vector, [time_step, time_step*2])

    if not r.successful():
        print 'Error trying to integrate'
        return None
    state_vector = r.y

    # Simulate floor. Coordinate z in position is always greater than zero.
    if state_vector[2] < 0:
        state_vector[2] = 0.
        # Velocity towards the floor
        if state_vector[5] < 0:
            state_vector[5] = 0.

    return state_vector
