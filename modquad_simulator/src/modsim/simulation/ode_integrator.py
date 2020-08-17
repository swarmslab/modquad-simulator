from scipy.integrate import ode
from modsim.simulation.motion import state_derivative


def simulation_step(structure, state_vector, F, M, time_step):
    """
    Integrates the state of the qudrotor based on the control input.
    :param structure:
    :param state_vector:
    :param F:
    :param M:
    :param time_step:
    :return:
    """
    ## Derivative of the robot dynamics
    f_dot = lambda t1, s: state_derivative(s, F, M, structure)

    # Solve the differential equation of motion
    r = ode(f_dot).set_integrator('dopri5', nsteps=5000)
    r.set_initial_value(state_vector, 0)
    r.integrate(time_step, step=True)
    if not r.successful():
        print('Error trying to integrate')
        return None
    state_vector = r.y

    # Simulate floor. Coordinate z in position is always greater than zero.
    if state_vector[2] < 0:
        state_vector[2] = 0.
        # Velocity towards the floor
        if state_vector[5] < 0:
            state_vector[5] = 0.

    # Return the new state vector and the derivative used to arrive at it
    return state_vector
