import math

from modsim.util.state import state_to_quadrotor
import numpy as np

# Old error
#accumulated_error = np.array([0., 0., 0.])

en_att_i_gain = False

def enable_attitude_i_gain():
    global en_att_i_gain
    en_att_i_gain = True

def disable_attitude_i_gain():
    global en_att_i_gain
    en_att_i_gain = False

def attitude_controller(structure, control_in, yaw_des):
    global en_att_i_gain
    """
    Attitude controller for crazyflie, receiving pwm as input.
    the output are forces and moments. F_newtons in Newtons
    :type control_in: tuple defined as (F_newtons, roll_des, pitch_des, yawdot_des)
    :param x:
    :return:
    """
    #global accumulated_error
    x = structure.state_vector
    F_newtons = control_in[0]
    roll_des = control_in[1]
    pitch_des = control_in[2]
    yawdot_des = control_in[3]

    ### Moments
    # Quaternion to angles
    quad_state = state_to_quadrotor(x)


    # Where are these numbers from? -> From Nanokontrol Kumar lab repo
    #kpx, kdx, kix = 1.43e-5 * 250, 1.43e-5 * 60, .0002 # ORIGINAL
    if en_att_i_gain:
        num_mod = len(structure.xx)
        kpx, kdx, kix = 1.43e-5 * 250, 1.43e-5 * 60, .0002 / (num_mod/2.5)
    else:
        kpx, kdx, kix = 1.43e-5 * 250, 1.43e-5 * 60, .0000

    e = [max(min(math.radians(roll_des)  - quad_state.euler[0], 0.05), -0.05),
         max(min(math.radians(pitch_des) - quad_state.euler[1], 0.05), -0.05),
         max(min(math.radians(yaw_des)   - quad_state.euler[2], 0.05), -0.05)]

    structure.att_accumulated_error += e
    #print(accumulated_error[0])

    Mx = kpx * e[0] + kdx * (0 - quad_state.omega[0]) + kix * structure.att_accumulated_error[0]
    My = kpx * e[1] + kdx * (0 - quad_state.omega[1]) + kix * structure.att_accumulated_error[1]
    #print(F_newtons, Mx, My)
    #print('---')
    return F_newtons, [Mx, My, 0]
