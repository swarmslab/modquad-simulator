import rospy
import modsim.params as params
from math import sin, cos
import numpy as np
from math import sqrt
import math
from tf.transformations import euler_from_quaternion
from modsim.util.thrust import convert_thrust_newtons_to_pwm

def position_controller(structure, desired_state, dt):
    """
    PD controller to convert from position to accelerations, and accelerations to attitude.
    Controller: Using these current and desired states, you have to compute the desired controls
    :param struc_mgr: StructureManager object
    :param qd: The object qt contains the current state and the desired state:
                * The current states are: qd.pos, qd.vel, qd.euler = [rollpitchyaw], qd.omega.
                * The desired states are: qd.pos_des, qd.vel_des, qd.acc_des, qd.yaw_des, qd.yawdot_des
    :return: desired thrust and attitude
    """
    #global accumulated_error
    state_vector = structure.state_vector
    num_mod = len(structure.xx)

    # Desired state
    [pos_des, vel_des, acc_des, yaw_des, des_yawdot] = desired_state

    yaw_des = 0

    # current state
    pos = state_vector[:3]
    vel = state_vector[3:6]
    avel= state_vector[-3:] # angular velocity, pqr

    # constants
    m = params.mass
    g = params.grav

    # Get boolean telling us whether real or sim robots
    is_sim           = rospy.get_param("is_modquad_sim", True)
    is_bottom_framed = rospy.get_param("is_modquad_bottom_framed", False)
    is_unframed      = rospy.get_param("is_modquad_unframed", False)
    is_strong_rots   = rospy.get_param("is_strong_rots", True)

    # Horizontal Plane Position Gains
    xyp = 30.0
    xyd = 15.0
    xyi =  0.5

    # z gains, I-Gain handles countering gravity
    # NOTE: these are gains in crazyflie PWM
    zp  =  7000
    zd  =  7770
    zi  =  3500

    yaw_kp =  -5
    yaw_kd = -12
    yaw_ki =   0

    # Default bounds from crazyflie_ros are 30 deg and 200 deg/s
    max_ang      =  2.5
    max_yaw_rate = 30.0

    kp1_u, kd1_u, ki1_u =  xyp,  xyd,  xyi # 10.0, 71.0, 0.0 
    kp2_u, kd2_u, ki2_u =  xyp,  xyd,  xyi # 10.0, 71.0, 0.0 
    kp3_u, kd3_u, ki3_u =   zp,   zd,   zi # 10.0, 48.0, 0.0 

    euler = euler_from_quaternion(state_vector[6:10])
    cur_yaw = euler[2]


    # Error
    pos_error = (np.array(pos_des) - np.array(pos))
    yaw_error = yaw_des - cur_yaw
    structure.prev_error = structure.error
    structure.error = np.array([pos_error[0], pos_error[1], pos_error[2], yaw_error])
    if np.all(structure.prev_error == 0): # Happens at initialization
    	structure.prev_error = structure.error
    vel_error = (structure.error - structure.prev_error) / dt

    # Update the accumulated error in all three dimensions
    max_error = np.array([0.1, 0.1, 1000.0])
    structure.pos_accumulated_error += np.array(pos_error * dt)
    min_indices = structure.pos_accumulated_error < -max_error
    max_indices = structure.pos_accumulated_error >  max_error
    structure.pos_accumulated_error[min_indices] = -max_error[min_indices]
    structure.pos_accumulated_error[max_indices] =  max_error[max_indices]

    # Desired acceleration
    r1_acc = kp1_u * pos_error[0] + \
             kd1_u * vel_error[0] + \
             ki1_u * structure.pos_accumulated_error[0]
                     #acc_des[0]   + \

    r2_acc = kp2_u * pos_error[1] + \
             kd2_u * vel_error[1] + \
             ki2_u * structure.pos_accumulated_error[1]
                      #acc_des[1]   + \

    #rospy.loginfo("perr={}, verr={}, accerr={}".format(
    #    pos_error[2], vel_error[2], structure.pos_accumulated_error[2]
    #))
    r3_acc = kp3_u * pos_error[2] + \
             kd3_u * vel_error[2] + \
             ki3_u * structure.pos_accumulated_error[2]
                     #acc_des[2]   + \

    #import pdb; pdb.set_trace()

    #print("----- R1 -----")
    #print("{} * {} + {} * {} + {} + {} * {} = {}".format(
    #    kp1_u, pos_error[0], kd1_u, vel_error[0], acc_des[0],
    #    ki1_u, structure.pos_accumulated_error[0], r1_acc
    #))
    #print("----- R2 -----")
    #print("{} * {} + {} * {} + {} + {} * {} = {}".format(
    #    kp2_u, pos_error[1], kd2_u, vel_error[1], acc_des[1],
    #    ki2_u, structure.pos_accumulated_error[1], r2_acc
    #))
    #print("----- R3 -----")
    #print("{} * ({}-{}) + {} * {} + {} + {} * {} = {}".format(
    #    kp3_u, pos_des[2], pos[2], kd3_u, vel_error[2], acc_des[2],
    #    ki3_u, structure.pos_accumulated_error[2], r3_acc
    #))
    #print("=======================")

    # ------------------------------ #
    # Effectively, since yaw_des = 0 #
    # phi_des   = -r2_acc / g        #
    # theta_des =  r1_acc / g        #
    #rad_yaw_des = math.radians(yaw_des)
    phi_des   =  -r2_acc # ((r1_acc * sin(yaw_des) - r2_acc * cos(yaw_des)) / g)
    theta_des     =   r1_acc # ((r1_acc * cos(yaw_des) + r2_acc * sin(yaw_des)) / g)

    # Compute new output yaw rate
    psi_des = yaw_kp * yaw_error + yaw_kd * vel_error[3]

    # Orientation max angle limiting
    phi_des   = max(min(phi_des  , max_ang), -max_ang)
    theta_des = max(min(theta_des, max_ang), -max_ang)

    # Max yaw rate needs more limiting
    psi_dot_des   = max(min(psi_des, max_yaw_rate), -max_yaw_rate)

    # Thrust
    #thrust = m * g + r3_acc # Removed "m *" term from r3_acc, mass in gains
    thrust = r3_acc # countering gravity is handled by the I-Gain

    #rospy.loginfo("thrust={}".format(thrust))
    thrust = min(max(thrust, 0000), 60000)

    # desired thrust and attitude
    return [thrust, phi_des, theta_des, psi_dot_des]

def modquad_torque_control(F, M, structure,
                            motor_sat=False, en_fail_rotor=False, 
                            ramp_rotor_set=[], ramp_factor=[],
                            fail_type=1):
    """
    This function is similar to crazyflie_motion, but it is made for modular robots. So it specifies the dynamics
    of the modular structure. It receives a desired force and moment of a single robot.
    :param structure:
    :param motor_sat: motor saturation
    :param F: desired total thrust, float
    :param M: desired moments, 3 x 1 float vector
    :param ramp_rotor_set: [[],[]] list of two sets of rotor tuples 
                            (mod_id, rot-id) where first list shows set of
                            rotors to ramp down and second the set of 
                            rotors to ramp up
    :param ramp_factor: [x, y] where x is ramp_down factor and 
                        y is ramp_up factor. x + y = 1
    :return: thrust and moments for the whole structure
    """

    L = params.arm_length * sqrt(2) / 2.

    # Get position of each rotor in the structure
    rx, ry = _get_rotor_positions(structure)

    # Convert rotor position to signs
    sign_rx = [1 if rx_i > 0 else -1 for rx_i in rx]
    sign_ry = [1 if ry_i > 0 else -1 for ry_i in ry]


    # m = 4 * n  # Number of rotors
    A = [[0.25, sy * .25 / L, -sx * .25 / L] for sx, sy in zip(sign_rx, sign_ry)]

    rotor_forces = np.dot(A, [F, M[0], M[1]])  # Not using moment about Z-axis for limits

    # Motor saturation
    if motor_sat:
        rotor_forces[rotor_forces > params.maxF / 4] = params.maxF / 4.0
        rotor_forces[rotor_forces < params.minF / 4] = params.minF / 4.0

    # Update rotor forces being ramped on/off as part of fault IDing
    rotor_forces = _check_for_rotor_ramping(ramp_rotor_set, ramp_factor,
                                            rotor_forces, structure)

    # If there are faulty rotors, have their impact take place if enabled
    # NOTE: This must be done after the rotor ramping check, because it
    #       will only reduce the thrust further if the rotor_force of the failed
    #       rotor is below a threshold
    rotor_forces = _check_for_failed_rotors(en_fail_rotor, fail_type, 
                                            structure, rotor_forces)

    # From prop forces to total moments. Equation (1) of the modquad paper (ICRA 18)
    F  =  np.sum(rotor_forces)
    Mx =  np.dot(ry, rotor_forces)
    My = -np.dot(rx, rotor_forces)
    # TODO Mz
    Mz = M[2]

    #print(structure.ids)
    #print(F)
    #print(Mx)
    #print(My)
    #print(rotor_forces)
    #print('---')

    return F, [Mx, My, Mz], rotor_forces

def _get_rotor_positions(structure):
    ## From moments to rotor forces (power distribution)
    # positions of the rotors
    #         ^ X
    #    (4)  |      (1) [L, -L]
    #   Y<-----
    #    (3)         (2)

    # Will change later, but the mqscheduler package was written as:
    #    (3)    |    (2) [L, -L]
    #     ------------
    #    (4)    |    (1)
    # So this needs to do a transferance

    # 1 is the first mapping, 2 is the second
    rotor_map_mode = rospy.get_param("rotor_map", 1) 

    rx, ry = [], []
    L = params.arm_length * sqrt(2) / 2.

    for x, y in zip(structure.xx, structure.yy):
        if rotor_map_mode == 1:
            # x-axis
            rx.append(x + L) # R
            rx.append(x - L)
            rx.append(x - L)
            rx.append(x + L)
            # y-axis
            ry.append(y - L)
            ry.append(y - L)
            ry.append(y + L)
            ry.append(y + L)
        else:
            # x-axis
            rx.append(x - L) # R
            rx.append(x + L)
            rx.append(x + L)
            rx.append(x - L)
            # y-axis
            ry.append(y - L)
            ry.append(y - L)
            ry.append(y + L)
            ry.append(y + L)

    return rx, ry

def _check_for_failed_rotors(en_fail_rotor, fail_type, structure, rotor_forces):
    # Failing motors -- IDs are 1-indexed, but rotor pos are 0-indexed
    if not en_fail_rotor:
        return rotor_forces

    min_ramp = rospy.get_param("min_ramp")

    for mf in structure.motor_failure:
        try:
            ind = structure.ids.index('modquad{:02d}'.format(mf[0]))

            #rospy.loginfo("Rot ({:02d}, {:02d}) force orig: {:6.4f}".format(
            #               mf[0], mf[1], rotor_forces[4*ind + mf[1]]))

            if fail_type == 1: # COMPLETE ROTOR FAILURE
                rotor_forces[4 * (ind) + mf[1]] = 0.0

            #elif fail_type == 2: # LOWER MAX THRUST
            #    if rotor_forces[4 * (ind) + mf[1]] >= (params.maxF / 4.0) / 4.0:
            #        rotor_forces[4 * (ind) + mf[1]] = (params.maxF / 4.0) / 4.0
            elif fail_type == 2: # HALVE THRUST RANGE
                if rotor_forces[4 * (ind) + mf[1]] > 0.25 * params.maxF * min_ramp:
                    rotor_forces[4 * (ind) + mf[1]] = 0.25 * params.maxF * min_ramp

            elif fail_type == 3: # 1/4TH THRUST RANGE
                rotor_forces[4 * (ind) + mf[1]] /= 4.0

            else:
                raise Exception("Unknown rotor failure type")
            #rospy.loginfo("                    modified to: {:6.4f}".format(
            #               rotor_forces[4*ind + mf[1]]))
        except:
            print("")
            print("Fail rotor real: {}, {}".format(mf[0], mf[1]))
            print("Fail rotor computed: {}".format(4*(mf[0]-1) + mf[1]))
            print(structure.ids)
            print(structure.motor_failure)
            print(rotor_forces)
            raise Exception("ERROR IN ZEROING FAILED MOTOR THRUST")

    return rotor_forces

def _check_for_rotor_ramping(ramp_rotor_set, ramp_factor, 
                             rotor_forces, structure):
    # If ramping is being done, use ramping factors to update thrusts
    # Ramping is part of fault detection mechanism
    if len(ramp_rotor_set) > 0:
        if len(ramp_rotor_set) != 2 or len(ramp_factor) != 2:
            raise Exception(["Rotor ramping error, list size is wrong\n",
                            "ramp_rotor_set = {}, len = {}".format(
                                ramp_rotor_set, len(ramp_rotor_set)),
                            "ramp_factor = {}, len = {}".format(
                                ramp_factor, len(ramp_factor))
                            ])

        for rr in ramp_rotor_set[0]:
            try:
                ind = structure.ids.index('modquad{:02d}'.format(rr[0]))
                rotor_forces[4 * ind + rr[1]] *= ramp_factor[0]
                #print("Rdown {} to {}".format(rr, ramp_factor[0]))
            except:
                raise Exception("Error ramping rotor down: {} | {}".format(
                                                                    rr, ind))

        for rr in ramp_rotor_set[1]:
            try:
                ind = structure.ids.index('modquad{:02d}'.format(rr[0]))
                rotor_forces[4 * ind + rr[1]] *= ramp_factor[1]
                #print("Rup {} to {}".format(rr, ramp_factor[1]))
            except:
                raise Exception("Error ramping rotor up: {} | {}".format(
                                                                  rr, ind))

    return rotor_forces

# Import here to avoid circ dependency issue
from modsim.datatype.structure import Structure
