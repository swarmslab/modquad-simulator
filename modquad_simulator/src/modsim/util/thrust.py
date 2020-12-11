from math import sqrt

## Control input callback
def convert_thrust_newtons_to_pwm(thrust_newtons):
    # For more info, check:
    # https://github.com/whoenig/crazyflie_ros
    c1, c2, c3 = -0.6709, 0.1932, 13.0652

    """
    # Computations based on reversal of following equations
    F_g = ((thrust_pwm / 60000. - c1) / c2) ** 2 - c3  # Force in grams
    if F_g<0:
        F_g = 0

    thrust_newtons = 9.81 * F_g / 1000.  # Force in Newtons
    """

    # if thrust_newtons < 0:
    #     thrust_newtons = 0.2

    F_g = thrust_newtons * (1000.0 / 9.81)

    if F_g < 0:
        F_g = 0

    thrust = 60000 * (c2 * sqrt(F_g + c3) + c1)


    if thrust < 0000:
        thrust = 0000
    elif thrust > 60000:
        thrust = 60000

    return thrust

def convert_thrust_pwm_to_newtons(thrust_pwm):
    # # For more info, check:
    # # https://github.com/whoenig/crazyflie_ros
    # c1, c2, c3 = -0.6709, 0.1932, 13.0652
    # F_g = ((thrust_pwm / 60000. - c1) / c2) ** 2 - c3  # Force in grams
    # if F_g < 0:
    #     F_g = 0

    # return 9.81 * F_g / 1000.  # Force in Newtons

    # Re-did the best-fit line based on data from 
    # https://wiki.bitcraze.io/misc:investigations:thrust
    # ORIGINAL
    c1 =  6.3590474 * pow(10, -10)
    c2 =  5.5956151 * pow(10, -5)
    c3 = -0.0100687

    # MODIFIED FOR WAY MORE THRUST
    #c1 =  1.2718095 * pow(10, -9)
    #c2 =  0.0001119
    #c3 = -0.0201375

    F_n = c1 * (thrust_pwm ** 2) + c2 * thrust_pwm + c3

    if F_n < 0:
        F_n = 0

    return F_n
