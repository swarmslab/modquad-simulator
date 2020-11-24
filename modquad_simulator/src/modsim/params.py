import numpy as np
from math import pi

# Crazyflie: physical parameters for the Crazyflie 2.0
# Model assumptions based on physical measurements:
# 
# motor + mount + vicon marker = mass point of 3g
# arm length of mass point: 0.046m from center
# battery pack + main board are combined into cuboid (mass 18g) of
# dimensions:
# 
#    width  = 0.03m
#    depth  = 0.03m
#    height = 0.012m
chassis_width = 0.03#m 

#m = 0.034   # mass (in kg) without cage (each is about 0.032kg)
# m = 0.036  # mass (in kg) with bottom of cage

#m = 0.034  # mass (in kg) no magnet 1x2 structure - mass for SIMULATION ALSO
m = 0.0345  # mass (in kg) no magnet 1x4 structure, 4 carbon fiber rods
#m = 0.0335  # mass (in kg) no magnet 4x1 structure

#m = 0.04  # mass (in kg) with cage (each is about 0.04kg)
#m = 0.042  # mass (in kg) with cage and larger battery
g = 9.81  # gravitational constant
# inertia tensor in m^2 kg
# I = [[1.43e-5, 0, 0],
#      [0, 1.43e-5, 0],
#      [0, 0, 2.89e-5]]

# Inertia from document http://groups.csail.mit.edu/robotics-center/public_papers/Landry15.pdf
# page 39. inertia tensor in m^2 kg
I = [[2.3951e-5, 0, 0],
     [0, 2.3951e-5, 0],
     [0, 0, 3.2347e-5]]


L = 0.05  # arm length in m

mass = m

invI = np.linalg.inv(I)
grav = g
arm_length = L
cage_width = 0.115


maxangle = 40 * pi / 180  # you can specify the maximum commanded angle here

# Based on https://wiki.bitcraze.io/misc:investigations:thrust
# Max thrust for entire quadrotor, NOT for an individual rotor
maxF = 57.9 / g # Max thrust (60k PWM, 93.5% of actual max) is 57.9 g, convert to Newtons
minF = 0.0

# FIXME the maximum force should be 4*60g
#maxF = 2.5 * m * g  # left these untouched from the nano plus
#minF = 0.0 * m * g  # left these untouched from the nano plus


max_thrust = 0.597 # from bitcraze

class RunType:
    SIM = 0
    VICON = 1
    FLOWDECK = 2
