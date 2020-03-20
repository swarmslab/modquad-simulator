# ModQuad and Crazyflie Simulator

### Requirements
* Install crazyflie_ros: https://github.com/whoenig/crazyflie_ros
* install python packages 
``` sudo pip install --upgrade numpy scipy transforms3d```



### Launching the simulator
The simulator for five quadrotors is launched by
```
$ roslaunch modquad_simulator simulation.launch
```
Each robot simulates the dynamics of a quadrotor by runing the `modquad_sim` node and its pose is displayed in RViz.
The launch file `simulation.launch` can be easily modified to change the number of quadrotors, their initial location and their color.

As well as the actual crazyflie robot, the simulator receives two inputs:
* Attitude command: the topic _cmd_vel_ receives the desired thrust, roll, pitch and yaw inputs. It follows the same format as the `crazyflie_ros `package.
* Goal: using the `crazyflie_controller`, the simulator also receives goals trhough the _goal_ topic and takeoff commands through the _takeoff_ service. 

### Demo
Once the simulator is running, we can send desired goals to the robots.  The following command runs a demo script that 
takes off the robots and makes them move in circle.
```
rosrun demo-simulator demo_circle_multiple.py
```


# Architecture

The main script that simulates a single aerial vehicle is ```modquad_simulator/scripts/modquad_sim.py```. 
There are some parameters that can be set such as: robot name (_~robot_id_), initial location (_init_x, init_y, init_z_),
and a predefined trajectory (_~demo_trajectory_). The predefined trajectory generates attitude control inputs to move the
aerial vehicle in circles.

### Topics
The simulator subscribes to an attitude input topic: _/cmd_vel_

The simulator publishes the odometry of the aerial vehicle in _/odom_
It also publishes the frame of the robot using _tf2_ros.TransformBroadcaster()_

### Internal data types
* The **state_vector** has dimensions 13 x 1, and contains the position (_x,y,z_), linear velocities (_dx,dy,dz_), 
attitude represented by a quaternion (_qw, qx, qy, qz_), and angular velocities (_p, q, r_). Summarized is _x = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]_.
* datatype.QuadState: state of the aerial vehicle, and the goal.
* datatype.Structure: x and y coordinates of the n modules rigidly attached.

### Main loop
The main loop of the simulator performs the following five steps:
1. Publish the current odometry, using the function _publish_structure_odometry()_ in the same file.
2. Read the control input variables (_thrust_newtons, roll, pitch, yaw_) either from the callback function or the _demo_trajectory_.
3. Compute the required force and moments for a single quadrotor (_F_single, M_single_). See _attitude.py_ file.
4. Compute the required force and moments for a structure (_F_structure, M_structure_) based on the single quadrotor output. See _modquad_torque_control_ file.
5. Simulate by integrating the dynamics. See _ode_integrator.py_ file. 




### Credits
This is an open source project, most of the code is licensed under GPL v3.0.

Part of the dynamics simulation is an adaptation of the Quadrotor Simulator in Matlab by Daniel Mellinger at University of Pennsylvania.


The simulator is currently developed and maintained by Lehigh University and University of Pennsylvania. 
 

