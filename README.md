# ModQuad and Crazyflie Simulator

### Versions
This fork of https://github.com/swarmslab/modquad-simulator works with ROS
Noetic on Lubuntu 20. Not the master branch, but the modulardynamics
branch is where it works at the moment.

### Requirements
* Install crazyflie_ros: https://github.com/whoenig/crazyflie_ros
* install python packages `$ sudo pip install --upgrade numpy scipy transforms3d`

### Launching the simulator
The simulator for ```num_robots``` robots is launched by calling:
```
$ roslaunch modquad_simulator programmatic_launch.launch nr:=${num_robots}
```
This launch file creates a module `modquad_sim` for each robot. This module
simulates the dynamics of a quadrotor and displays its pose in RViz.

This launch will auto-generated the requisite RViz settings for the number of
robots specified.

Note that the initial location of quadrotors is not modifiable as it is when you
call
```
$ roslaunch modquad_simulator simulation.launch
```
and manually enter the initial locations and colors.

The launch file `simulation.launch` can be easily modified to change the number
of quadrotors, their initial location and their color. Note that every new robot
needs to be added in RViz too.

As well as the actual crazyflie robot, the simulated quadrotor receives two inputs:
* Attitude command: the topic _cmd_vel_ receives the desired thrust, roll, pitch
  and yaw inputs. It follows the same format as the `crazyflie_ros `package.
* Goal: using the `crazyflie_controller` package, the simulator also receives
  goals through the _goal_ topic. This package also includes the services for
  taking off and landing (see the documentation of the package). 

### Out-of-the-Box Demos
Once the simulator is running, we can send desired goals to the robots.  The
following script runs a demo that takes off the robots and makes them move in a
circle.

One pre-configured demo is a 3x3 square structure flying in a helix:
```
rosrun demo_simulator helix_3x3structure.py
```

#### Demo for Reconfiguration
After many more changes, here is the new set of commands to run:
NOTE: This requires the mqscheduler package to run correctly
```zsh
roslaunch modquad_simulator simulation.launch
rosrun modquad_simulator dock_detector.py
rosrun split_structure_server.py
rosrun modquad_simulator modquad_sim.py
```
Note: These values must be manually edited currently.
* assembly_manager.py: self.n = number of robots in simulation
* dock_detector.py: n = number of robots in simulation

### Credits
This is an open source project, most of the code is licensed under GPL v3.0.

This simulator and is developed and maintained by [David Saldaña](http://davidsaldana.co/) at University of Pennsylvania.

Part of the dynamics simulation is an adaptation of the Quadrotor Simulator in Matlab by Daniel Mellinger at University of Pennsylvania.
