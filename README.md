# ram_ba_package
==============

This repository contains the package "ram" I am writing for my Bachelor Assignment at the Robotics and Mechatronics group at the University of Twente.
It heavily depends on the ardrone_autonomy package. Make sure this package is installed and configured. There are some launch files provided.
Everything will be written in C++.

Current executables:
- fly_from_keyboard

## fly_from_keyboard
This executable lets you control a connected quadcopter with your keyboard. To run this, perform the following steps. Assumed is that the the latest version of the source is in a source folder of your Catkin workspace.

1. Run catkin_make from your Catkin workspace root.
2. Run "roscore".
3. Open a new terminal and run your ardrone_driver. Recommended is to do this by running the launch file supplied using "roslaunch". It may be necessary to run "source devel/setup.bash" for this operation, if you built ardrone_driver from source.
4. Open yet another terminal and source the freshly built packages using "source devel/setup.bash".
5. Run this package by "rosrun ram fly_from_keyboard".

The following shortcuts can be used now:

| Action | Key |
| ----- | ----- |
| Go left | a |
| Go right | d |
| Go forward | w |
| Go backward | s |
| Go up | k |
| Go down | m |
| Take off | o |
| Land | l |
| Reset | r |

