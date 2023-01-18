# vention-homework

## Getting started
Install ROS Noetic with MoveIt
> https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html

Downloading Panda configuration for MoveIt in the catkin_ws.
> git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel

Clonning this repository inside the catkin_ws.
> git clone https://github.com/madumont/vention-homework.git

Compiling the catkin_ws
> catkin_make

## How to launch it

> roslaunch panda_moveit_config demo.launch
> roslaunch vention_homework cube_grabber.launch

In RVIZ you should see the panda arm moving.
In the second terminal you should see the position of the cube printing every 0.5 seconde.


The output of the position of the cube and the Up vector of the cube are not working. With the time I wanted to put in this homework, it's all I could do.
