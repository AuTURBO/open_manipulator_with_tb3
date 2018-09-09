# OpenManipulator
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/openmanipulator/OpenManipulator.png">
<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/openmanipulator/OpenManipulator_Chain_Capture.png" width="500">

## ROS Packages for OpenManipulator
|Version|Kinetic + Ubuntu Xenial|Melodic + Ubuntu Bionic|
|:---:|:---:|:---:|
|[![GitHub version](https://badge.fury.io/gh/ROBOTIS-GIT%2Fopen_manipulator_with_tb3.svg)](https://badge.fury.io/gh/ROBOTIS-GIT%2Fopen_manipulator_with_tb3)|[![Build Status](https://travis-ci.org/ROBOTIS-GIT/open_manipulator_with_tb3.svg?branch=kinetic-devel)](https://travis-ci.org/ROBOTIS-GIT/open_manipulator_with_tb3)|-|

## ROBOTIS e-Manual for OpenManipulator
- [ROBOTIS e-Manual for OpenManipulator with tb3](http://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#manipulation)

## Wiki for open_manipulator_with_tb3 Packages
- http://wiki.ros.org/open_manipulator_with_tb3 (metapackage)
- http://wiki.ros.org/open_manipulator_with_tb3_description
- http://wiki.ros.org/open_manipulator_with_tb3_tools
- http://wiki.ros.org/open_manipulator_with_tb3_waffle_moveit
- http://wiki.ros.org/open_manipulator_with_tb3_waffle_pi_moveit

## Open Source related to OpenManipulator
- [open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator)
- [open_manipulator_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_msgs)
- [open_manipulator_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_simulations)
- [open_manipulator_perceptions](https://github.com/ROBOTIS-GIT/open_manipulator_perceptions)
- [open_manipulator_with_tb3](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3)
- [open_manipulator_with_tb3_msgs](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_msgs)
- [open_manipulator_with_tb3_simulations](https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3_simulations)
- [turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [turtlebot3_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_msgs)
- [turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- [turtlebot3_applications](https://github.com/ROBOTIS-GIT/turtlebot3_applications)
- [turtlebot3_applications_msgs](https://github.com/ROBOTIS-GIT/turtlebot3_applications_msgs)
- [turtlebot3_autorace](https://github.com/ROBOTIS-GIT/turtlebot3_autorace)
- [turtlebot3_deliver](https://github.com/ROBOTIS-GIT/turtlebot3_deliver)
- [hls_lfcd_lds_driver](https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver)
- [manipulator_h](https://github.com/ROBOTIS-GIT/ROBOTIS-MANIPULATOR-H)
- [dynamixel_sdk](https://github.com/ROBOTIS-GIT/DynamixelSDK)
- [dynamixel_workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench)
- [robotis_math](https://github.com/ROBOTIS-GIT/ROBOTIS-Math)
- [OpenCR-Hardware](https://github.com/ROBOTIS-GIT/OpenCR-Hardware)
- [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR)

## Documents and Videos related to OpenManipulator
- [ROBOTIS e-Manual for OpenManipulator](http://emanual.robotis.com/docs/en/platform/openmanipulator/)
- [ROBOTIS e-Manual for TurtleBot3](http://turtlebot3.robotis.com/)
- [ROBOTIS e-Manual for ROBOTIS MANIPULATOR-H](http://emanual.robotis.com/docs/en/platform/manipulator_h/introduction/)
- [ROBOTIS e-Manual for Dynamixel SDK](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)
- [ROBOTIS e-Manual for Dynamixel Workbench](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/)
- [e-Book for TurtleBot3 and OpenManipulator](https://community.robotsource.org/t/download-the-ros-robot-programming-book-for-free/51/)
- [Videos for OpenManipulator](https://www.youtube.com/playlist?list=PLRG6WP3c31_WpEsB6_Rdt3KhiopXQlUkb)
- [Videos for TurtleBot3 and OpenManipulator](https://www.youtube.com/playlist?list=PLRG6WP3c31_XI3wlvHlx2Mp8BYqgqDURU)


## add server clinet node to contorl open_manipulator_position_ctrl / arm_controller.cpp

run command  

roscore   
roslaunch open_manipulator_with_tb3_gazebo open_manipulator_with_tb3_gazebo2.launch   
roslaunch open_manipulator_with_tb3_waffle_moveit demo2.launch use_gazebo:=true   
roslaunch open_manipulator_with_tb3_tools open_manipulator_with_tb3_getset.launch   
rostopic pub /getarm_position std_msgs/String "get_kinematics_pose" --once   
rostopic pub /getarm_position std_msgs/String "set_kinematics_pose" --once   
rostopic pub /getarm_position std_msgs/String "get_joint_pose" --once   
rostopic pub /getarm_position std_msgs/String "set_joint_pose" --once   
