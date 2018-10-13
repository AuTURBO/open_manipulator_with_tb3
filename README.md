## Description
This git is addition application git for Open Manipulator with turtlebot3.  
Original Reference Code is   
https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3  

## environment setting 

*First you must set open manipulator and turtlebot3 environment and mobile manipulator   
http://emanual.robotis.com/docs/en/platform/openmanipulator/  
http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/  

*Sencond,  download modifed git. 
```bash
git clone https://github.com/AuTURBO/open_manipulator_with_tb3.git  
git clone https://github.com/AuTURBO/open_manipulator_with_tb3_simulations.git  
```
*Third, set model waffle at ~/.bashrc file.    
```bash
#export TURTLEBOT3_MODEL=burger
export TURTLEBOT3_MODEL=waffle
#export TURTLEBOT3_MODEL=waffle_pi
```
## Run Room1

```bash
$roscore  
$roslaunch open_manipulator_with_tb3_gazebo open_manipulator_with_tb3_rooms1.launch  
$roslaunch open_manipulator_with_tb3_description open_manipulator_with_tb3_model.launch use_gazebo:=true  
!!!!! please push play button gazebo  window.  
$roslaunch open_manipulator_with_tb3_tools open_manipulator_with_tb3_controller1.launch
```
Click image to link to YouTube video.  
[![Video Label](http://img.youtube.com/vi/xfhoDjVyetg/0.jpg)](https://youtu.be/xfhoDjVyetg?t=0s) 

## Run Room2

```bash
$roscore  
$roslaunch open_manipulator_with_tb3_gazebo open_manipulator_with_tb3_rooms2.launch  
$roslaunch open_manipulator_with_tb3_description open_manipulator_with_tb3_model.launch use_gazebo:=true  
!!!!! please push play button gazebo window.  
$roslaunch open_manipulator_with_tb3_tools open_manipulator_with_tb3_controller2.launch
```
Click image to link to YouTube video.  
[![Video Label](http://img.youtube.com/vi/XlLM5o116SQ/0.jpg)](https://youtu.be/XlLM5o116SQ?t=0s) 

## Run Room3

Room3 is a demo simulation that is objecti picking by 3D and 2D image processing.   
The used algorithm is Darknet ROS ( YOLO3, Cuda ) , jsk_recognition , MoveIt , MoveBase and Gmapping.   

refernce site is   
https://github.com/ROBOTIS-GIT/open_manipulator_with_tb3  
https://github.com/ROBOTIS-JAPAN-GIT/turtlebot3_slam_3d  

* addition install packages   
```bash
git clone https://github.com/hyunoklee/darknet_ros.git
git clone https://github.com/AuTURBO/turtlebot3_slam_3d
git clone https://github.com/jsk-ros-pkg/jsk_common.git
git clone https://github.com/jsk-ros-pkg/jsk_recognition.git
catkin_ws && catkin_make
sudo apt-get install ros-kinetic-jsk-recognition
sudo apt-get install ros-kinetic-libsiftfast
sudo apt-get install ros-kinetic-laser-assembler
sudo apt-get install ros-kinetic-octomap-server
sudo apt-get install ros-kinetic-nodelet
sudo apt-get install ros-kinetic-depth-image-proc
sudo apt-get install ros-kinetic-jsk-topic-tools
sudo apt-get install ros-kinetic-rtabmap-ros
```
* run  
```bash
$roscore  
$roslaunch open_manipulator_with_tb3_gazebo open_manipulator_with_tb3_rooms3.launch  
$roslaunch open_manipulator_with_tb3_description open_manipulator_with_tb3_model.launch use_gazebo:=true  
!!!!! please push play button gazebo window.  
$roslaunch open_manipulator_with_tb3_tools open_manipulator_with_tb3_controller3.launch  
```
Click image to link to YouTube video.  
[![Video Label](http://img.youtube.com/vi/G6I1NdUfEP0/0.jpg)](https://youtu.be/G6I1NdUfEP0?t=0s) 

## Run Room10 

This example is a room example to take picture for a Korean Thanksgiving card.

```bash
$roscore  
$roslaunch open_manipulator_with_tb3_gazebo open_manipulator_with_tb3_rooms10.launch  
!!!!! please push play button at gazebo .  
$roslaunch open_manipulator_with_tb3_waffle_moveit demo2.launch use_gazebo:=true  
-> control manipulator position   
-> next please push stop button at gazebo window. 
-> move turtlebot3 place where you want   
```

<img src="/picture/room10_1.png" width="70%" height="70%">  
<img src="/picture/room10_2.jpg" width="70%" height="70%">  
 

## You can set the open manipulator position by kinematics and joint. 
## You can get the open manipulator position by kinematics and joint. 

I add node to contorl open_manipulator_position_ctrl (arm_controller.cpp) by server and clinet. 
```bash
$roscore   
$roslaunch open_manipulator_with_tb3_gazebo open_manipulator_with_tb3_gazebo2.launch   
$roslaunch open_manipulator_with_tb3_waffle_moveit demo2.launch use_gazebo:=true   
$roslaunch open_manipulator_with_tb3_tools open_manipulator_with_tb3_getset.launch   
$rostopic pub /getarm_position std_msgs/String "get_kinematics_pose" --once   
$rostopic pub /getarm_position std_msgs/String "set_kinematics_pose" --once   
$rostopic pub /getarm_position std_msgs/String "get_joint_pose" --once   
$rostopic pub /getarm_position std_msgs/String "set_joint_pose" --once   
```
