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

$roscore  
$roslaunch open_manipulator_with_tb3_gazebo open_manipulator_with_tb3_rooms1.launch  
$roslaunch open_manipulator_with_tb3_description open_manipulator_with_tb3_model.launch use_gazebo:=true  
!!!!! please push play button gazebo map .  
$roslaunch open_manipulator_with_tb3_tools open_manipulator_with_tb3_controller1.launch

Click image to link to YouTube video.  
[![Video Label](http://img.youtube.com/vi/xfhoDjVyetg/0.jpg)](https://youtu.be/xfhoDjVyetg?t=0s) 

## Run Room2

$roscore  
$roslaunch open_manipulator_with_tb3_gazebo open_manipulator_with_tb3_rooms2.launch  
$roslaunch open_manipulator_with_tb3_description open_manipulator_with_tb3_model.launch use_gazebo:=true  
!!!!! please push play button gazebo map .  
$roslaunch open_manipulator_with_tb3_tools open_manipulator_with_tb3_controller2.launch

Click image to link to YouTube video.  
[![Video Label](http://img.youtube.com/vi/XlLM5o116SQ/0.jpg)](https://youtu.be/XlLM5o116SQ?t=0s) 

## Run Room10 

This example is a room example to take picture for a Korean Thanksgiving card.   

$roscore  
$roslaunch open_manipulator_with_tb3_gazebo open_manipulator_with_tb3_rooms10.launch  
!!!!! please push play button at gazebo .  
$roslaunch open_manipulator_with_tb3_waffle_moveit demo2.launch use_gazebo:=true  
-> control manipulator position   
-> next please push stop button at gazebo .  
-> move turtlebot3 place where you want   

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
