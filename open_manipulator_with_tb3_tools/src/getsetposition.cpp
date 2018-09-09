/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <open_manipulator_msgs/SetJointPosition.h>
#include <open_manipulator_msgs/GetJointPosition.h>
#include <open_manipulator_msgs/SetKinematicsPose.h>
#include <open_manipulator_msgs/GetKinematicsPose.h>
#include <open_manipulator_msgs/State.h>
#include <open_manipulator_with_tb3_msgs/Pick.h>

#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131


ros::ServiceClient joint_position_command_client;
ros::ServiceClient joint_getposition_command_client;
ros::ServiceClient kinematics_pose_command_client;
ros::ServiceClient kinematics_getpose_command_client;

ros::ServiceClient gripper_position_command_client;

ros::Publisher grip_pub;
std::string robot_name;

const uint8_t IS_MOVING = 0;
const uint8_t STOPPED   = 1;

typedef struct
{
  bool arm;
  bool gripper;
} State;

State state = {false, false};

bool initJointPosition()
{

  open_manipulator_msgs::SetJointPosition msg;
  ROS_INFO("SET INIT POSITION");

  msg.request.joint_position.joint_name.push_back("joint1");
  msg.request.joint_position.joint_name.push_back("joint2");
  msg.request.joint_position.joint_name.push_back("joint3");
  msg.request.joint_position.joint_name.push_back("joint4");

  msg.request.joint_position.position.push_back( 0.0);
  msg.request.joint_position.position.push_back(-0.65);
  msg.request.joint_position.position.push_back( 1.20);
  msg.request.joint_position.position.push_back(-0.54);

  msg.request.joint_position.max_velocity_scaling_factor = 0.3;
  msg.request.joint_position.max_accelerations_scaling_factor = 0.5;

  ros::service::waitForService(robot_name + "/set_joint_position");
  if (joint_position_command_client.call(msg))
  {
    ROS_INFO("PLANNING IS SUCCESSED");
    ros::WallDuration sleep_time(1.0);
    sleep_time.sleep();
    return msg.response.isPlanned;
  }
  else
  {
    ROS_ERROR("FAILED TO CALL SERVER");
    return false;
  }
}

void armStateMsgCallback(const open_manipulator_msgs::State::ConstPtr &msg)
{
  std::string get_arm_state = msg->robot;
  
  if (get_arm_state == msg->STOPPED){
    state.arm = STOPPED;
    //ROS_INFO("Recieve ARM STATE STOPPED");
  }
  else if (get_arm_state == msg->IS_MOVING){
    state.arm = IS_MOVING;
    //ROS_INFO("Recieve ARM STATE is moving ");
  }
}

void gripperStateMsgCallback(const open_manipulator_msgs::State::ConstPtr &msg)
{
  std::string get_gripper_state = msg->robot;

  if (get_gripper_state == msg->STOPPED)
    state.gripper = STOPPED;
  else if (get_gripper_state == msg->IS_MOVING)
    state.gripper = IS_MOVING;
}

void get_kinematics_pose(void)
{        

	if (state.arm == IS_MOVING)
        {
		ROS_WARN("arm is mobing now");
		return ;
	}
        open_manipulator_msgs::GetKinematicsPose eef_pose;
        geometry_msgs::PoseStamped desired_pose;
        ROS_INFO("get_kinematics_pose");

        desired_pose.pose.position.x = 0.1;
        desired_pose.pose.position.y = 0;
        desired_pose.pose.position.z = 0.3;

        desired_pose.pose.orientation.x = 0;
        desired_pose.pose.orientation.y = 0;
        desired_pose.pose.orientation.z = 0;
        desired_pose.pose.orientation.w = 1;


        ros::service::waitForService(robot_name + "/get_kinematics_pose");
        if (kinematics_getpose_command_client.call(eef_pose))
        {

          ROS_INFO("PLANNING0");

          ROS_INFO("x = %.3f", eef_pose.response.kinematics_pose.pose.position.x);
          ROS_INFO("y = %.3f", eef_pose.response.kinematics_pose.pose.position.y);
          ROS_INFO("z = %.3f", eef_pose.response.kinematics_pose.pose.position.z);

          ROS_INFO("qx = %.3f", eef_pose.response.kinematics_pose.pose.orientation.x);
          ROS_INFO("qy = %.3f", eef_pose.response.kinematics_pose.pose.orientation.y);
          ROS_INFO("qz = %.3f", eef_pose.response.kinematics_pose.pose.orientation.z);
          ROS_INFO("qw = %.3f", eef_pose.response.kinematics_pose.pose.orientation.w);

          //ROS_INFO("yaw = %.3f", yaw * RAD2DEG);

 	  ros::WallDuration sleep_time(1.0);
          sleep_time.sleep();
	}

}

void set_kinematics_pose(void)
{

	if (state.arm == IS_MOVING)
        {
		ROS_WARN("arm is mobing now");
		return ;
	}
        open_manipulator_msgs::SetKinematicsPose eef_pose;
        geometry_msgs::PoseStamped desired_pose;	

	ROS_INFO("set_kinematics_pose");

        desired_pose.pose.position.x = 0 ;
        desired_pose.pose.position.y = 0 ;
        desired_pose.pose.position.z = 0.3 ;

        desired_pose.pose.orientation.x = 0 ;
        desired_pose.pose.orientation.y = 0 ;
        desired_pose.pose.orientation.z = 0 ;
        desired_pose.pose.orientation.w = 1 ;

        //ROS_INFO("yaw = %.3f", yaw * RAD2DEG);

        eef_pose.request.kinematics_pose.group_name = "arm";
        eef_pose.request.kinematics_pose.pose = desired_pose.pose;
        eef_pose.request.kinematics_pose.max_velocity_scaling_factor = 0.1;
        eef_pose.request.kinematics_pose.max_accelerations_scaling_factor = 0.5;
        eef_pose.request.kinematics_pose.tolerance = 0.01;

        ros::service::waitForService(robot_name + "/set_kinematics_pose");
        if (kinematics_pose_command_client.call(eef_pose))
        {
          if (eef_pose.response.isPlanned == true)
          {
            ROS_INFO("PLANNING IS SUCCESSED");
            ros::WallDuration sleep_time(1.0);
            sleep_time.sleep();

          }
          else
          {
	    ROS_INFO("PLANNING IS Fail");
            
          }
        }else{
	    ROS_INFO("SERVER RESPONSE Fail");
	}
}

void get_joint_pose(void)
{

	if (state.arm == IS_MOVING)
        {
		ROS_WARN("arm is mobing now");
		return ;
	}
	ROS_INFO("get_joint_pose");
	open_manipulator_msgs::GetJointPosition msg;

	std::string joint_name ;
	float joint_position ;

	ros::service::waitForService(robot_name + "/get_joint_position") ;
	if (joint_getposition_command_client.call(msg))
	{
	  for (std::size_t i = 0; i < msg.response.joint_position.position.size(); ++i)
	  {
            joint_name = msg.response.joint_position.joint_name[i] ;
	    joint_position = msg.response.joint_position.position[i] ;
	    ROS_INFO("%s: %f", joint_name.c_str(), joint_position);
	  }
	  ros::WallDuration sleep_time(1.0);
	  sleep_time.sleep();
	}
	else
	{
	  ROS_ERROR("FAILED TO CALL SERVER");
	}

}

void set_joint_pose(void)
{

	if (state.arm == IS_MOVING)
        {
		ROS_WARN("arm is mobing now");
		return ;
	}

	ROS_INFO("set_joint_pose");
	open_manipulator_msgs::SetJointPosition msg;

	msg.request.joint_position.joint_name.push_back("joint1");
	msg.request.joint_position.joint_name.push_back("joint2");
	msg.request.joint_position.joint_name.push_back("joint3");
	msg.request.joint_position.joint_name.push_back("joint4");

	msg.request.joint_position.position.push_back( 0.0);
	msg.request.joint_position.position.push_back(-0.65);
	msg.request.joint_position.position.push_back( 1.20);
	msg.request.joint_position.position.push_back(-0.54);

	msg.request.joint_position.max_velocity_scaling_factor = 0.3;
	msg.request.joint_position.max_accelerations_scaling_factor = 0.5;

	ros::service::waitForService(robot_name + "/set_joint_position");
	if (joint_position_command_client.call(msg))
	{
	  ROS_INFO("PLANNING IS SUCCESSED");
	  ros::WallDuration sleep_time(1.0);
	  sleep_time.sleep();
	}
	else
	{
	  ROS_ERROR("FAILED TO CALL SERVER");

	}

}

void getArmStateMsgCallback(const std_msgs::String::ConstPtr& msg)
{ 
	
 	if( msg->data == "get_kinematics_pose" ){
                get_kinematics_pose();
	}else if( msg->data == "set_kinematics_pose" ){
		set_kinematics_pose();
	}else if( msg->data == "get_joint_pose" ){
		get_joint_pose();
	}else if( msg->data == "set_joint_pose" ){
		set_joint_pose();
	}else{
		ROS_INFO("not support command");
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getsetposition");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  priv_nh.getParam("robot_name", robot_name);

  joint_position_command_client = nh.serviceClient<open_manipulator_msgs::SetJointPosition>(robot_name + "/set_joint_position");
  joint_getposition_command_client = nh.serviceClient<open_manipulator_msgs::GetJointPosition>(robot_name + "/get_joint_position");
  kinematics_pose_command_client = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>(robot_name + "/set_kinematics_pose");
  kinematics_getpose_command_client = nh.serviceClient<open_manipulator_msgs::GetKinematicsPose>(robot_name + "/get_kinematics_pose");
  gripper_position_command_client = nh.serviceClient<open_manipulator_msgs::SetJointPosition>(robot_name + "/set_gripper_position");

  ros::Subscriber arm_state_sub = nh.subscribe(robot_name + "/arm_state", 10, armStateMsgCallback);
  ros::Subscriber gripper_state_sub = nh.subscribe(robot_name + "/gripper_state", 10, gripperStateMsgCallback);
  
  ros::Subscriber getarm_position_sub = nh.subscribe("/getarm_position", 10, getArmStateMsgCallback);
  ROS_INFO("Ready to GETSET Task");  
  
  initJointPosition();    

  ros::spin();
  return 0;
}
