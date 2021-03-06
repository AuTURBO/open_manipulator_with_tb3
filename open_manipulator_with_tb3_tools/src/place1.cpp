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

/* Authors: Darby Lim */

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <open_manipulator_msgs/SetJointPosition.h>
#include <open_manipulator_msgs/SetKinematicsPose.h>
#include <open_manipulator_msgs/State.h>
#include <open_manipulator_with_tb3_msgs/Place.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <geometry_msgs/Twist.h>

#define DEG2RAD 0.01745329251
#define RAD2DEG 57.2957795131

#define ON  true
#define OFF false

const uint8_t IS_MOVING = 0;
const uint8_t STOPPED   = 1;

#define DIST_GRIPPER_TO_JOINT4       0.145
#define DIST_EDGE_TO_CENTER_OF_PALM  0.025

enum
{
  WAITING_FOR_SIGNAL = 1,
  CHECK_AR_MARKER_POSE,
  MOVE_ARM,
  CLOSE_TO_BOX,
  PLACE_OBJECT,
  INIT_POSITION,
  WAITING_FOR_STOP
};

ros::ServiceClient joint_position_command_client;
ros::ServiceClient kinematics_pose_command_client;
ros::ServiceClient gripper_position_command_client;

ros::ServiceClient place_result_client;

ros::Publisher grip_pub;
ros::Publisher cmd_vel_pub ;

typedef struct
{
  bool arm;
  bool gripper;
  bool marker;
} State;

ar_track_alvar_msgs::AlvarMarker ar_marker_pose;
geometry_msgs::PoseStamped desired_pose;

State state = {false, false, false};

uint8_t task = 0, pre_task = 0;

double roll = 0.0, pitch = 0.0, yaw = 0.0;

double tolerance = 0.01;

ar_track_alvar_msgs::AlvarMarker markers;

std::string robot_name;
double offset_for_object_height, dist_ar_marker_to_box;

int get_marker_id;

bool initJointPosition()
{
  open_manipulator_msgs::SetJointPosition msg;

  msg.request.joint_position.joint_name.push_back("joint1");
  msg.request.joint_position.joint_name.push_back("joint2");
  msg.request.joint_position.joint_name.push_back("joint3");
  msg.request.joint_position.joint_name.push_back("joint4");

  msg.request.joint_position.position.push_back( 0.0);
  msg.request.joint_position.position.push_back(-1.5707);
  msg.request.joint_position.position.push_back( 1.37);
  msg.request.joint_position.position.push_back(0.2258);

  msg.request.joint_position.max_velocity_scaling_factor = 0.3;
  msg.request.joint_position.max_accelerations_scaling_factor = 0.5;

  ros::service::waitForService(robot_name + "/set_joint_position");
  if (joint_position_command_client.call(msg))
  {
    return msg.response.isPlanned;
  }
  else
  {
    ROS_ERROR("FAILED TO CALL SERVER");
    return false;
  }
}

bool gripper(bool onoff)
{
  open_manipulator_msgs::SetJointPosition msg;

  msg.request.joint_position.joint_name.push_back("grip_joint");
  msg.request.joint_position.joint_name.push_back("grip_joint_sub");

  if (onoff == true)
    msg.request.joint_position.position.push_back(0.01);
  else
    msg.request.joint_position.position.push_back(-0.01);

  msg.request.joint_position.max_velocity_scaling_factor = 0.3;
  msg.request.joint_position.max_accelerations_scaling_factor = 0.01;

  ros::service::waitForService(robot_name + "/set_gripper_position");
  if (gripper_position_command_client.call(msg))
  {
    return msg.response.isPlanned;
  }
  else
  {
    ROS_ERROR("FAILED TO CALL SERVER");
    return false;
  }
}

bool resultOfPlace(std_msgs::String res_msg)
{
  open_manipulator_with_tb3_msgs::Place msg;

  msg.request.state = res_msg.data;

  ros::service::waitForService(robot_name + "/result_of_place");
  if (place_result_client.call(msg))
  {
    return true;
  }
  else
  {
    ROS_ERROR("FAILED TO CALL SERVER");
    return false;
  }
}

geometry_msgs::PoseStamped calcDesiredPose(ar_track_alvar_msgs::AlvarMarker marker)
{
  geometry_msgs::PoseStamped get_pose;

  if (marker.id == get_marker_id)
  {
    get_pose = marker.pose;

    get_pose.pose.position.x = marker.pose.pose.position.x - DIST_GRIPPER_TO_JOINT4;
    get_pose.pose.position.y = 0.0;
    get_pose.pose.position.z = marker.pose.pose.position.z + offset_for_object_height;

    double dist = sqrt((marker.pose.pose.position.x * marker.pose.pose.position.x) +
                       (marker.pose.pose.position.y * marker.pose.pose.position.y));

    if (marker.pose.pose.position.y > 0) yaw =        acos(marker.pose.pose.position.x/dist);
    else                                 yaw = (-1) * acos(marker.pose.pose.position.x/dist);

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    get_pose.pose.orientation.w = cy * cr * cp + sy * sr * sp;
    get_pose.pose.orientation.x = cy * sr * cp - sy * cr * sp;
    get_pose.pose.orientation.y = cy * cr * sp + sy * sr * cp;
    get_pose.pose.orientation.z = sy * cr * cp - cy * sr * sp;

    get_pose.pose.orientation.w = 1;
    get_pose.pose.orientation.x = 0;
    get_pose.pose.orientation.y = 0;
    get_pose.pose.orientation.z = 0;

    state.marker = true;
  }
  else
  {
    state.marker = false;
  }

  return get_pose;
}


bool placeMsgCallback(open_manipulator_with_tb3_msgs::Place::Request &req,
                      open_manipulator_with_tb3_msgs::Place::Response &res)
{
  if ((state.arm == STOPPED) && (state.gripper == STOPPED))
  {
    res.result = "START PLACE 1 TASK!";
    //task = CHECK_AR_MARKER_POSE;
    task = MOVE_ARM;
  }
  else
  {
    res.result = "SOME TASKS IS WORKING";
    task = WAITING_FOR_SIGNAL;
  }
}

void armStateMsgCallback(const open_manipulator_msgs::State::ConstPtr &msg)
{
  std::string get_arm_state = msg->robot;

  if (get_arm_state == msg->STOPPED)
    state.arm = STOPPED;
  else if (get_arm_state == msg->IS_MOVING)
    state.arm = IS_MOVING;
}

void gripperStateMsgCallback(const open_manipulator_msgs::State::ConstPtr &msg)
{
  std::string get_gripper_state = msg->robot;

  if (get_gripper_state == msg->STOPPED)
    state.gripper = STOPPED;
  else if (get_gripper_state == msg->IS_MOVING)
    state.gripper = IS_MOVING;
}

void arMarkerPoseMsgCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{  
  if (msg->markers.size() == 0)
    return;

  ar_marker_pose = msg->markers[0];
}

void place()
{
  open_manipulator_msgs::SetKinematicsPose eef_pose;
  std_msgs::String result_msg;

  static uint8_t planning_cnt = 0;

  switch (task)
  {
    case CHECK_AR_MARKER_POSE:
      desired_pose = calcDesiredPose(ar_marker_pose);
      if (state.marker == true)
      {
        ROS_WARN("SAVE POSE OF AR MARKER");
        task = MOVE_ARM;
      }
      else
      {
        ROS_ERROR("CAN NOT FIND AR MARKER(ID : 8)");
        task = WAITING_FOR_SIGNAL;
      }
     break;

    case MOVE_ARM:
      if (state.arm == STOPPED)
      {
        ROS_WARN("MOVE ARM 1 TO PLACE");

	desired_pose.pose.position.x = 0.110 ;
	desired_pose.pose.position.y = 0.008 ;
	desired_pose.pose.position.z = 0.100 ;

	desired_pose.pose.orientation.x = 0.000 ;
	desired_pose.pose.orientation.y = -0.003 ;
	desired_pose.pose.orientation.z = 0.021 ;
	desired_pose.pose.orientation.w = 1.000 ;

        ROS_INFO("x = %.3f", desired_pose.pose.position.x);
        ROS_INFO("y = %.3f", desired_pose.pose.position.y);
        ROS_INFO("z = %.3f", desired_pose.pose.position.z);

        ROS_INFO("qx = %.3f", desired_pose.pose.orientation.x);
        ROS_INFO("qy = %.3f", desired_pose.pose.orientation.y);
        ROS_INFO("qz = %.3f", desired_pose.pose.orientation.z);
        ROS_INFO("qw = %.3f", desired_pose.pose.orientation.w);

        ROS_INFO("yaw = %.3f", yaw * RAD2DEG);

        eef_pose.request.kinematics_pose.group_name = "arm";
        eef_pose.request.kinematics_pose.pose = desired_pose.pose;
        eef_pose.request.kinematics_pose.max_velocity_scaling_factor = 0.1;
        eef_pose.request.kinematics_pose.max_accelerations_scaling_factor = 0.5;
        eef_pose.request.kinematics_pose.tolerance = tolerance;

        ros::service::waitForService(robot_name + "/set_kinematics_pose");
        if (kinematics_pose_command_client.call(eef_pose))
        {
          if (eef_pose.response.isPlanned == true)
          {
            ROS_INFO("PLANNING IS SUCCESSED");

            ros::WallDuration sleep_time(1.0);
            sleep_time.sleep();

            pre_task = MOVE_ARM;
            task = WAITING_FOR_STOP;
          }
          else
          {
            if (planning_cnt > 10)
            {
              result_msg.data = "Failed to plan";
              resultOfPlace(result_msg);

              planning_cnt = 0;
              task = WAITING_FOR_SIGNAL;
            }
            else
            {
              planning_cnt++;

              tolerance += 0.005;
              ROS_ERROR("PLANNING IS FAILED (%d, tolerance : %.2f)", planning_cnt, tolerance);

              task = MOVE_ARM;
            }
          }
        }
      }
     break;

    case CLOSE_TO_BOX:
      if (state.arm == STOPPED)
      {
        ROS_WARN("CLOSE TO BOX");

	geometry_msgs::Twist msg;
	msg.linear.x = 0.12;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
	cmd_vel_pub.publish(msg);

	ros::WallDuration sleep_time(2.0);
        sleep_time.sleep();

	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
	cmd_vel_pub.publish(msg);

	pre_task = CLOSE_TO_BOX;
        task = WAITING_FOR_STOP;

        /*geometry_msgs::PoseStamped object_pose = desired_pose;

        object_pose.pose.position.x = object_pose.pose.position.x + (DIST_EDGE_TO_CENTER_OF_PALM + dist_ar_marker_to_box);

        ROS_INFO("x = %.3f", object_pose.pose.position.x);
        ROS_INFO("y = %.3f", object_pose.pose.position.y);
        ROS_INFO("z = %.3f", object_pose.pose.position.z);

        ROS_INFO("qx = %.3f", object_pose.pose.orientation.x);
        ROS_INFO("qy = %.3f", object_pose.pose.orientation.y);
        ROS_INFO("qz = %.3f", object_pose.pose.orientation.z);
        ROS_INFO("qw = %.3f", object_pose.pose.orientation.w);

        ROS_INFO("yaw = %.3f", yaw * RAD2DEG);

        eef_pose.request.kinematics_pose.group_name = "arm";
        eef_pose.request.kinematics_pose.pose = object_pose.pose;

        eef_pose.request.kinematics_pose.max_velocity_scaling_factor = 0.1;
        eef_pose.request.kinematics_pose.max_accelerations_scaling_factor = 0.1;
        eef_pose.request.kinematics_pose.tolerance = tolerance;

        ros::service::waitForService(robot_name + "/set_kinematics_pose");
        if (kinematics_pose_command_client.call(eef_pose))
        {
          if (eef_pose.response.isPlanned == true)
          {
            ROS_INFO("PLANNING IS SUCCESSED");

            ros::WallDuration sleep_time(1.0);
            sleep_time.sleep();

            pre_task = CLOSE_TO_BOX;
            task = WAITING_FOR_STOP;
          }
          else
          {
            if (planning_cnt > 10)
            {
              result_msg.data = "Failed to plan";
              resultOfPlace(result_msg);

              planning_cnt = 0;
              task = WAITING_FOR_SIGNAL;
            }
            else
            {
              planning_cnt++;
              tolerance += 0.005;
              ROS_ERROR("PLANNING IS FAILED (%d, tolerance : %.2f)", planning_cnt, tolerance);

              task = CLOSE_TO_BOX;
            }
          }
        }*/
      }
     break;

    case PLACE_OBJECT:
      if (state.gripper == STOPPED)
      {
        ROS_WARN("PLACE OBJECT IN THE BOX");

        bool result = gripper(OFF);

        if (result)
        {
          ROS_INFO("Griip PLANNING IS SUCCESSED");

          ros::WallDuration sleep_time(3.0);
          sleep_time.sleep();

	  geometry_msgs::Twist msg;
	  msg.linear.x = -0.12;
	  msg.linear.y = 0;
	  msg.linear.z = 0;
	  msg.angular.x = 0;
	  msg.angular.y = 0;
	  msg.angular.z = 0;
	  cmd_vel_pub.publish(msg);

	  ros::WallDuration sleep_time2(2.5);
          sleep_time2.sleep();

	  msg.linear.x = 0;
	  msg.linear.y = 0;
	  msg.linear.z = 0;
	  msg.angular.x = 0;
	  msg.angular.y = 0;
	  msg.angular.z = 0;
	  cmd_vel_pub.publish(msg);

          pre_task = PLACE_OBJECT;
          task = WAITING_FOR_STOP;

        }
        else
        {
          planning_cnt++;
          ROS_ERROR("PLANNING IS FAILED (%d)", planning_cnt);
          task = PLACE_OBJECT;
        }	

      }
     break;

    case INIT_POSITION:
      if (state.arm == STOPPED)
      {
        ROS_WARN("SET INIT POSITION");

        bool result = initJointPosition();

        if (result)
        {
          ROS_INFO("PLANNING IS SUCCESSED");

          ros::WallDuration sleep_time(1.0);
          sleep_time.sleep();

          pre_task = INIT_POSITION;
          task = WAITING_FOR_STOP;
        }
        else
        {
          planning_cnt++;
          ROS_ERROR("PLANNING IS FAILED (%d)", planning_cnt);
          task = INIT_POSITION;
        }
      }
     break;

    case WAITING_FOR_STOP:
      if ((state.arm == STOPPED) && (state.gripper == STOPPED))
      {
        tolerance = 0.01;
        planning_cnt = 0;

        if (pre_task == INIT_POSITION)
        {
          task = WAITING_FOR_SIGNAL;

          result_msg.data = "Success";
          resultOfPlace(result_msg);
        }
        else
          task = pre_task + 1;

      }
     break;

    default:
     break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "place");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  priv_nh.getParam("robot_name", robot_name);
  priv_nh.getParam("marker_id", get_marker_id);
  priv_nh.getParam("offset_for_object_height", offset_for_object_height);
  priv_nh.getParam("dist_ar_marker_to_box", dist_ar_marker_to_box);

  joint_position_command_client = nh.serviceClient<open_manipulator_msgs::SetJointPosition>(robot_name + "/set_joint_position");
  kinematics_pose_command_client = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>(robot_name + "/set_kinematics_pose");

  gripper_position_command_client = nh.serviceClient<open_manipulator_msgs::SetJointPosition>(robot_name + "/set_gripper_position");

  place_result_client = nh.serviceClient<open_manipulator_with_tb3_msgs::Place>(robot_name + "/result_of_place");

  ros::Subscriber arm_state_sub = nh.subscribe(robot_name + "/arm_state", 10, armStateMsgCallback);
  ros::Subscriber gripper_state_sub = nh.subscribe(robot_name + "/gripper_state", 10, gripperStateMsgCallback);
  ros::Subscriber ar_marker_pose_sub = nh.subscribe("/ar_pose_marker", 10, arMarkerPoseMsgCallback);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);


  ros::ServiceServer place_server = nh.advertiseService(robot_name + "/place", placeMsgCallback);
 
  ROS_INFO("Ready to Place 2 Task");

  ros::Rate loop_rate(25);

  while (ros::ok())
  {
    place();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
