/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
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

/* Authors: Ryan Shim */

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ur_rh_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ur_rh_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv) 
    :init_argc(argc),
     init_argv(argv),
     open_manipulator_actuator_enabled_(false),
     open_manipulator_is_moving_(false)
{}

QNode::~QNode()
{
  if(ros::isStarted()) 
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
	wait();
}

bool QNode::init()
{
	ros::init(init_argc,init_argv,"ur_rh_gui");
	if ( ! ros::master::check() ) 
  {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n("");
  ros::NodeHandle priv_n("~");

  // Moveit 
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  std::string planning_group_name = "manipulator";
  move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name);

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(planning_group_name);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(planning_group_name);

  robot_state::RobotState start_state(*move_group.getCurrentState());

  // geometry_msgs::Pose current_pose = (move_group_->getCurrentPose()).pose;

  // msg publisher
  open_manipulator_option_pub_ = n.advertise<std_msgs::String>("option", 10);

  // msg subscriber
  open_manipulator_states_sub_       = n.subscribe("states", 10, &QNode::manipulatorStatesCallback, this);
  open_manipulator_joint_states_sub_ = n.subscribe("joint_states", 10, &QNode::jointStatesCallback, this);

  // service client
  goal_joint_space_path_client_ = n.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_task_space_path_position_only_client_ = n.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
  goal_task_space_path_client_= n.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path");
  goal_tool_control_client_ = n.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
  set_actuator_state_client_ = n.serviceClient<open_manipulator_msgs::SetActuatorState>("set_actuator_state");

  start();
	return true;
}

void QNode::run()
{
  ros::Rate loop_rate(10);
	while ( ros::ok() )
  {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown();
}

void QNode::manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
{
  if(msg->open_manipulator_moving_state == msg->IS_MOVING)
    open_manipulator_is_moving_ = true;
  else
    open_manipulator_is_moving_ = false;

  if(msg->open_manipulator_actuator_state == msg->ACTUATOR_ENABLED)
    open_manipulator_actuator_enabled_ = true;
  else
    open_manipulator_actuator_enabled_ = false;
}

void QNode::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT_AND_TOOL - 1);
  if (0) temp_angle.resize(NUM_OF_JOINT_AND_TOOL);

  for(int i = 0; i < msg->name.size(); i ++)
  {
    if     (!msg->name.at(i).compare("shoulder_pan_joint"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("shoulder_lift_joint")) temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("elbow_joint"))         temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("wrist_1_joint"))       temp_angle.at(3) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("wrist_2_joint"))       temp_angle.at(4) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("wrist_3_joint"))       temp_angle.at(5) = (msg->position.at(i));
    
    if (0)
    {
      if(!msg->name.at(i).compare("gripper")) temp_angle.at(6) = (msg->position.at(i));      
    }
  }
  present_joint_angle_ = temp_angle;


  ros::AsyncSpinner spinner(1); 
  spinner.start();

  geometry_msgs::Pose current_pose = move_group_->getCurrentPose().pose;  
  std::vector<double> temp_position;
  temp_position.push_back(current_pose.position.x);
  temp_position.push_back(current_pose.position.y);
  temp_position.push_back(current_pose.position.z);
  present_kinematics_position_ = temp_position;

  Eigen::Quaterniond temp_orientation(
    current_pose.orientation.w, 
    current_pose.orientation.x,
    current_pose.orientation.y, 
    current_pose.orientation.z);

  present_kinematics_orientation_ = temp_orientation;
}

std::vector<double> QNode::getPresentJointAngle()
{
  return present_joint_angle_;
}

std::vector<double> QNode::getPresentKinematicsPosition()
{
  return present_kinematics_position_;
}

Eigen::Quaterniond QNode::getPresentKinematicsOrientation()
{
  return present_kinematics_orientation_;
}

Eigen::Vector3d QNode::getPresentKinematicsOrientationRPY()
{
  present_kinematics_orientation_rpy_ = robotis_manipulator::math::convertQuaternionToRPYVector(present_kinematics_orientation_);

  return present_kinematics_orientation_rpy_;
}

bool QNode::getOpenManipulatorMovingState()
{
  return open_manipulator_is_moving_;
}

bool QNode::getOpenManipulatorActuatorState()
{
  return open_manipulator_actuator_enabled_;
}

void QNode::setOption(std::string opt)
{
  std_msgs::String msg;
  msg.data = opt;
  open_manipulator_option_pub_.publish(msg);
}

bool QNode::setJointSpacePath(std::vector<double> joint_angle, double path_time)
{
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  // Next get the current set of joint values for the group.
  const robot_state::JointModelGroup* joint_model_group =
      move_group_->getCurrentState()->getJointModelGroup("manipulator");
      
  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = joint_angle.at(0);  // radians
  joint_group_positions[1] = joint_angle.at(1);  // radians
  joint_group_positions[2] = joint_angle.at(2);  // radians
  joint_group_positions[3] = joint_angle.at(3);  // radians
  joint_group_positions[4] = joint_angle.at(4);  // radians
  joint_group_positions[5] = joint_angle.at(5);  // radians
  move_group_->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group_->move();

  spinner.stop();
  return true;
}

bool QNode::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  // move_group_->setMaxVelocityScalingFactor(0.1);
  // move_group_->setMaxAccelerationScalingFactor(0.1);
  move_group_->setGoalTolerance(0.1);

  geometry_msgs::Pose target_pose1;
  target_pose1.position.x = kinematics_pose.at(0);
  target_pose1.position.y = kinematics_pose.at(1);
  target_pose1.position.z = kinematics_pose.at(2);
  target_pose1.orientation.w = kinematics_pose.at(3);
  target_pose1.orientation.x = kinematics_pose.at(4);
  target_pose1.orientation.y = kinematics_pose.at(5);
  target_pose1.orientation.z = kinematics_pose.at(6);
  move_group_->setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group_->move();

  spinner.stop();
  return true;
}

bool QNode::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back("gripper");
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool QNode::setActuatorState(bool actuator_state)
{
  open_manipulator_msgs::SetActuatorState srv;
  srv.request.set_actuator_state = actuator_state;

  if(set_actuator_state_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}
}  // namespace ur_rh_gui
