/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/cmd_vel_arbiter/CmdVelArbiterNode.h"

CmdVelArbiterNode::CmdVelArbiterNode(ros::NodeHandle* nh) : _nh(*nh), _arbiterState(ArbiterState::NO_SELECTION)
{
  // Out topic
  _outPub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);

  // Read topic parameters
  _navTopic = "/cmd_vel_navigation";
  _teleopTopic = "/cmd_vel_manual";

  std::string startingState;
  _nh.param<std::string>("starting_state", startingState, "stop");
  _nh.param<float>("timeout", timeout, 0.5);
  timeout = 600.0;
  _nh.param<float>("chassis_length", chassisLength, 2.1);
  _nh.param<float>("min_radius", minRadius, 2.5);
  _nh.param<float>("cutoff_radius", cutoffRadius, 11.0);

  // Create one-shot timeout timer and leave it in stopped state
  createTimeoutTimer();
  stopTimeoutTimer();

  ROS_INFO_STREAM("Created cmd_vel timeout timer with " << timeout << "s...");

  if (startingState == "stop")
  {
    switchToStop();
  }
  else if (startingState == "navigation")
  {
    switchToNavigation();
  }
  else if (startingState == "teleop")
  {
    switchToTeleop();
  }
  else
  {
    ROS_ERROR_STREAM("Unsuported starting_state provided. Exiting...");
    exit(0);
  }

  // Switching services
  switchToStopService = _nh.advertiseService("switch_to_stop", &CmdVelArbiterNode::switchToStopSrvCb, this);
  switchToNavigationService =
      _nh.advertiseService("switch_to_navigation", &CmdVelArbiterNode::switchToNavigationSrvCb, this);
  switchToTeleopService = _nh.advertiseService("switch_to_teleop", &CmdVelArbiterNode::switchToTeleopSrvCb, this);
}

void CmdVelArbiterNode::subscribeToNavigation()
{
  _inSub = _nh.subscribe(_navTopic, 1, &CmdVelArbiterNode::navCallback, this);
}

void CmdVelArbiterNode::subscribeToTeleop()
{
  _inSub = _nh.subscribe(_teleopTopic, 1, &CmdVelArbiterNode::teleopCallback, this);
}

bool CmdVelArbiterNode::switchToStop()
{
  if (_arbiterState != ArbiterState::STOP)
  {
    // Disconnect previous subscriber
    _inSub.shutdown();

    // Set ArbiterState
    _arbiterState = ArbiterState::STOP;

    // Stop timeoutTimer
    stopTimeoutTimer();

    // Publish zeros to cmd_vel once to stop robot
    publishStopToCmdVel();

    ROS_INFO_STREAM("cmd_vel switched to stop");
  }
  else
  {
    ROS_WARN_STREAM("cmd_vel IS ALREADY switched to stop");
  }

  return true;
}

bool CmdVelArbiterNode::switchToNavigation()
{
  if (_arbiterState != ArbiterState::NAVIGATION)
  {
    // Disconnect previous subscriber
    _inSub.shutdown();

    // Set ArbiterState
    _arbiterState = ArbiterState::NAVIGATION;

    // Start timeoutTimer
    startTimeoutTimer();

    // Subscribe to navigation cmd_vel
    subscribeToNavigation();

    ROS_INFO_STREAM("cmd_vel switched to navigation (topic: " << _navTopic << ")");
  }
  else
  {
    ROS_WARN_STREAM("cmd_vel IS ALREADY switched to navigation");
  }

  return true;
}

bool CmdVelArbiterNode::switchToTeleop()
{
  if (_arbiterState != ArbiterState::TELEOP)
  {
    // Disconnect previous subscriber
    _inSub.shutdown();

    // Set ArbiterState
    _arbiterState = ArbiterState::TELEOP;

    // Start timeoutTimer
    startTimeoutTimer();

    // Subscribe to teleop cmd_vel
    subscribeToTeleop();

    ROS_INFO_STREAM("cmd_vel switched to teleop (topic: " << _teleopTopic << ")");
  }
  else
  {
    ROS_WARN_STREAM("cmd_vel IS ALREADY switched to teleop");
  }

  return true;
}

void CmdVelArbiterNode::createTimeoutTimer()
{
  timeoutTimer = _nh.createTimer(ros::Duration(timeout), &CmdVelArbiterNode::timeoutCallback, this, true);
}

void CmdVelArbiterNode::stopTimeoutTimer()
{
  timeoutTimer.stop();
}

void CmdVelArbiterNode::startTimeoutTimer()
{
  timeoutTimer.start();
}

void CmdVelArbiterNode::restartTimeoutTimer()
{
  timeoutTimer.stop();
  timeoutTimer.start();
}

bool CmdVelArbiterNode::switchToStopSrvCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  return switchToStop();
}

bool CmdVelArbiterNode::switchToNavigationSrvCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  return switchToNavigation();
}

bool CmdVelArbiterNode::switchToTeleopSrvCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  return switchToTeleop();
}

void CmdVelArbiterNode::navCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  // Rewrite message
  _outPub.publish(msg);

  // Restart timeoutTimer
  restartTimeoutTimer();
}

void CmdVelArbiterNode::teleopCallback(const agribot_msgs::AgribotManualControl& msg)
{
  // Rewrite message
  _outPub.publish(teleopToVel(msg));

  // Restart timeoutTimer
  restartTimeoutTimer();
}

geometry_msgs::Twist CmdVelArbiterNode::teleopToVel(const agribot_msgs::AgribotManualControl& in_msg)
{
  geometry_msgs::Twist out_msg;
  out_msg.linear.x = in_msg.linear_velocity;
  if (abs(in_msg.circle_radius) > minRadius && abs(in_msg.circle_radius) < cutoffRadius)
  {
    out_msg.angular.z = in_msg.linear_velocity / in_msg.circle_radius;
  }
  else if (abs(in_msg.circle_radius) < minRadius && in_msg.circle_radius > 0)
  {
    out_msg.angular.z = in_msg.linear_velocity / minRadius;
  }
  else if (abs(in_msg.circle_radius) < minRadius && in_msg.circle_radius < 0)
  {
    out_msg.angular.z = -in_msg.linear_velocity / minRadius;
  }
  return out_msg;
}

void CmdVelArbiterNode::timeoutCallback(const ros::TimerEvent& event)
{
  ROS_ERROR_STREAM("cmd_vel timeout exceeded " << timeout << "s...");

  switchToStop();
}

void CmdVelArbiterNode::publishStopToCmdVel()
{
  // Publish zero velocities to stop the robot
  geometry_msgs::Twist msg;
  _outPub.publish(msg);
}

void CmdVelArbiterNode::run()
{
  ros::spin();
}
