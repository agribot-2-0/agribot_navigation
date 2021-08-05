/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#include "agribot_navigation/cmd_vel_arbiter/CmdVelArbiterNode.h"

#include <agribot_common/CommonFunctions/ros_common_functions.h>

CmdVelArbiterNode::CmdVelArbiterNode(ros::NodeHandle* nh) : _nh(*nh), _arbiterState(ArbiterState::NO_SELECTION)
{
  // Out topic
  _outPub = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);

  // Read topic parameters
  _navTopic = "/cmd_vel_navigation";
  _teleopTopic = "/cmd_vel_manual";

  std::string startingState;

  try
  {
    startingState = ros_common_functions::getRosParam<std::string>(_nh.getNamespace() + "/starting_state");
    timeout = ros_common_functions::getRosParam<double>(_nh.getNamespace() + "/timeout");
    minRadius = ros_common_functions::getRosParam<double>(_nh.getNamespace() + "/min_radius");
    cutoffRadius = ros_common_functions::getRosParam<double>(_nh.getNamespace() + "/cutoff_radius");
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("[KinematicsController] " << ex.what());
    _nh.shutdown();
    exit(0);
  }

  if (!switchToStartingState(startingState))
  {
    ROS_ERROR_STREAM("Unsupported starting_state " << startingState << " provided. Exiting...");
    _nh.shutdown();
    exit(0);
  }

  // Create one-shot timeout timer and leave it in stopped state
  createTimeoutTimer();
  stopTimeoutTimer();
  ROS_INFO_STREAM("Created cmd_vel timeout timer with " << timeout << "s...");

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

bool CmdVelArbiterNode::switchToStartingState(const std::string& starting_state)
{
  if (starting_state == "stop")
  {
    return switchToStop();
  }
  else if (starting_state == "navigation")
  {
    return switchToNavigation();
  }
  else if (starting_state == "teleop")
  {
    return switchToTeleop();
  }
  else
  {
    return false;
  }
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

    ROS_WARN_STREAM("[CmdVelArbiter] cmd_vel switched to stop");
  }
  else
  {
    ROS_WARN_STREAM("[CmdVelArbiter] cmd_vel IS ALREADY switched to stop");
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

    ROS_WARN_STREAM("[CmdVelArbiter] cmd_vel switched to navigation (topic: " << _navTopic << ")");
  }
  else
  {
    ROS_WARN_STREAM("[CmdVelArbiter] cmd_vel IS ALREADY switched to navigation");
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

    ROS_WARN_STREAM("[CmdVelArbiter] cmd_vel switched to teleop (topic: " << _teleopTopic << ")");
  }
  else
  {
    ROS_WARN_STREAM("[CmdVelArbiter] cmd_vel IS ALREADY switched to teleop");
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
  static_cast<void>(req);
  static_cast<void>(res);
  return switchToStop();
}

bool CmdVelArbiterNode::switchToNavigationSrvCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  static_cast<void>(req);
  static_cast<void>(res);
  return switchToNavigation();
}

bool CmdVelArbiterNode::switchToTeleopSrvCb(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  static_cast<void>(req);
  static_cast<void>(res);
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
  _outPub.publish(CommonFunctions::teleopToVel(msg, minRadius, cutoffRadius));

  // Restart timeoutTimer
  restartTimeoutTimer();
}

void CmdVelArbiterNode::timeoutCallback(const ros::TimerEvent& event)
{
  static_cast<void>(event);
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
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
