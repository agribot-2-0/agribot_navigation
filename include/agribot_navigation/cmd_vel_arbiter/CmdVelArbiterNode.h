/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>

#include <agribot_msgs/AgribotManualControl.h>

#include <string>

enum class ArbiterState
{
  NO_SELECTION,
  STOP,
  NAVIGATION,
  TELEOP
};

class CmdVelArbiterNode
{
private:
  ros::NodeHandle _nh;
  ros::Publisher _outPub;
  ros::Subscriber _inSub;

  ros::ServiceServer switchToStopService;
  ros::ServiceServer switchToNavigationService;
  ros::ServiceServer switchToTeleopService;

  ros::Timer timeoutTimer;
  float timeout;
  float minRadius;
  float cutoffRadius;

  ArbiterState _arbiterState;
  std::string _navTopic;
  std::string _teleopTopic;

public:
  explicit CmdVelArbiterNode(ros::NodeHandle* nh);

  void subscribeToNavigation();
  void subscribeToTeleop();
  bool switchToStartingState(const std::string& starting_state);
  bool switchToStop();
  bool switchToNavigation();
  bool switchToTeleop();

  void createTimeoutTimer();
  void stopTimeoutTimer();
  void startTimeoutTimer();
  void restartTimeoutTimer();

  bool switchToStopSrvCb(std_srvs::Empty::Request& req,          // NOLINT(runtime/references)
                         std_srvs::Empty::Response& res);        // NOLINT(runtime/references)
  bool switchToNavigationSrvCb(std_srvs::Empty::Request& req,    // NOLINT(runtime/references)
                               std_srvs::Empty::Response& res);  // NOLINT(runtime/references)
  bool switchToTeleopSrvCb(std_srvs::Empty::Request& req,        // NOLINT(runtime/references)
                           std_srvs::Empty::Response& res);      // NOLINT(runtime/references)

  void navCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void teleopCallback(const agribot_msgs::AgribotManualControl& msg);
  geometry_msgs::Twist teleopToVel(const agribot_msgs::AgribotManualControl& in_msg);
  void timeoutCallback(const ros::TimerEvent& event);
  void publishStopToCmdVel();
  void run();
};
