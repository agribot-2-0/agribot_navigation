/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */
#pragma once
#include <agribot_common/CommonFunctions/ros_common_functions.h>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"

class VelocitySmoother
{
private:
  ros::NodeHandle* nh_;
  ros::Publisher smooth_velocity_publisher_;
  ros::Subscriber raw_velocity_subscriber_;
  double last_sent_filtered_velocity_ = 0;
  double acceleration_;  // m/s^2
  double current_raw_velocity_ = 0;
  double frequency_;  // 1/s
  double current_filtered_velocity_ = 0;
  const double stop_ = 0;
  double angular_velocity_;
  double velocity_treshold_;
  double acceleration_after_treshold_;
  double step_;
  double step_after_treshold_;

public:
  explicit VelocitySmoother(ros::NodeHandle* nh);
  int checkParameters();
  void run();
  void rawVelocityCallback(const geometry_msgs::Twist msg);
  bool isCloserToTresholdThanToDesiredVelocity();
  bool isCloserToDesiredVelocityThanByStep();
  bool isBelowTreshold();
  bool isCloserToDesiredVelocityThanByStepAfterTreshold();
  bool isBelowTresholdAndIncreasing();
  bool isAboveTresholdAndDecreasing();
  void calculateCurrentFilteredVelocity();
};
