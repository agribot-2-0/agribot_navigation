/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */
#pragma once
#include <agribot_common/CommonFunctions/ros_common_functions.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class VelocitySmoother
{
private:
  ros::NodeHandle* nh_;
  ros::Publisher smooth_velocity_publisher_;
  ros::Subscriber raw_velocity_subscriber_;
  double last_sent_filtered_velocity_ = 0;
  double acceleration_;
  double current_raw_velocity_ = 0;
  double frequency_;
  double current_filtered_velocity_ = 0;
  const double stop_ = 0;
  double angular_velocity_;
  double velocity_threshold_;
  double acceleration_after_threshold_;
  double step_;
  double step_after_threshold_;

public:
  explicit VelocitySmoother(ros::NodeHandle* nh);
  int getParameters();
  void run();
  void rawVelocityCallback(const geometry_msgs::Twist msg);
  bool isCloserToThresholdThanToDesiredVelocity();
  bool isCloserToDesiredVelocityThanByStep();
  bool isBelowThreshold();
  bool isCloserToDesiredVelocityThanByStepAfterThreshold();
  bool isBelowThresholdAndIncreasing();
  bool isAboveThresholdAndDecreasing();
  void calculateCurrentFilteredVelocity();
};
