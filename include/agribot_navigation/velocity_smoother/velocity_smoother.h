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
  double last_sent_filtered_velocity_{ 0.0 };
  double acceleration_{ 0.0 };
  double current_raw_velocity_{ 0.0 };
  double frequency_{ 0.0 };
  double current_filtered_velocity_{ 0.0 };
  double angular_velocity_{ 0.0 };
  double velocity_threshold_{ 0.0 };
  double acceleration_after_threshold_{ 0.0 };
  double step_{ 0.0 };
  double step_after_threshold_{ 0.0 };

public:
  explicit VelocitySmoother(ros::NodeHandle* nh);
  void getParameters();
  void run();
  void rawVelocityCallback(const geometry_msgs::Twist msg);
  bool isCloserToThresholdThanToDesiredVelocity();
  bool isCloserToDesiredVelocityThanByStep();
  bool isBelowThreshold();
  bool isCloserToDesiredVelocityThanByStepAfterThreshold();
  bool isCloseToThresholdAndIncreasing();
  bool isCloseToThresholdAndDecreasing();
  void calculateCurrentFilteredVelocity();
  bool isCloseToThreshold();
  bool isIncreasing();
  bool isDecreasing();
};
