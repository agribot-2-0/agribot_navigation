/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */
#include <agribot_navigation/velocity_smoother/velocity_smoother.hh>

VelocitySmoother::VelocitySmoother(ros::NodeHandle* nh)
{
  nh_ = nh;
  smooth_velocity_publisher_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel_filtered", 1);
  raw_velocity_subscriber_ = nh_->subscribe("/cmd_vel", 1, &VelocitySmoother::rawVelocityCallback, this);
}

void VelocitySmoother::getParameters()
{
  frequency_ = ros_common_functions::getRosParam<double>("/default_frequency");
  acceleration_ = ros_common_functions::getRosParam<double>("/default_acceleration");
  velocity_threshold_ = ros_common_functions::getRosParam<double>("/default_velocity_threshold");
  acceleration_after_threshold_ = ros_common_functions::getRosParam<double>("/default_acceleration_after_threshold");

  step_ = acceleration_ / frequency_;
  step_after_threshold_ = acceleration_after_threshold_ / frequency_;
}

bool VelocitySmoother::isCloserToDesiredVelocityThanByStep()
{
  return (abs(last_sent_filtered_velocity_ - current_raw_velocity_) <= step_);
}

bool VelocitySmoother::isCloserToThresholdThanToDesiredVelocity()
{
  return (abs(last_sent_filtered_velocity_ - current_raw_velocity_) >
          abs(abs(last_sent_filtered_velocity_) - velocity_threshold_));
}

bool VelocitySmoother::isBelowThreshold()
{
  return ((abs(last_sent_filtered_velocity_) <= velocity_threshold_));
}

bool VelocitySmoother::isCloserToDesiredVelocityThanByStepAfterThreshold()
{
  return (abs(last_sent_filtered_velocity_ - current_raw_velocity_) <= step_after_threshold_);
}

bool VelocitySmoother::isBelowThresholdAndIncreasing()
{
  return ((abs(abs(last_sent_filtered_velocity_) - velocity_threshold_) < step_after_threshold_) &&
          (abs(last_sent_filtered_velocity_) > velocity_threshold_) &&
          (((current_raw_velocity_ - last_sent_filtered_velocity_ < 0) && (last_sent_filtered_velocity_ > 0)) ||
           ((current_raw_velocity_ - last_sent_filtered_velocity_ > 0) && (last_sent_filtered_velocity_ < 0))));
}

bool VelocitySmoother::isAboveThresholdAndDecreasing()
{
  return ((abs(last_sent_filtered_velocity_) < velocity_threshold_) &&
          (abs(abs(last_sent_filtered_velocity_) - velocity_threshold_) < step_after_threshold_) &&
          (((current_raw_velocity_ - last_sent_filtered_velocity_ < 0) && (last_sent_filtered_velocity_ < 0)) ||
           ((current_raw_velocity_ - last_sent_filtered_velocity_ > 0) && (last_sent_filtered_velocity_ > 0))));
}

void VelocitySmoother::calculateCurrentFilteredVelocity()
{
  if (isCloserToDesiredVelocityThanByStep())
  {
    if (isCloserToThresholdThanToDesiredVelocity() && isAboveThresholdAndDecreasing())
    {
      current_filtered_velocity_ =
          (last_sent_filtered_velocity_ > current_raw_velocity_ ? last_sent_filtered_velocity_ - step_after_threshold_ :
                                                                  last_sent_filtered_velocity_ + step_after_threshold_);
    }
    else if (isBelowThreshold())
    {
      if (isCloserToDesiredVelocityThanByStepAfterThreshold())
      {
        current_filtered_velocity_ = current_raw_velocity_;
      }
      else
      {
        current_filtered_velocity_ = (last_sent_filtered_velocity_ > current_raw_velocity_ ?
                                          last_sent_filtered_velocity_ - step_after_threshold_ :
                                          last_sent_filtered_velocity_ + step_after_threshold_);
      }
    }
    else
    {
      current_filtered_velocity_ = current_raw_velocity_;
    }
  }
  else if (isBelowThresholdAndIncreasing())
  {
    current_filtered_velocity_ =
        (last_sent_filtered_velocity_ > current_raw_velocity_ ? last_sent_filtered_velocity_ - step_after_threshold_ :
                                                                last_sent_filtered_velocity_ + step_after_threshold_);
  }
  else if (isAboveThresholdAndDecreasing())
  {
    current_filtered_velocity_ =
        (last_sent_filtered_velocity_ > current_raw_velocity_ ? last_sent_filtered_velocity_ - step_after_threshold_ :
                                                                last_sent_filtered_velocity_ + step_after_threshold_);
  }
  else if (isBelowThreshold())
  {
    current_filtered_velocity_ =
        (last_sent_filtered_velocity_ > current_raw_velocity_ ? last_sent_filtered_velocity_ - step_after_threshold_ :
                                                                last_sent_filtered_velocity_ + step_after_threshold_);
  }
  else
  {
    current_filtered_velocity_ =
        (last_sent_filtered_velocity_ > current_raw_velocity_ ? last_sent_filtered_velocity_ - step_ :
                                                                last_sent_filtered_velocity_ + step_);
  }
}

void VelocitySmoother::run()
{
  geometry_msgs::Twist filtered_msg;

  ros::Rate loop_rate(frequency_);
  while (ros::ok())
  {
    ros::spinOnce();

    this->calculateCurrentFilteredVelocity();
    filtered_msg.linear.x = current_filtered_velocity_;
    filtered_msg.angular.z = angular_velocity_;
    smooth_velocity_publisher_.publish(filtered_msg);
    last_sent_filtered_velocity_ = current_filtered_velocity_;

    loop_rate.sleep();
  }
}

void VelocitySmoother::rawVelocityCallback(const geometry_msgs::Twist msg)
{
  current_raw_velocity_ = msg.linear.x;
  angular_velocity_ = msg.angular.z;
}
