/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */
#include <agribot_navigation/velocity_smoother/velocity_smoother.hh>

VelocitySmoother::VelocitySmoother(ros::NodeHandle* nh)
{
  nh_ = nh;
  smooth_velocity_publisher_ = nh_->advertise<geometry_msgs::Twist>("/cmd_vel_filtered", 1);
  raw_velocity_subscriber_ = nh_->subscribe("/cmd_vel", 1, &VelocitySmoother::rawVelocityCallback, this);
}

int VelocitySmoother::checkParameters()
{
  try
  {
    frequency_ = ros_common_functions::getRosParam<double>("/default_frequency");
    acceleration_ = ros_common_functions::getRosParam<double>("/default_acceleration");
    velocity_treshold_ = ros_common_functions::getRosParam<double>("/default_velocity_treshold");
    acceleration_after_treshold_ = ros_common_functions::getRosParam<double>("/default_acceleration_after_treshold");
  }

  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("[VelocitySmoother] " << ex.what());
    return 0;
  }
  step_ = acceleration_ / frequency_;
  step_after_treshold_ = acceleration_after_treshold_ / frequency_;
  return 1;
}

bool VelocitySmoother::isCloserToDesiredVelocityThanStep()
{
  return (abs(last_sent_filtered_velocity_ - current_raw_velocity_) <= step_);
}

bool VelocitySmoother::isCloserToTresholdThanDesiredVelocity()
{
  return ((abs(abs(last_sent_filtered_velocity_) - velocity_treshold_) < step_) &&
          (abs(last_sent_filtered_velocity_) != velocity_treshold_) &&
          (abs(last_sent_filtered_velocity_ - current_raw_velocity_) >
           abs(abs(last_sent_filtered_velocity_) - velocity_treshold_)) &&
          (((current_raw_velocity_ - last_sent_filtered_velocity_ < 0) && (last_sent_filtered_velocity_ < 0)) ||
           ((current_raw_velocity_ - last_sent_filtered_velocity_ > 0) && (last_sent_filtered_velocity_ > 0))));
}

bool VelocitySmoother::isBelowTreshold()
{
  return ((abs(last_sent_filtered_velocity_) <= velocity_treshold_));
}

bool VelocitySmoother::isCloserToDesiredVelocityThanStepAfterTreshold()
{
  return (abs(last_sent_filtered_velocity_ - current_raw_velocity_) <= step_after_treshold_);
}

bool VelocitySmoother::isBelowTresholdAndIncreasing()
{
  return ((abs(abs(last_sent_filtered_velocity_) - velocity_treshold_) < step_) &&
          (abs(last_sent_filtered_velocity_) > velocity_treshold_) &&
          (((current_raw_velocity_ - last_sent_filtered_velocity_ < 0) && (last_sent_filtered_velocity_ > 0)) ||
           ((current_raw_velocity_ - last_sent_filtered_velocity_ > 0) && (last_sent_filtered_velocity_ < 0))));
}

bool VelocitySmoother::isAboveTresholdAndDecreasing()
{
  return ((abs(last_sent_filtered_velocity_) < velocity_treshold_) &&
          (abs(abs(last_sent_filtered_velocity_) - velocity_treshold_) < step_after_treshold_) &&
          (((current_raw_velocity_ - last_sent_filtered_velocity_ < 0) && (last_sent_filtered_velocity_ < 0)) ||
           ((current_raw_velocity_ - last_sent_filtered_velocity_ > 0) && (last_sent_filtered_velocity_ > 0))));
}

void VelocitySmoother::calculateCurrentFilteredVelocity()
{
  if (isCloserToDesiredVelocityThanStep())
  {
    if (isCloserToTresholdThanDesiredVelocity())
    {
      current_filtered_velocity_ =
          (last_sent_filtered_velocity_ > current_raw_velocity_ ? last_sent_filtered_velocity_ - step_after_treshold_ :
                                                                  last_sent_filtered_velocity_ + step_after_treshold_);
    }
    else if (isBelowTreshold())
    {
      if (isCloserToDesiredVelocityThanStepAfterTreshold())
      {
        current_filtered_velocity_ = current_raw_velocity_;
      }
      else
      {
        current_filtered_velocity_ = (last_sent_filtered_velocity_ > current_raw_velocity_ ?
                                          last_sent_filtered_velocity_ - step_after_treshold_ :
                                          last_sent_filtered_velocity_ + step_after_treshold_);
      }
    }
    else
    {
      current_filtered_velocity_ = current_raw_velocity_;
    }
  }
  else if (isBelowTresholdAndIncreasing())
  {
    current_filtered_velocity_ =
        (last_sent_filtered_velocity_ > current_raw_velocity_ ? last_sent_filtered_velocity_ - step_after_treshold_ :
                                                                last_sent_filtered_velocity_ + step_after_treshold_);
  }
  else if (isAboveTresholdAndDecreasing())
  {
    current_filtered_velocity_ =
        (last_sent_filtered_velocity_ > current_raw_velocity_ ? last_sent_filtered_velocity_ - step_after_treshold_ :
                                                                last_sent_filtered_velocity_ + step_after_treshold_);
  }
  else if (isBelowTreshold())
  {
    current_filtered_velocity_ =
        (last_sent_filtered_velocity_ > current_raw_velocity_ ? last_sent_filtered_velocity_ - step_after_treshold_ :
                                                                last_sent_filtered_velocity_ + step_after_treshold_);
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
