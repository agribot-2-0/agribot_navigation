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
  }

  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("[VelocitySmoother] " << ex.what());
    return 0;
  }
  return 1;
}

void VelocitySmoother::run()
{
  geometry_msgs::Twist filtered_msg;

  ros::Rate loop_rate(frequency_);
  while (ros::ok())
  {
    ros::spinOnce();

    if (current_raw_velocity_ == last_sent_filtered_velocity_)
    {
      current_filtered_velocity_ = last_sent_filtered_velocity_;
    }
    else
    {
      if (abs(last_sent_filtered_velocity_ - current_raw_velocity_) < acceleration_ / frequency_)
      {
        current_filtered_velocity_ = current_raw_velocity_;
      }
      else
      {
        current_filtered_velocity_ = (last_sent_filtered_velocity_ > current_raw_velocity_ ?
                                          last_sent_filtered_velocity_ - acceleration_ / frequency_ :
                                          last_sent_filtered_velocity_ + acceleration_ / frequency_);
      }
    }

    filtered_msg.linear.x = current_filtered_velocity_;
    smooth_velocity_publisher_.publish(filtered_msg);
    last_sent_filtered_velocity_ = current_filtered_velocity_;

    message_received_ = false;
    loop_rate.sleep();
  }
}

void VelocitySmoother::rawVelocityCallback(const geometry_msgs::Twist msg)
{
  message_received_ = true;
  current_raw_velocity_ = msg.linear.x;
}
