/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <agribot_navigation/velocity_smoother/velocity_smoother.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_smoother_node");
  ros::NodeHandle nh("~");
  VelocitySmoother vel_smooth = VelocitySmoother(&nh);
  try
  {
    vel_smooth.getParameters();
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM("[VelocitySmoother] " << ex.what());
    return 0;
  }
  vel_smooth.run();
}
