/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include <agribot_navigation/velocity_smoother/velocity_smoother.hh>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_smoother_node");
  ros::NodeHandle nh;
  VelocitySmoother vel_smooth = VelocitySmoother(&nh);
  if (vel_smooth.getParameters() == 0)
  {
    return 0;
  }
  vel_smooth.run();
}
