/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <ros/ros.h>
#include <agribot_navigation/velocity_smoother/velocity_smoother.hh>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_smoother_node");
  ros::NodeHandle nh;
  VelocitySmoother vel_smooth = VelocitySmoother(&nh);
  if (vel_smooth.checkParameters() == 0)
  {
    return 0;
  }
  vel_smooth.run();
}
