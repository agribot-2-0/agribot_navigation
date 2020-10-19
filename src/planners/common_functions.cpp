/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/common_functions.h"

namespace agribot_local_planner
{
CommonFunctions::CommonFunctions()
{
}

CommonFunctions::~CommonFunctions()
{
}

double CommonFunctions::dGetYawFromQuaternion(const geometry_msgs::Quaternion& quat) const
{
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

/**
 * Limit angles to range <-180, 180> degrees
 */
double CommonFunctions::dLimitAngle(double angle) const
{
  if (angle > 180 * deg_to_rad)
  {
    angle -= 360 * deg_to_rad;
  }
  if (angle < -180 * deg_to_rad)
  {
    angle += 360 * deg_to_rad;
  }
  return angle;
}

double CommonFunctions::dAngleBetweenFrames(const pos& targetFrame, const pos& currentFrame) const
{
  return std::atan2(targetFrame.y - currentFrame.y, targetFrame.x - currentFrame.x);
}

double CommonFunctions::dDistanceBetweenFrames(const pos& targetFrame, const pos& currentFrame) const
{
  return std::sqrt(std::pow(targetFrame.y - currentFrame.y, 2) + std::pow(targetFrame.x - currentFrame.x, 2));
}

pos CommonFunctions::pFrameFromPoseStamped(const geometry_msgs::PoseStamped& ps) const
{
  pos frame;
  frame.x = ps.pose.position.x;
  frame.y = ps.pose.position.y;
  frame.az = dGetYawFromQuaternion(ps.pose.orientation);
  return frame;
}

void CommonFunctions::updateOrientation(std::vector<geometry_msgs::PoseStamped>& path, const int start_index,
                                      const int end_index) const
{
  // add proper orientation to each point
  for (int i = start_index; i < end_index; i++)
  {
    double angle = std::atan2(path[i + 1].pose.position.y - path[i].pose.position.y,
                              path[i + 1].pose.position.x - path[i].pose.position.x);
    path[i].pose.orientation.x = 0;
    path[i].pose.orientation.y = 0;
    path[i].pose.orientation.z = sin(angle / 2.0);
    path[i].pose.orientation.w = cos(angle / 2.0);
  }
}
}  // namespace agribot_local_planner
