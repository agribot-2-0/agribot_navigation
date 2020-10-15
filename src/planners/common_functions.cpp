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
}  // namespace agribot_local_planner
