/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/common_functions.h"

namespace CommonFunctions
{
double dGetYawFromQuaternion(const geometry_msgs::Quaternion& quat)
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
double dLimitAngle(double angle)
{
  if (angle > toRadians(180))
  {
    angle -= toRadians(360);
  }
  if (angle < toRadians(-180))
  {
    angle += toRadians(360);
  }
  return angle;
}

double dAngleBetweenCoordinates(const agribot_local_planner::pos& targetFrame,
                                const agribot_local_planner::pos& currentFrame)
{
  return std::atan2(targetFrame.y - currentFrame.y, targetFrame.x - currentFrame.x);
}

double dDistanceBetweenCoordinates(const agribot_local_planner::pos& targetFrame,
                                   const agribot_local_planner::pos& currentFrame)
{
  return std::sqrt(std::pow(targetFrame.y - currentFrame.y, 2) + std::pow(targetFrame.x - currentFrame.x, 2));
}

agribot_local_planner::pos pCoordinatesFromPoseStamped(const geometry_msgs::PoseStamped& ps)
{
  agribot_local_planner::pos coordinates;
  coordinates.x = ps.pose.position.x;
  coordinates.y = ps.pose.position.y;
  coordinates.az = dGetYawFromQuaternion(ps.pose.orientation);
  return coordinates;
}

void updateOrientationInPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                             const int start_index, const int end_index)
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
}  // namespace CommonFunctions
