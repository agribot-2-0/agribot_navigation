/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/common/common_functions.h"

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
  while (angle > toRadians(180))
  {
    angle -= toRadians(360);
  }
  while (angle < toRadians(-180))
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

double angleBetweenPoses(const geometry_msgs::PoseStamped& target_frame,
                         const geometry_msgs::PoseStamped& current_frame)
{
  return dAngleBetweenCoordinates(CommonFunctions::pCoordinatesFromPoseStamped(target_frame),
                                  CommonFunctions::pCoordinatesFromPoseStamped(current_frame));
}
double distanceBetweenPoses(const geometry_msgs::PoseStamped& target_frame,
                            const geometry_msgs::PoseStamped& current_frame)
{
  return dDistanceBetweenCoordinates(CommonFunctions::pCoordinatesFromPoseStamped(target_frame),
                                     CommonFunctions::pCoordinatesFromPoseStamped(current_frame));
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

agribot_local_planner::pos pCalculateProjection(const agribot_local_planner::pos& lastWaypoint,
                                                const agribot_local_planner::pos& nextWaypoint,
                                                const agribot_local_planner::pos& robotCenter)
{
  agribot_local_planner::pos projection;
  double a = dDistanceBetweenCoordinates(nextWaypoint, lastWaypoint);
  double b = dDistanceBetweenCoordinates(lastWaypoint, robotCenter);
  double c = dDistanceBetweenCoordinates(nextWaypoint, robotCenter);
  double dx = nextWaypoint.x - lastWaypoint.x;
  double dy = nextWaypoint.y - lastWaypoint.y;
  // taken from cosine law
  // cosine = (a*a + b*b - c*c) / (2*a*b)
  // a1 = b * cosine
  // ratio = a1 / a
  double ratio = (a * a + b * b - c * c) / (2 * a * a);
  projection.x = lastWaypoint.x + ratio * dx;
  projection.y = lastWaypoint.y + ratio * dy;
  projection.az = dAngleBetweenCoordinates(nextWaypoint, lastWaypoint);
  return projection;
}

double dCalculateRadiusFromVelocity(const geometry_msgs::Twist& velocity)
{
  if (velocity.angular.z != 0)
  {
    return velocity.linear.x / velocity.angular.z;
  }
  return 0.0;
}
}  // namespace CommonFunctions
