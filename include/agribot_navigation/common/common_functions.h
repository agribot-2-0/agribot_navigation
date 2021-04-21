/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <cmath>
#include <vector>

#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/structures.h"

namespace CommonFunctions
{
constexpr double cot(double alfa)
{
  return 1.0 / tan(alfa);
}
constexpr double toRadians(double degrees)
{
  return degrees * (M_PI / 180);
}
constexpr double toDegrees(double radians)
{
  return radians * (180 / M_PI);
}
double dGetYawFromQuaternion(const geometry_msgs::Quaternion& quat);
double limitAngle(double angle);
double dAngleBetweenCoordinates(const agribot_local_planner::pos& targetFrame,
                                const agribot_local_planner::pos& currentFrame);
double dDistanceBetweenCoordinates(const agribot_local_planner::pos& targetFrame,
                                   const agribot_local_planner::pos& currentFrame);
double angleBetweenPoses(const geometry_msgs::PoseStamped& target_frame,
                         const geometry_msgs::PoseStamped& current_frame);
double distanceBetweenPoses(const geometry_msgs::PoseStamped& target_frame,
                            const geometry_msgs::PoseStamped& current_frame);
agribot_local_planner::pos pCoordinatesFromPoseStamped(const geometry_msgs::PoseStamped& ps);
void updateOrientationInPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                             const int start_index, const int end_index);
agribot_local_planner::pos pCalculateProjection(const agribot_local_planner::pos& lastWaypoint,
                                                const agribot_local_planner::pos& nextWaypoint,
                                                const agribot_local_planner::pos& robotCenter);
double dCalculateRadiusFromVelocity(const geometry_msgs::Twist& velocity);

}  // namespace CommonFunctions
