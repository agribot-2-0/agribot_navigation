/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>

#include "agribot_navigation/planners/structures.h"
#include "agribot_navigation/planners/default_values.h"

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
double dLimitAngle(double angle);
double dAngleBetweenCoordinates(const agribot_local_planner::pos& targetFrame,
                                const agribot_local_planner::pos& currentFrame);
double dDistanceBetweenCoordinates(const agribot_local_planner::pos& targetFrame,
                                   const agribot_local_planner::pos& currentFrame);
agribot_local_planner::pos pCoordinatesFromPoseStamped(const geometry_msgs::PoseStamped& ps);
void updateOrientationInPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                             const int start_index, const int end_index);

}  // namespace CommonFunctions
