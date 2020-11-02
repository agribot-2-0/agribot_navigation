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
inline double toRadians(double degrees)
{
  return degrees * (M_PI / 180);
}
inline double toDegrees(double radians)
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
agribot_local_planner::pos pCalculateProjection(const agribot_local_planner::pos& lastWaypoint,
                                                const agribot_local_planner::pos& nextWaypoint,
                                                const agribot_local_planner::pos& robotCenter);

}  // namespace CommonFunctions
