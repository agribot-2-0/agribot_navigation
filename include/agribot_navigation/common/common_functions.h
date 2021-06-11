/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <agribot_common/CommonFunctions/common_functions.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <cmath>
#include <string>
#include <vector>

#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/structures.h"

namespace CommonFunctions
{
constexpr double cot(double alfa)
{
  return 1.0 / tan(alfa);
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
void straightPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                  const geometry_msgs::PoseStamped& start_point, const geometry_msgs::PoseStamped& end_point,
                  const std::string& frame_id, const double step_size);

}  // namespace CommonFunctions
