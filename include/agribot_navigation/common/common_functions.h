/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#pragma once
#include <agribot_common/CommonFunctions/common_functions.h>
#include <agribot_msgs/AgribotManualControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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

double getYawFromQuaternion(const geometry_msgs::Quaternion& quat);
geometry_msgs::Quaternion getQuaternionFromYaw(const double yaw);

double limitAngle(double angle);
double limitAngle(double angle, double range);
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
                             const std::size_t start_index, const std::size_t end_index);
agribot_local_planner::pos pCalculateProjection(const agribot_local_planner::pos& lastWaypoint,
                                                const agribot_local_planner::pos& nextWaypoint,
                                                const agribot_local_planner::pos& robotCenter);
geometry_msgs::PoseStamped calculateProjection(const geometry_msgs::PoseStamped& lastWaypoint,
                                               const geometry_msgs::PoseStamped& nextWaypoint,
                                               const geometry_msgs::PoseStamped& robotCenter);
double dCalculateRadiusFromVelocity(const geometry_msgs::Twist& velocity);
void straightPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                  const geometry_msgs::PoseStamped& start_point, const geometry_msgs::PoseStamped& end_point,
                  const std::string& frame_id, const double step_size);

double calculateCrosstrackError(const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                                const geometry_msgs::PoseStamped& robot_pose);

geometry_msgs::Quaternion orientationBetweenQuaternions(const geometry_msgs::Quaternion& quaternion1,
                                                        const geometry_msgs::Quaternion& quaternion2);
geometry_msgs::Twist teleopToVel(const agribot_msgs::AgribotManualControl& in_msg, const double min_motion_radius,
                                 const double cutoff_radius);
geometry_msgs::Twist teleopToVel(const agribot_msgs::AgribotManualControl& in_msg, const double min_motion_radius);
}  // namespace CommonFunctions
