/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>

#include "agribot_navigation/planners/structures.h"

constexpr double deg_to_rad = (M_PI / 180);
constexpr double rad_to_deg = (180 / M_PI);

namespace agribot_local_planner
{
class CommonFunctions
{
public:
  CommonFunctions();
  ~CommonFunctions();
  double dGetYawFromQuaternion(const geometry_msgs::Quaternion& quat) const;
  double dLimitAngle(double angle) const;
  double dAngleBetweenFrames(const pos& targetFrame, const pos& currentFrame) const;
  double dDistanceBetweenFrames(const pos& targetFrame, const pos& currentFrame) const;
  pos pFrameFromPoseStamped(const geometry_msgs::PoseStamped& ps) const;
};
}  // namespace agribot_local_planner
