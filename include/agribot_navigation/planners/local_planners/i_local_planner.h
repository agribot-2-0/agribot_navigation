/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>

#include "agribot_navigation/planners/structures.h"

#define DEG_TO_RAD (M_PI / 180)
#define RAD_TO_DEG (180 / M_PI)

namespace agribot_local_planner
{
class ILocalPlaner
{
public:
  ILocalPlaner();
  ~ILocalPlaner();
  virtual bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                          const geometry_msgs::PoseStamped startPose, const geometry_msgs::PoseStamped endPose) = 0;

protected:
  mutable std_msgs::Header header;
  void updateOrientation(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                         const int start_index, const int end_index) const;
  double dGetYawFromQuaternion(const geometry_msgs::Quaternion quat) const;
  double dLimitAngle(double angle) const;
  double dAngleBetweenFrames(const pos targetFrame, const pos currentFrame) const;
  double dDistanceBetweenFrames(const pos targetFrame, const pos currentFrame) const;
  pos pFrameFromPoseStamped(const geometry_msgs::PoseStamped ps) const;
};
}  // namespace agribot_local_planner
