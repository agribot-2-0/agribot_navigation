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

namespace agribot_local_planner
{
class ILocalPlanner
{
public:
  virtual bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                          const geometry_msgs::PoseStamped& startPose,
                          const geometry_msgs::PoseStamped& endPose) const = 0;
  void setHeader(const std::string frame);

protected:
  std::string frame_id;
  void updateOrientation(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                         const int start_index, const int end_index) const;
  double dGetYawFromQuaternion(const geometry_msgs::Quaternion& quat) const;
  double dLimitAngle(double angle) const;
  double dAngleBetweenFrames(const pos& targetFrame, const pos& currentFrame) const;
  double dDistanceBetweenFrames(const pos& targetFrame, const pos& currentFrame) const;
  pos pFrameFromPoseStamped(const geometry_msgs::PoseStamped& ps) const;
};
}  // namespace agribot_local_planner
