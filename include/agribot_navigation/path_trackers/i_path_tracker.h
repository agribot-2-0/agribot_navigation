/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>

#include <vector>

namespace agribot_path_trackers
{
class IPathTracker
{
public:
  virtual ~IPathTracker() = default;
  virtual bool findPathSlice(std::vector<geometry_msgs::PoseStamped>& path_slice,  // NOLINT(runtime/references)
                             const std::vector<geometry_msgs::PoseStamped>& local_path,
                             const geometry_msgs::PoseStamped& robot_pose) = 0;
  virtual void reset() = 0;
};
}  // namespace agribot_path_trackers
