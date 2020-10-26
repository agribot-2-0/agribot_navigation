/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <cmath>
#include <string>
#include <vector>

namespace agribot_path_trackers
{
class IPathTracker
{
public:
  virtual bool computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                              const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                              const geometry_msgs::PoseStamped& robot_pose) = 0;
};
}  // namespace agribot_path_trackers
