/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <string>
#include <vector>

#include "agribot_navigation/path_trackers/i_path_tracker.h"

namespace agribot_path_trackers
{
class StanleyTracker : public IPathTracker
{
public:
  StanleyTracker();
  ~StanleyTracker();
  bool computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                      const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                      const geometry_msgs::PoseStamped& robot_pose) override;
};
}  // namespace agribot_path_trackers
