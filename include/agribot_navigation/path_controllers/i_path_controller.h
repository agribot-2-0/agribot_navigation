/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <cmath>
#include <string>
#include <vector>

#include "agribot_navigation/common/common_functions.h"

namespace agribot_path_controllers
{
class IPathController
{
public:
  virtual ~IPathController() = default;
  virtual bool computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                              const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                              const geometry_msgs::PoseStamped& robot_pose) = 0;
  virtual void reset() = 0;
  virtual std::string getFrameId() = 0;
};
}  // namespace agribot_path_controllers
