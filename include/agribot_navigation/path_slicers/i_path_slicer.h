/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <cmath>
#include <string>
#include <vector>

#include "agribot_navigation/planners/common_functions.h"

namespace agribot_path_slicers
{
class IPathSlicer
{
public:
  virtual bool findPathSlice(std::vector<geometry_msgs::PoseStamped>& path_slice,  // NOLINT(runtime/references)
                             const std::vector<geometry_msgs::PoseStamped>& local_path,
                             const geometry_msgs::PoseStamped& robot_pose) = 0;
};
}  // namespace agribot_path_slicers
