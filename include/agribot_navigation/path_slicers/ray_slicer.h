/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <vector>

#include "agribot_navigation/path_slicers/i_path_slicer.h"
#include "agribot_navigation/planners/structures.h"
#include "agribot_navigation/planners/default_values.h"
#include "agribot_navigation/planners/common_functions.h"

namespace agribot_path_slicers
{
class RaySlicer : public IPathSlicer
{
public:
  RaySlicer();
  ~RaySlicer();
  bool findPathSlice(std::vector<geometry_msgs::PoseStamped>& path_slice,  // NOLINT(runtime/references)
                     const std::vector<geometry_msgs::PoseStamped>& local_path,
                     const geometry_msgs::PoseStamped& robot_pose) override;
};
}  // namespace agribot_path_slicers
