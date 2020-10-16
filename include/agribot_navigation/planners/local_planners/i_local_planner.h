/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>
#include <string>

#include "agribot_navigation/planners/structures.h"
#include "agribot_navigation/planners/common_functions.h"
#include "agribot_navigation/planners/default_values.h"

namespace agribot_local_planner
{
class ILocalPlanner
{
public:
  ILocalPlanner();
  virtual bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                          const geometry_msgs::PoseStamped& startPose,
                          const geometry_msgs::PoseStamped& endPose) const = 0;
  void setHeader(const std::string frame);

protected:
  std::string frame_id;
  CommonFunctions functions;
  void updateOrientation(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                         const int start_index, const int end_index) const;
};
}  // namespace agribot_local_planner
