/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>
#include <string>

#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/common/common_functions.h"
#include "agribot_navigation/common/default_values.h"

namespace agribot_local_planner
{
class ILocalPlanner
{
public:
  virtual bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                          const geometry_msgs::PoseStamped& start_pose,
                          const geometry_msgs::PoseStamped& end_pose) const = 0;
  void setFrameId(const std::string& frame);

protected:
  std::string frame_id;
};
}  // namespace agribot_local_planner
