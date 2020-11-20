/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>

#include <vector>

namespace agribot_path_validators
{
class IPathValidator
{
public:
  virtual bool
  validatePath(const std::vector<geometry_msgs::PoseStamped>& path) = 0;  // NOLINT(runtime/references)
};
}  // namespace agribot_path_validators
