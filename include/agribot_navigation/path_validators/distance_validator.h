/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <vector>

#include "agribot_navigation/path_validators/i_path_validator.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/common_functions.h"
namespace agribot_path_validators
{
class DistanceValidator : public IPathValidator
{
public:
  DistanceValidator();
  ~DistanceValidator();
  bool validatePath(const std::vector<geometry_msgs::PoseStamped>& path) override;
  void setMinDistance(const double distance);

private:
  double min_distance_;
};
}  // namespace agribot_path_validators
