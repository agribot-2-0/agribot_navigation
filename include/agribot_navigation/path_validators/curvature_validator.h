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
class CurvatureValidator : public IPathValidator
{
public:
  CurvatureValidator();
  ~CurvatureValidator();
  bool validatePath(const std::vector<geometry_msgs::PoseStamped>& path) override;
  void setMaxCurvature(const double curvature);
  void setSearchWidth(const std::size_t width);

private:
  bool calculateMengerCurvature(const geometry_msgs::PoseStamped& previous_waypoint,
                                const geometry_msgs::PoseStamped& current_waypoint,
                                const geometry_msgs::PoseStamped& next_waypoint) const;
  double max_curvature_;
  std::size_t search_width_;
};
}  // namespace agribot_path_validators
 // NOLINT(runtime/references)