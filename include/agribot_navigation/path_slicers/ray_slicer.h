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
  void setRayLength(const double length);
  void setRayWidth(const double width);
  void reset();

private:
  bool checkWaypoint(const geometry_msgs::PoseStamped& robot_pose, const geometry_msgs::PoseStamped& waypoint_pose) const;
  bool withinRayDistance(const agribot_local_planner::pos& robot_center,
                         const agribot_local_planner::pos& waypoint) const;
  bool withinRayAngle(const agribot_local_planner::pos& robot_center, const agribot_local_planner::pos& waypoint) const;
  bool calculateLinesSpan(const double target_line_angle, const double base_line_angle) const;
  double ray_length_;  // metres
  double ray_width_;   // degrees
  std::size_t search_start_index_;
};
}  // namespace agribot_path_slicers
