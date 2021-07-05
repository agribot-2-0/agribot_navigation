/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <vector>

#include "agribot_navigation/path_trackers/i_path_tracker.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/common_functions.h"

namespace agribot_path_trackers
{
class RayTracker : public IPathTracker
{
public:
  RayTracker();
  bool findPathSlice(std::vector<geometry_msgs::PoseStamped>& path_slice,  // NOLINT(runtime/references)
                     const std::vector<geometry_msgs::PoseStamped>& local_path,
                     const geometry_msgs::PoseStamped& robot_pose) override;
  void setRayLength(const double length);
  void setRayWidth(const double width);
  void setTrackingMethod(const bool side_tracking);
  void reset() override;

private:
  bool checkWaypoint(const agribot_local_planner::pos& robot_coordinates,
                     const agribot_local_planner::pos& waypoint_coordinates) const;
  bool withinRayDistance(const agribot_local_planner::pos& robot_center,
                         const agribot_local_planner::pos& waypoint) const;
  bool withinSideRayAngle(const agribot_local_planner::pos& robot_center,
                          const agribot_local_planner::pos& waypoint) const;
  bool withinFrontRayAngle(const agribot_local_planner::pos& robot_center,
                           const agribot_local_planner::pos& waypoint) const;
  double calculateLinesSpan(const double target_line_angle, const double base_line_angle) const;
  double ray_length_;  // metres
  double ray_width_;   // degrees
  bool side_tracking_;
};
}  // namespace agribot_path_trackers
