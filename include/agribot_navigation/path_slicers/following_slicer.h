/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <vector>

#include "agribot_navigation/path_slicers/i_path_slicer.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/common_functions.h"

namespace agribot_path_slicers
{
class FollowingSlicer : public IPathSlicer
{
public:
  FollowingSlicer();
  ~FollowingSlicer();
  bool findPathSlice(std::vector<geometry_msgs::PoseStamped>& path_slice,  // NOLINT(runtime/references)
                     const std::vector<geometry_msgs::PoseStamped>& local_path,
                     const geometry_msgs::PoseStamped& robot_pose) override;
  void reset();
  void setMissedDistanceCoefficient(const double coefficient);
  void setGoalDistanceTolerance(const double tolerance);

private:
  void takeNextGoal();
  bool isLastWaypointInPath(std::size_t waypoints_count, std::size_t path_length);
  bool waypointHitCondition(const agribot_local_planner::pos& robot_center,
                             const agribot_local_planner::pos& last_waypoint,
                             const agribot_local_planner::pos& next_waypoint);
  bool waypointMissedCondition(const agribot_local_planner::pos& robot_center,
                                const agribot_local_planner::pos& last_waypoint,
                                const agribot_local_planner::pos& next_waypoint);
  bool traveledDistanceBetweenWaypoints(const agribot_local_planner::pos& robot_center,
                                         const agribot_local_planner::pos& last_waypoint,
                                         const agribot_local_planner::pos& next_waypoint);
  bool withinGoalRange(const agribot_local_planner::pos& robot_center,
                        const agribot_local_planner::pos& next_waypoint);
  bool centerTraveledBeyondGoal(const agribot_local_planner::pos& robot_center,
                                 const agribot_local_planner::pos& last_waypoint,
                                 const agribot_local_planner::pos& next_waypoint);
  bool centerOutOfGoalRange(const agribot_local_planner::pos& robot_center,
                             const agribot_local_planner::pos& next_waypoint);
  double calculateTraveledDistance(const agribot_local_planner::pos& current_position,
                                   const agribot_local_planner::pos& last_position);
  agribot_local_planner::pos last_robot_center_;
  std::size_t waypoint_count_;
  double param_missed_distance_coefficient_;
  double param_goal_distance_tolerance_;
  double last_traveled_distance_;
  double traveled_distance_;
  bool is_last_robot_center_initialized_;
};
}  // namespace agribot_path_slicers
