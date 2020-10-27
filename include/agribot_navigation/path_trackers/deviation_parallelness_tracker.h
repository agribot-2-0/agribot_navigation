/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <string>
#include <vector>

#include "agribot_navigation/path_trackers/i_path_tracker.h"
#include "agribot_navigation/planners/structures.h"
#include "agribot_navigation/planners/default_values.h"
#include "agribot_navigation/planners/common_functions.h"

namespace agribot_path_trackers
{
class DeviationParallelnessTracker : public IPathTracker
{
public:
  DeviationParallelnessTracker();
  ~DeviationParallelnessTracker();
  bool computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                      const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                      const geometry_msgs::PoseStamped& robot_pose) override;

private:
  double dCloseToZero(double value);
  void xSetVel(const agribot_local_planner::pos& next_waypoint, const agribot_local_planner::pos& last_waypoint,
               const agribot_local_planner::pos& robot_center,
               geometry_msgs::Twist& velocity);             // NOLINT(runtime/references)
  void xLimitVelocities(geometry_msgs::Twist& velocities);  // NOLINT(runtime/references)
  void xDeviationApproach(const agribot_local_planner::pos& next_waypoint,
                          const agribot_local_planner::pos& last_waypoint,
                          const agribot_local_planner::pos& robot_center,
                          geometry_msgs::Twist& velocity);  // NOLINT(runtime/references)
  void xLinearSpeedLimitedApproach(const agribot_local_planner::pos& next_waypoint,
                                   const agribot_local_planner::pos& last_waypoint,
                                   const agribot_local_planner::pos& robot_center,
                                   geometry_msgs::Twist& velocity);  // NOLINT(runtime/references)
  void xStopRobot(geometry_msgs::Twist& velocity);                   // NOLINT(runtime/references)
  double dCalculateDeviation(const agribot_local_planner::pos& last_waypoint,
                             const agribot_local_planner::pos& next_waypoint,
                             const agribot_local_planner::pos& robot_center);
  double dCalculateParallelness(const agribot_local_planner::pos& last_waypoint,
                                const agribot_local_planner::pos& next_waypoint,
                                const agribot_local_planner::pos& robot_center);

  void setSpeedLimiting(bool limiting);
  void setParallelnessWeight(double parallelness_weight);
  void setDeviationWeight(double deviation_weight);
  void setMinRadius(double min_radius);
  void setMaxLinearVelocity(double max_velocity);
  void setMinLinearVelocity(double min_velocity);
  void setEpsilon(double epsilon);

  double param_epsilon;                      // unitless
  double param_max_linear_vel;               // m/s
  double param_min_linear_vel;               // m/s
  double param_min_radius;                   // metres
  double param_current_deviation_weight;     // factor
  double param_current_parallelness_weight;  // factor
  bool param_speed_limit;
};
}  // namespace agribot_path_trackers
