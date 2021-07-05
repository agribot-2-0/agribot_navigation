/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <string>
#include <vector>

#include "agribot_navigation/common/common_functions.h"
#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/path_controllers/i_path_controller.h"

namespace agribot_path_controllers
{
class DeviationParallelnessController : public IPathController
{
public:
  DeviationParallelnessController();
  bool computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                      const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                      const geometry_msgs::PoseStamped& robot_pose) override;
  void reset() override{};
  void setSpeedLimiting(const bool limiting);
  void setParallelnessWeight(const double parallelness_weight);
  void setDeviationWeight(const double deviation_weight);
  void setMinRadius(const double min_radius);
  void setMaxLinearVelocity(const double max_velocity);
  void setMinLinearVelocity(const double min_velocity);
  void setEpsilon(const double epsilon);

private:
  double dCloseToZero(const double value);
  void xSetVel(geometry_msgs::Twist& velocity,  // NOLINT(runtime/references)
               const agribot_local_planner::pos& next_waypoint, const agribot_local_planner::pos& last_waypoint,
               const agribot_local_planner::pos& robot_center);
  void xLimitVelocities(geometry_msgs::Twist& velocities);  // NOLINT(runtime/references)
  void xDeviationApproach(geometry_msgs::Twist& velocity,   // NOLINT(runtime/references)
                          const agribot_local_planner::pos& next_waypoint,
                          const agribot_local_planner::pos& last_waypoint,
                          const agribot_local_planner::pos& robot_center);
  void xLinearSpeedLimitedApproach(geometry_msgs::Twist& velocity,  // NOLINT(runtime/references)
                                   const agribot_local_planner::pos& next_waypoint,
                                   const agribot_local_planner::pos& last_waypoint,
                                   const agribot_local_planner::pos& robot_center);
  void xStopRobot(geometry_msgs::Twist& velocity);  // NOLINT(runtime/references)
  double dCalculateDeviation(const agribot_local_planner::pos& last_waypoint,
                             const agribot_local_planner::pos& next_waypoint,
                             const agribot_local_planner::pos& robot_center);
  double dCalculateParallelness(const agribot_local_planner::pos& last_waypoint,
                                const agribot_local_planner::pos& next_waypoint,
                                const agribot_local_planner::pos& robot_center);

  double param_epsilon;                      // unitless
  double param_max_linear_vel;               // m/s
  double param_min_linear_vel;               // m/s
  double param_min_radius;                   // metres
  double param_current_deviation_weight;     // factor
  double param_current_parallelness_weight;  // factor
  bool param_speed_limiting;
};
}  // namespace agribot_path_controllers
