/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <string>
#include <vector>

#include "agribot_navigation/path_trackers/i_path_tracker.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/common_functions.h"

namespace agribot_path_trackers
{
class PurePursuitTracker : public IPathTracker
{
public:
  PurePursuitTracker();
  ~PurePursuitTracker();
  bool computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                      const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                      const geometry_msgs::PoseStamped& robot_pose) override;
  void setVelocityLinear(const double velocity);
  void setCurvature(const double curvature);
  void setRabbitDistance(const double distance);
  void resetRabbit();

private:
  void computeRabbit(const agribot_local_planner::pos& rabbit_pos, const agribot_local_planner::pos& robot_center,
                     const int path_size);
  double lin_vel_;
  double curvature_;
  double rabbit_distance_;
  uint rabbit_count_;
};
}  // namespace agribot_path_trackers
