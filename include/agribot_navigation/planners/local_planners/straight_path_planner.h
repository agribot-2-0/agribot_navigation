/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <string>
#include <vector>

#include "agribot_navigation/planners/local_planners/i_local_planner.h"

namespace agribot_local_planner
{
class StraightPathPlanner : public ILocalPlanner
{
public:
  StraightPathPlanner() = delete;
  explicit StraightPathPlanner(const std::string& frame);

  bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                  const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& end_pose) override;
  void addObstacle(const ObstacleDetection::Obstacle& obstacle) override;
  std::vector<ObstacleDetection::Obstacle> getObstacles(void) override;
  void clearObstacles(void) override;
  void setStepSize(const double param_step_size);

private:
  std::string frame_id;
  double step_size_;
  std::vector<ObstacleDetection::Obstacle> obstacles_;
};
}  // namespace agribot_local_planner
