/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wformat"
#include <costmap_converter/costmap_converter_interface.h>
#pragma GCC diagnostic pop

#include <string>
#include <vector>

#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/obstacle_detection/detection.h"
#include "agribot_navigation/planners/local_planners/i_local_planner.h"

namespace agribot_local_planner
{
class NarrowCorridorPlanner : public ILocalPlanner
{
public:
  NarrowCorridorPlanner() = delete;
  explicit NarrowCorridorPlanner(const std::string& frame);

  bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                  const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& end_pose) override;
  void addObstacle(const ObstacleDetection::Obstacle& obstacle) override;
  std::vector<ObstacleDetection::Obstacle> getObstacles(void) override;
  void clearObstacles(void);
  void setChassisLength(double chassis_length);
  void setStepSize(const double param_step_size);

private:
  std::string frame_id;
  ObstacleDetection::Obstacle obstacles_left_;
  ObstacleDetection::Obstacle obstacles_right_;
  std::vector<ObstacleDetection::Obstacle> obstacles_;
  ObstacleDetection::Detection detection_;
  double chassis_length_ = 0;
  double step_size_ = 0.1;

  geometry_msgs::PoseStamped calculateMidpose(const geometry_msgs::PoseStamped& point1,
                                              const geometry_msgs::PoseStamped& point2);
};
}  // namespace agribot_local_planner
