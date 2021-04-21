/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#pragma once

#include <costmap_converter/costmap_converter_interface.h>

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
  NarrowCorridorPlanner();
  explicit NarrowCorridorPlanner(const std::string& frame);
  ~NarrowCorridorPlanner();
  bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                  const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& end_pose) override;
  void addObstacle(const ObstacleDetection::Obstacle& obstacle) override;
  std::vector<ObstacleDetection::Obstacle> getObstacles(void) override;
  void clearObstacles(void)
  {
    obstacles_left_.initializeEmpty();
    obstacles_right_.initializeEmpty();
    obstacles_.clear();
  }
  bool straightPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                    const geometry_msgs::PoseStamped& start_point, const geometry_msgs::PoseStamped& end_point,
                    const double step_size);

private:
  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;

  ObstacleDetection::Obstacle obstacles_left_;
  ObstacleDetection::Obstacle obstacles_right_;
  std::vector<ObstacleDetection::Obstacle> obstacles_;
  ObstacleDetection::Detection detection_;

  geometry_msgs::PoseStamped calculateMidpoint(const geometry_msgs::PoseStamped& point1,
                                               const geometry_msgs::PoseStamped& point2);
};
}  // namespace agribot_local_planner
