/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <costmap_2d/costmap_2d_ros.h>
#pragma GCC diagnostic pop
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

#include <cmath>
#include <string>
#include <vector>

#include "agribot_navigation/common/common_functions.h"
#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/obstacle_detection/obstacle.h"

namespace agribot_local_planner
{
class ILocalPlanner
{
public:
  virtual ~ILocalPlanner() = default;
  virtual bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                          const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& end_pose) = 0;
  virtual void addObstacle(const ObstacleDetection::Obstacle& obstacle) = 0;
  virtual std::vector<ObstacleDetection::Obstacle> getObstacles(void) = 0;
  virtual void clearObstacles(void) = 0;

  virtual void setLineLeft(const nav_msgs::Path& line) = 0;
  virtual void setLineRight(const nav_msgs::Path& line) = 0;
};
}  // namespace agribot_local_planner
