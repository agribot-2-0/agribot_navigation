/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#pragma once
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
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
  virtual bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                          const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& end_pose) = 0;
  virtual void addObstacle(const ObstacleDetection::Obstacle& obstacle) = 0;
  virtual std::vector<ObstacleDetection::Obstacle> getObstacles(void) = 0;
  virtual void clearObstacles(void) = 0;

  void setFrameId(const std::string& frame);

protected:
  std::string frame_id;
};
}  // namespace agribot_local_planner
