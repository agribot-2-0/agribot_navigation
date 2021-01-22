/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/obstacle_detection/detection.h"

namespace ObstacleDetection
{
Detection::Detection()
{
  minimal_obstacle_length_ = 0.0;
  minimal_longitudinal_distance_to_obstacle_ = 0.0;
  minimal_lateral_distance_to_obstacle_ = 0.0;
  colison_width_ = 1.0;
  collision_distance_ = 2.0;
}

bool Detection::checkInterRow(const std::vector<Obstacle>& obstacles) const
{
  bool row_on_the_left = false;
  bool row_on_the_right = false;
  for (const auto& obstacle : obstacles)
  {
    if (isRowObstacle(obstacle))
    {
        ROS_ERROR_STREAM("[DEBUG]\tRow obstacle");
      if (obstacleOnTheLeft(obstacle, minimal_lateral_distance_to_obstacle_))
      {
        row_on_the_left = true;
      }
      if (obstacleOnTheRight(obstacle, minimal_lateral_distance_to_obstacle_))
      {
        row_on_the_right = true;
      }
    }
  }
  ROS_ERROR_STREAM("[DEBUG]\tleft row " << row_on_the_left << " right row " << row_on_the_right);
  return row_on_the_left && row_on_the_right;
}

bool Detection::checkColidingObstacle(const std::vector<Obstacle>& obstacles) const
{
  for (const auto& obstacle : obstacles)
  {
    if (isColiding(obstacle))
    {
      return true;
    }
  }
  return false;
}

bool Detection::isRowObstacle(const Obstacle& obstacle) const
{
  return checkObstacleLength(obstacle, minimal_obstacle_length_) &&
         checkLongitudinalDistanceToObstacle(obstacle, minimal_longitudinal_distance_to_obstacle_) && isAhead(obstacle);
}

bool Detection::isColiding(const Obstacle& obstacle) const
{
  checkLongitudinalDistanceToObstacle(obstacle, collision_distance_) && isInFront(obstacle);
}

bool Detection::checkObstacleLength(const Obstacle& obstacle, const double minimal_length) const
{
  return obstacle.getMaxX().x() - obstacle.getMinX().x() >= minimal_length;
}

bool Detection::checkLongitudinalDistanceToObstacle(const Obstacle& obstacle, const double minimal_distance) const
{
  return obstacle.getMinX().x() <= minimal_distance;
}

bool Detection::isAhead(const Obstacle& obstacle) const
{
  return obstacle.getMaxX().x() > 0.0;
}

bool Detection::obstacleOnTheLeft(const Obstacle& obstacle, const double lateral_distance) const
{
  return obstacle.getMaxY().y() < -lateral_distance;
}

bool Detection::obstacleOnTheRight(const Obstacle& obstacle, const double lateral_distance) const
{
  return obstacle.getMinY().y() > lateral_distance;
}

bool Detection::isInFront(const Obstacle& obstacle) const
{
  return (obstacle.getMinY().y() > -colison_width_ / 2.0 && obstacle.getMinY().y() < colison_width_ / 2.0) ||
         (obstacle.getMaxY().y() > -colison_width_ / 2.0 && obstacle.getMaxY().y() < colison_width_ / 2.0);
}

}  // namespace ObstacleDetection
