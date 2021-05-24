/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/obstacle_detection/detection.h"

namespace ObstacleDetection
{
Detection::Detection()
{
  minimal_obstacle_length_ = 0.0;
  minimal_longitudinal_distance_to_obstacle_ = 5.0;
  minimal_lateral_distance_to_obstacle_ = 0.0;
  collision_width_ = 1.0;
  collision_distance_ = 4.0;
}

bool Detection::isInRow(const std::vector<Obstacle>& obstacles) const
{
  bool row_on_the_left = false;
  bool row_on_the_right = false;

  if (obstacles.size() == 0)
  {
    return false;
  }

  for (const auto& obstacle : obstacles)
  {
    if (isRowObstacle(obstacle))
    {
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
  return isAhead(obstacle) && checkObstacleLength(obstacle, minimal_obstacle_length_) &&
         checkLongitudinalDistanceToObstacle(obstacle, minimal_longitudinal_distance_to_obstacle_);
}

bool Detection::isColiding(const Obstacle& obstacle) const
{
  return isAhead(obstacle) && checkLongitudinalDistanceToObstacle(obstacle, collision_distance_) && isInFront(obstacle);
}

bool Detection::checkObstacleLength(const Obstacle& obstacle, const double minimal_length) const
{
  return (obstacle.getMaxX().x() - obstacle.getMinX().x()) >= minimal_length;
}

bool Detection::checkLongitudinalDistanceToObstacle(const Obstacle& obstacle, const double minimal_distance) const
{
  return obstacle.getMinX().x() <= minimal_distance;
}

bool Detection::isAhead(const Obstacle& obstacle) const
{
  return obstacle.getMaxX().x() > 0.0;
}

bool Detection::obstacleOnTheRight(const Obstacle& obstacle, const double lateral_distance) const
{
  return obstacle.getMaxY().y() < -lateral_distance;
}

bool Detection::obstacleOnTheLeft(const Obstacle& obstacle, const double lateral_distance) const
{
  return obstacle.getMinY().y() > lateral_distance;
}

bool Detection::isInFront(const Obstacle& obstacle) const
{
  return (obstacle.getMinY().y() > -collision_width_ / 2.0 && obstacle.getMinY().y() < collision_width_ / 2.0) ||
         (obstacle.getMaxY().y() > -collision_width_ / 2.0 && obstacle.getMaxY().y() < collision_width_ / 2.0);
}

void Detection::setRobotPose(const geometry_msgs::PoseStamped& robot_pose)
{
  robot_pose_ = robot_pose;
}

}  // namespace ObstacleDetection
