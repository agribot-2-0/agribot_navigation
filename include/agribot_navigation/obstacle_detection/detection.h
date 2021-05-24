/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <agribot_navigation/obstacle_detection/obstacle.h>
#include <ros/ros.h>

#include <vector>

namespace ObstacleDetection
{
class Detection
{
public:
  Detection();
  bool isInRow(const std::vector<Obstacle>& obstacles) const;
  bool checkColidingObstacle(const std::vector<Obstacle>& obstacles) const;
  void setRobotPose(const geometry_msgs::PoseStamped& robot_pose);

private:
  bool isRowObstacle(const Obstacle& obstacle) const;
  bool isColiding(const Obstacle& obstacle) const;
  bool isAhead(const Obstacle& obstacle) const;
  bool isInFront(const Obstacle& obstacle) const;
  bool checkObstacleLength(const Obstacle& obstacle, const double minimal_length) const;
  bool checkLongitudinalDistanceToObstacle(const Obstacle& obstacle, const double minimal_distance) const;
  bool obstacleOnTheLeft(const Obstacle& obstacle, const double lateral_distance) const;
  bool obstacleOnTheRight(const Obstacle& obstacle, const double lateral_distance) const;

  double minimal_obstacle_length_;
  double minimal_longitudinal_distance_to_obstacle_;
  double minimal_lateral_distance_to_obstacle_;
  double collision_width_;
  double collision_distance_;
  geometry_msgs::PoseStamped robot_pose_;
};
}  // namespace ObstacleDetection
