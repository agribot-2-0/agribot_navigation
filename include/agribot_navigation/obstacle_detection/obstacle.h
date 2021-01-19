/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <Eigen/Geometry>

namespace ObstacleDetection
{
class Obstacle
{
public:
  Obstacle();
  Obstacle(const geometry_msgs::Polygon& polygon);
  friend std::ostream & operator<<(std::ostream & out, const Obstacle & o);

private:
  std::size_t vertices_number_;
  std::vector<Eigen::Vector2d> vertices_;
  std::vector<Eigen::Vector2d> edges_;
  std::vector<double> edges_length_;
  Eigen::Vector2d centroid_;
  Eigen::Vector2d min_x_;
  Eigen::Vector2d max_x_;
  Eigen::Vector2d min_y_;
  Eigen::Vector2d max_y_;
  double area_;
  double perimeter_;

  void initializeEmpty();
  double computeArea();
  double computePerimeter();
  Eigen::Vector2d findCentroid();
};
}  // namespace ObstacleDetection
