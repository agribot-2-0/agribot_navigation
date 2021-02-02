/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>

#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace ObstacleDetection
{
class Obstacle
{
public:
  Obstacle();
  explicit Obstacle(const geometry_msgs::PolygonStamped& polygon);
  geometry_msgs::PolygonStamped toRectangle() const;
  geometry_msgs::PolygonStamped toPolygon() const;
  friend std::ostream& operator<<(std::ostream& out, const Obstacle& o);
  Eigen::Vector2d getCentroid() const;
  Eigen::Vector2d getMinX() const;
  Eigen::Vector2d getMaxX() const;
  Eigen::Vector2d getMinY() const;
  Eigen::Vector2d getMaxY() const;
  std::size_t getVerticesNumber() const;
  std::vector<Eigen::Vector2d> getVertices() const;
  std::vector<Eigen::Vector2d> getEdges() const;
  std::vector<double> getEdgesLength() const;
  std::string getFrameID() const;

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
  std::string frame_id_;

  void initializeEmpty();
  double computeArea() const;
  double computePerimeter() const;
  Eigen::Vector2d findCentroid() const;
};
}  // namespace ObstacleDetection
