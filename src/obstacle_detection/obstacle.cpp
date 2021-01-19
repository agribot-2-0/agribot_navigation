/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/obstacle_detection/obstacle.h"

namespace ObstacleDetection
{
Obstacle::Obstacle()
{
  initializeEmpty();
}

Obstacle::Obstacle(const geometry_msgs::Polygon& polygon)
{
  if (polygon.points.size() < 3)
  {
    initializeEmpty();
  }
  else
  {
    initializeEmpty();
    // assign number of vertices
    vertices_number_ = polygon.points.size();
    // append vertices and edges length
    for (auto& p : polygon.points)
    {
      Eigen::Vector2d vertex(p.x, p.y);
      //   append edge length
      if (!vertices_.empty())
      {
        edges_length_.push_back((vertex - vertices_.back()).norm());
      }
      //   append vertex
      vertices_.push_back(vertex);
      if(vertex.x() < min_x_.x()) min_x_ = vertex;
      if(vertex.x() > max_x_.x()) max_x_ = vertex;
      if(vertex.y() < min_y_.y()) min_y_ = vertex;
      if(vertex.y() > max_y_.y()) max_y_ = vertex;
    }
    // append last edge
    edges_length_.push_back((vertices_.front() - vertices_.back()).norm());
  }
}

void Obstacle::initializeEmpty()
{
  vertices_number_ = 0;
  vertices_.clear();
  edges_length_.clear();
  max_y_.y() = -INFINITY;
  min_y_.y() = INFINITY;
  max_x_.x() = -INFINITY;
  min_x_.x() = INFINITY;
}

std::ostream& operator<<(std::ostream& out, const Obstacle& o)
{
  out << "Vertices: " << o.vertices_number_;
  for (auto const& v : o.vertices_)
  {
      out << "\nx: " << v.x() << "\ty: " << v.y();
  }
  return out;
}

}  // namespace ObstacleDetection
