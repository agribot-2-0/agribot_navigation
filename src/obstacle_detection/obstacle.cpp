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
      //   append edge
      if (!vertices_.empty())
      {
        edges_.push_back(vertex - vertices_.back());
      }
      //   append vertex
      vertices_.push_back(vertex);
      if (vertex.x() < min_x_.x())
        min_x_ = vertex;
      if (vertex.x() > max_x_.x())
        max_x_ = vertex;
      if (vertex.y() < min_y_.y())
        min_y_ = vertex;
      if (vertex.y() > max_y_.y())
        max_y_ = vertex;
    }
    // append last edge
    edges_.push_back(vertices_.front() - vertices_.back());
    area_ = computeArea();
    perimeter_ = computePerimeter();
    centroid_ = findCentroid();
  }
}

double Obstacle::computeArea()
{
  // computed from shoelace formula
  double area = 0.0;
  for (std::size_t i = 0; i < vertices_number_; i++)
  {
    area += vertices_[i].x() * vertices_[(i + 1) % vertices_number_].y() -
            vertices_[i].y() * vertices_[(i + 1) % vertices_number_].x();
  }
  area /= 2.0;
  return area;
}

double Obstacle::computePerimeter()
{
  double perimeter = 0.0;
  for (auto& edge : edges_)
  {
    perimeter += edge.norm();
  }
  return perimeter;
}

Eigen::Vector2d Obstacle::findCentroid()
{
  Eigen::Vector2d centroid;
  for (std::size_t i = 0; i < vertices_number_; i++)
  {
    Eigen::Vector2d V_n = vertices_[i];
    Eigen::Vector2d V_n1 = vertices_[(i + 1) % vertices_number_];
    double w = V_n.x() * V_n1.y() - V_n.y() * V_n1.x();
    centroid += (V_n + V_n1) * w;
  }
  centroid /= (area_ * 6.0);
}

void Obstacle::initializeEmpty()
{
  vertices_number_ = 0;
  vertices_.clear();
  edges_.clear();
  max_y_.y() = -INFINITY;
  min_y_.y() = INFINITY;
  max_x_.x() = -INFINITY;
  min_x_.x() = INFINITY;
  centroid_.x() = NAN;
  centroid_.y() = NAN;
  area_ = 0.0;
  perimeter_ = 0.0;
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
