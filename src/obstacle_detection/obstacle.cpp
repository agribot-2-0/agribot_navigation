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

Obstacle::Obstacle(const geometry_msgs::PolygonStamped& polygon)
{
  initializeEmpty();
  if (polygon.polygon.points.size() > 2)
  {
    frame_id_ = polygon.header.frame_id;
    // assign number of vertices
    vertices_number_ = polygon.polygon.points.size();
    // append vertices and edges length
    for (auto& point : polygon.polygon.points)
    {
      Eigen::Vector2d vertex(point.x, point.y);
      //   append edge
      if (!vertices_.empty())
      {
        edges_.push_back(vertex - vertices_.back());
      }
      //   append vertex
      vertices_.push_back(vertex);
      //  update extremal vertices
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

double Obstacle::computeArea() const
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

double Obstacle::computePerimeter() const
{
  double perimeter = 0.0;
  for (auto& edge : edges_)
  {
    perimeter += edge.norm();
  }
  return perimeter;
}

Eigen::Vector2d Obstacle::findCentroid() const
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

geometry_msgs::PolygonStamped Obstacle::toRectangle() const
{
  geometry_msgs::PolygonStamped rectangle;
  rectangle.header.frame_id = frame_id_;
  rectangle.header.stamp = ros::Time::now();
  geometry_msgs::Point32 point;

  point.x = min_x_.x();
  point.y = min_y_.y();
  rectangle.polygon.points.push_back(point);
  point.x = min_x_.x();
  point.y = max_y_.y();
  rectangle.polygon.points.push_back(point);
  point.x = max_x_.x();
  point.y = max_y_.y();
  rectangle.polygon.points.push_back(point);
  point.x = max_x_.x();
  point.y = min_y_.y();
  rectangle.polygon.points.push_back(point);

  return rectangle;
}

geometry_msgs::PolygonStamped Obstacle::toPolygon() const
{
  geometry_msgs::PolygonStamped polygon;
  polygon.header.frame_id = frame_id_;
  polygon.header.stamp = ros::Time::now();
  geometry_msgs::Point32 point;
  for (auto const& vertex : vertices_)
  {
    point.x = vertex.x();
    point.y = vertex.y();
    polygon.polygon.points.push_back(point);
  }
  return polygon;
}

Eigen::Vector2d Obstacle::getCentroid() const
{
  return centroid_;
}
Eigen::Vector2d Obstacle::getMinX() const
{
  return min_x_;
}
Eigen::Vector2d Obstacle::getMaxX() const
{
  return max_x_;
}
Eigen::Vector2d Obstacle::getMinY() const
{
  return min_y_;
}
Eigen::Vector2d Obstacle::getMaxY() const
{
  return max_y_;
}
std::size_t Obstacle::getVerticesNumber() const
{
  return vertices_number_;
}
std::vector<Eigen::Vector2d> Obstacle::getVertices() const
{
  return vertices_;
}
std::vector<Eigen::Vector2d> Obstacle::getEdges() const
{
  return edges_;
}
std::vector<double> Obstacle::getEdgesLength() const
{
  return edges_length_;
}

std::string Obstacle::getFrameID() const
{
  return frame_id_;
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
