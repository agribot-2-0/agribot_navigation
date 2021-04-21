/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/obstacle_detection/obstacle.h"

#include "agribot_navigation/common/common_functions.h"

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
    // append vertices and edges length
    for (auto& point : polygon.polygon.points)
    {
      Eigen::Vector2d vertex(point.x, point.y);

      //   append vertex
      addVertex(vertex);
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
  return centroid;
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

std::vector<geometry_msgs::PoseStamped> Obstacle::toPath() const
{
  double step_size = 0.5;
  std::vector<geometry_msgs::PoseStamped> line;

  geometry_msgs::PoseStamped pose_start;
  pose_start.header.frame_id = frame_id_;
  pose_start.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped pose_end;
  pose_end.header.frame_id = frame_id_;
  pose_end.header.stamp = ros::Time::now();
  if (abs(min_y_.y()) < abs(max_y_.y()))
  {
    pose_start.pose.position.x = min_x_.x();
    pose_start.pose.position.y = min_y_.y();

    pose_end.pose.position.x = max_x_.x();
    pose_end.pose.position.y = min_y_.y();

    line.push_back(pose_start);
    straightPath(line, pose_start, pose_end, step_size);
    line.push_back(pose_end);
  }
  else
  {
    pose_start.pose.position.x = min_x_.x();
    pose_start.pose.position.y = max_y_.y();

    pose_end.pose.position.x = max_x_.x();
    pose_end.pose.position.y = max_y_.y();

    line.push_back(pose_start);
    straightPath(line, pose_start, pose_end, step_size);
    line.push_back(pose_start);
  }

  return line;
}

bool Obstacle::straightPath(std::vector<geometry_msgs::PoseStamped>& path,
                            const geometry_msgs::PoseStamped& start_point, const geometry_msgs::PoseStamped& end_point,
                            const double step_size) const
{
  int steps = CommonFunctions::distanceBetweenPoses(start_point, end_point) / step_size;

  if (steps <= 0)
  {
    ROS_ERROR_STREAM("[Obstacle] Could not assemble straight path");
    return false;
  }

  for (int i = 0; i < steps; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id_;
    pose.header.seq = path.size() + 1;
    pose.pose.position.x = start_point.pose.position.x +
                           static_cast<double>(i) / steps * (end_point.pose.position.x - start_point.pose.position.x);
    pose.pose.position.y = start_point.pose.position.y +
                           static_cast<double>(i) / steps * (end_point.pose.position.y - start_point.pose.position.y);
    path.push_back(pose);
  }
  return true;
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

void Obstacle::addVertex(Eigen::Vector2d vertex)
{
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

  //   append edge
  if (!vertices_.empty())
  {
    edges_.push_back(vertex - vertices_.back());
  }

  vertices_number_ += 1;
}

std::ostream& operator<<(std::ostream& out, const Obstacle& obstacle)
{
  out << "Vertices: " << obstacle.vertices_number_;
  for (auto const& v : obstacle.vertices_)
  {
    out << std::endl << "x: " << v.x() << "\t y: " << v.y();
  }
  return out;
}

Obstacle& Obstacle::operator+=(const Obstacle& obstacle)
{
  for (const auto& vertex : obstacle.getVertices())
  {
    addVertex(vertex);
  }

  edges_.push_back(vertices_.front() - vertices_.back());
  area_ = computeArea();
  perimeter_ = computePerimeter();
  centroid_ = findCentroid();

  return *this;
}

}  // namespace ObstacleDetection
