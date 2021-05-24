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
    for (const auto& point : polygon.polygon.points)
    {
      Eigen::Vector2d vertex(point.x, point.y);
      // append vertex
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

  point.x = min_y_.x();
  point.y = min_y_.y();
  rectangle.polygon.points.push_back(point);
  point.x = max_x_.x();
  point.y = max_x_.y();
  rectangle.polygon.points.push_back(point);
  point.x = max_y_.x();
  point.y = max_y_.y();
  rectangle.polygon.points.push_back(point);
  point.x = min_x_.x();
  point.y = min_x_.y();
  rectangle.polygon.points.push_back(point);

  return rectangle;
}

bool isXCoordinateSmaller(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2)
{
  return p1.x() < p2.x() || (p1.x() == p2.x() && p1.y() < p2.y());
}

geometry_msgs::PolygonStamped Obstacle::toPolygon() const
{
  geometry_msgs::PolygonStamped polygonStamped;
  polygonStamped.header.frame_id = frame_id_;
  polygonStamped.header.stamp = ros::Time::now();

  geometry_msgs::Polygon polygon;
  auto cluster{ vertices_ };

  int k = 0;
  int n = cluster.size();

  std::sort(cluster.begin(), cluster.end(), isXCoordinateSmaller);

  polygon.points.resize(2 * n);

  // lower hull
  for (int i = 0; i < n; ++i)
  {
    while (k >= 2 && cross(polygon.points[k - 2], polygon.points[k - 1], cluster[i]) <= 0)
    {
      --k;
    }
    geometry_msgs::Point32 point;
    point.x = cluster[i].x();
    point.y = cluster[i].y();
    polygon.points[k] = point;
    ++k;
  }

  // upper hull
  for (int i = n - 2, t = k + 1; i >= 0; --i)
  {
    while (k >= t && cross(polygon.points[k - 2], polygon.points[k - 1], cluster[i]) <= 0)
    {
      --k;
    }
    geometry_msgs::Point32 point;
    point.x = cluster[i].x();
    point.y = cluster[i].y();
    polygon.points[k] = point;
    ++k;
  }

  polygon.points.resize(k);  // original
  polygonStamped.polygon = polygon;
  return polygonStamped;
}

std::vector<geometry_msgs::PoseStamped> Obstacle::toPath(const double step_size) const
{
  std::vector<geometry_msgs::PoseStamped> line;

  geometry_msgs::PoseStamped pose_start;
  pose_start.header.frame_id = frame_id_;
  pose_start.header.stamp = ros::Time::now();

  geometry_msgs::PoseStamped pose_end;
  pose_end.header.frame_id = frame_id_;
  pose_end.header.stamp = ros::Time::now();

  auto polygon = toPolygon();

  pose_start.pose.position.x = min_x_.x();
  pose_start.pose.position.y = min_x_.y();

  pose_end.pose.position.x = max_x_.x();
  pose_end.pose.position.y = max_x_.y();

  line.push_back(pose_start);
  try
  {
    CommonFunctions::straightPath(line, pose_start, pose_end, frame_id_, step_size);
  }
  catch (const std::exception& e)
  {
    throw std::invalid_argument("[Obstacle] " + std::string(e.what()));
  }
  line.push_back(pose_end);

  return line;
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

std::string Obstacle::getFrameId() const
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
  for (const auto& v : obstacle.vertices_)
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
