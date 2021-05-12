/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>

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
  std::vector<geometry_msgs::PoseStamped> toPath(const double step_size) const;
  friend std::ostream& operator<<(std::ostream& out, const Obstacle& o);
  Obstacle& operator+=(const Obstacle& obstacle);
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
  void addVertex(Eigen::Vector2d vertex);
  void initializeEmpty();

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

  double computeArea() const;
  double computePerimeter() const;
  Eigen::Vector2d findCentroid() const;
  bool straightPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                    const geometry_msgs::PoseStamped& start_point, const geometry_msgs::PoseStamped& end_point,
                    const double step_size) const;

  /**
   * @brief 2D Cross product of two vectors defined by two points and a common origin
   * @param O Origin
   * @param A First point
   * @param B Second point
   * @tparam P1 2D Point type with x and y member fields
   * @tparam P2 2D Point type with x and y member fields
   * @tparam P3 2D Point type with x and y member fields
   */
  template <typename P1, typename P2, typename P3>
  long double cross(const P1& O, const P2& A, const P3& B) const
  {
    return (long double)(A.x - O.x) * (long double)(B.y() - O.y) -
           (long double)(A.y - O.y) * (long double)(B.x() - O.x);
  }
};
}  // namespace ObstacleDetection
