/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/common/visualization_functions.h"

namespace VisualizationFunctions
{
void predictedPathVisualization(const geometry_msgs::Twist& velocity, const agribot_local_planner::pos& robot_center,
                                const double prediction_points_per_second, const double prediction_time,
                                const std::string& frame_id, const ros::Publisher& publisher)
{
  std::vector<geometry_msgs::PoseStamped> predicted_path;
  geometry_msgs::PoseStamped point;
  point.header.frame_id = frame_id;
  point.header.stamp = ros::Time::now();

  double radius = CommonFunctions::dCalculateRadiusFromVelocity(velocity);

  if (radius != 0)
  {
    // draw arc
    double tmp_x, tmp_y;
    for (int i = 0; i < prediction_points_per_second * prediction_time; i++)
    {
      double theta = i * velocity.angular.z / prediction_points_per_second;
      tmp_x = radius * cos(theta - common_functions::radians(90)) - radius * cos(common_functions::radians(-90));
      tmp_y = radius * sin(theta - common_functions::radians(90)) - radius * sin(common_functions::radians(-90));

      point.pose.position.x = robot_center.x + tmp_x * cos(robot_center.az) - tmp_y * sin(robot_center.az);
      point.pose.position.y = robot_center.y + tmp_x * sin(robot_center.az) + tmp_y * cos(robot_center.az);

      predicted_path.push_back(point);
    }
  }
  else
  {
    // draw straight line
    geometry_msgs::Point p;
    for (int i = 0; i < prediction_points_per_second * prediction_time; i++)
    {
      point.pose.position.x =
          robot_center.x + i * velocity.linear.x / prediction_points_per_second * cos(robot_center.az);
      point.pose.position.y =
          robot_center.y + i * velocity.linear.x / prediction_points_per_second * sin(robot_center.az);

      predicted_path.push_back(point);
    }
  }
  pathVisualization(predicted_path, frame_id, publisher);
}

void goalVisualization(const geometry_msgs::PoseStamped& goal, const double param_goal_distance_tolerance_x,
                       const double param_goal_distance_tolerance_y, const ros::Publisher& publisher)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = goal.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "next_goal";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::DELETEALL;

  marker.pose.position = goal.pose.position;
  marker.pose.orientation = goal.pose.orientation;
  marker.scale.x = 2.0 * param_goal_distance_tolerance_x;
  marker.scale.y = 2.0 * param_goal_distance_tolerance_y;
  marker.scale.z = 0.01;
  marker.color.a = 0.5;
  marker.color.r = 1.0;

  publisher.publish(marker);
  marker.action = visualization_msgs::Marker::ADD;
  publisher.publish(marker);
}

void pathVisualization(const std::vector<geometry_msgs::PoseStamped>& path, const std::string& frame_id,
                       const ros::Publisher& publisher)
{
  nav_msgs::Path local_path;
  local_path.header.stamp = ros::Time::now();
  local_path.header.frame_id = frame_id;
  local_path.poses = path;
  publisher.publish(local_path);
}

void poseVisualization(const geometry_msgs::PoseStamped& pose, const ros::Publisher& publisher)
{
  publisher.publish(pose);
}

void obstacleVisualization(const ObstacleDetection::Obstacle& obstacle, const ros::Publisher& publisher,
                           const std_msgs::ColorRGBA& color)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = obstacle.getFrameId();
  marker.header.stamp = ros::Time::now();
  marker.ns = "RecObstacles";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(1.0);
  marker.pose.orientation.w = 1;

  geometry_msgs::Point point;
  geometry_msgs::PolygonStamped polygon_stamped = obstacle.toPolygon();
  for (auto const& vertex : polygon_stamped.polygon.points)
  {
    point.x = vertex.x;
    point.y = vertex.y;
    point.z = 0;
    marker.points.push_back(point);
  }
  point.x = polygon_stamped.polygon.points.at(0).x;
  point.y = polygon_stamped.polygon.points.at(0).y;
  point.z = 0;
  marker.points.push_back(point);
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;

  marker.color = color;

  publisher.publish(marker);
}

void obstaclesVisualization(const std::vector<ObstacleDetection::Obstacle>& obstacles, const ros::Publisher& publisher)
{
  std::size_t idx = 0;
  for (auto const& obstacle : obstacles)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = obstacle.getFrameId();
    marker.header.stamp = ros::Time::now();
    marker.ns = "PolyObstacles";
    marker.id = idx++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(1.0);
    marker.pose.orientation.w = 1;

    geometry_msgs::Point point;
    geometry_msgs::PolygonStamped polygon_stamped = obstacle.toPolygon();
    for (auto const& vertex : polygon_stamped.polygon.points)
    {
      point.x = vertex.x;
      point.y = vertex.y;
      point.z = 0;
      marker.points.push_back(point);
    }
    point.x = polygon_stamped.polygon.points.at(0).x;
    point.y = polygon_stamped.polygon.points.at(0).y;
    point.z = 0;
    marker.points.push_back(point);

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    publisher.publish(marker);
  }
}

}  // namespace VisualizationFunctions
