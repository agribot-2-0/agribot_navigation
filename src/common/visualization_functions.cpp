/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/common/visualization_functions.h"

namespace VisualizatioFucntions
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
      tmp_x = radius * cos(theta - CommonFunctions::toRadians(90)) - radius * cos(CommonFunctions::toRadians(-90));
      tmp_y = radius * sin(theta - CommonFunctions::toRadians(90)) - radius * sin(CommonFunctions::toRadians(-90));

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

void goalVisualization(const agribot_local_planner::pos& last_waypoint, const agribot_local_planner::pos& goal,
                            const std::string& frame_id, const ros::Publisher& publisher)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "next_goal";
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 2;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.pose.orientation.w = 1;

  geometry_msgs::Point p;
  p.x = last_waypoint.x;
  p.y = last_waypoint.y;
  p.z = 0;
  marker.points.push_back(p);

  p.x = goal.x;
  p.y = goal.y;
  p.z = 0;
  marker.points.push_back(p);

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
}  // namespace VisualizatioFucntions
