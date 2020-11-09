/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/common/visualization_functions.h"

namespace VisualizatioFucntions
{
void vPredictedPathVisualization(const geometry_msgs::Twist& velocity, const agribot_local_planner::pos& robot_center,
                                 const double prediction_points_per_second, const double prediction_time,
                                 const ros::Publisher& publisher)
{
  visualization_msgs::Marker futurePoints;
  futurePoints.header.frame_id = "map";
  futurePoints.header.stamp = ros::Time::now();
  futurePoints.ns = "predicted_path";
  futurePoints.action = visualization_msgs::Marker::ADD;
  futurePoints.id = 1;
  futurePoints.type = visualization_msgs::Marker::LINE_STRIP;
  futurePoints.scale.x = 0.1;
  futurePoints.color.r = 1.0;
  futurePoints.color.a = 1.0;
  futurePoints.pose.orientation.w = 1;

  // calculating turn radius from angular and linear velocity
  // R = v / w

  double radius;
  if (velocity.angular.z != 0)
  {
    radius = velocity.linear.x / velocity.angular.z;
  }
  else
  {
    radius = 0;
  }

  if (radius != 0)
  {
    // draw arc
    geometry_msgs::Point p, tmp;
    for (int i = 0; i < prediction_points_per_second * prediction_time; i++)
    {
      double theta = i * velocity.angular.z / prediction_points_per_second;
      tmp.x = radius * cos(theta - CommonFunctions::toRadians(90)) - radius * cos(CommonFunctions::toRadians(-90));
      tmp.y = radius * sin(theta - CommonFunctions::toRadians(90)) - radius * sin(CommonFunctions::toRadians(-90));

      p.x = robot_center.x + tmp.x * cos(robot_center.az) - tmp.y * sin(robot_center.az);
      p.y = robot_center.y + tmp.x * sin(robot_center.az) + tmp.y * cos(robot_center.az);

      p.z = 0.1;
      futurePoints.points.push_back(p);
    }
  }
  else
  {
    // draw straight line
    geometry_msgs::Point p;
    for (int i = 0; i < prediction_points_per_second * prediction_time; i++)
    {
      p.x = robot_center.x + i * velocity.linear.x / prediction_points_per_second * cos(robot_center.az);
      p.y = robot_center.y + i * velocity.linear.x / prediction_points_per_second * sin(robot_center.az);
      p.z = 0.1;
      futurePoints.points.push_back(p);
    }
  }
  publisher.publish(futurePoints);
}

void vNextGoalVisualization(const agribot_local_planner::pos& last_waypoint, const agribot_local_planner::pos& goal,
                            const ros::Publisher& publisher)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
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

void vGoalRangeVisualization(const agribot_local_planner::pos& goal, const agribot_local_planner::pos& range,
                             const double goal_distance_tolerance, const ros::Publisher& publisher)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "goal_range";
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.color.r = 1.0;
  marker.color.a = 0.2;

  geometry_msgs::Point p;
  marker.scale.x = 2 * goal_distance_tolerance;
  marker.scale.y = 2 * goal_distance_tolerance;
  marker.scale.z = 0.01;
  marker.pose.position.x = goal.x;
  marker.pose.position.y = goal.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1;

  publisher.publish(marker);
}

void vLocalPathVisualization(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& publisher)
{
  nav_msgs::Path local_path;
  local_path.header.stamp = ros::Time::now();
  local_path.header.frame_id = "map";
  local_path.poses.clear();
  local_path.poses = path;
  publisher.publish(local_path);
}
}  // namespace VisualizatioFucntions
