/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/waypoint_global_planner.h"

#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

PLUGINLIB_EXPORT_CLASS(waypoint_global_planner::WaypointGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace waypoint_global_planner
{
WaypointGlobalPlanner::WaypointGlobalPlanner() : costmap_ros_(NULL), initialized_(false)
{
}

WaypointGlobalPlanner::WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

WaypointGlobalPlanner::~WaypointGlobalPlanner()
{
}

void WaypointGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    // get the costmap
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~" + name);

    // load parameters
    try
    {
      waypoints_per_meter_ = ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/waypoints_per_meter");
      param_goal_distance_tolerance =
          ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/goal_distance_tolerance");
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("[GlobalPlanner] " << ex.what());
    }

    waypoint_marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);

    path_.header.frame_id = "map";

    initialized_ = true;
    ROS_INFO("[GlobalPlanner] Planner has been initialized");
  }
  else
  {
    ROS_WARN("[GlobalPlanner] This planner has already been initialized");
  }
}

bool WaypointGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose,
                                     const geometry_msgs::PoseStamped& goal,
                                     std::vector<geometry_msgs::PoseStamped>& plan)
{
  ROS_INFO("[GlobalPlanner] Make plan start");
  path_.header.stamp = ros::Time::now();
  path_.poses.clear();
  path_.poses.insert(path_.poses.begin(), start_pose);
  path_.poses.insert(path_.poses.end(), goal);
  interpolatePath(path_);
  plan_pub_.publish(path_);
  // createAndPublishMarkersFromPath(path_.poses);
  plan = path_.poses;
  ROS_INFO("[GlobalPlanner] Make plan finish");
  return true;
}

void WaypointGlobalPlanner::interpolatePath(nav_msgs::Path& path)
{
  std::vector<geometry_msgs::PoseStamped> temp_path;
  for (int i = 0; i < static_cast<int>(path.poses.size() - 1); i++)
  {
    // calculate distance between two consecutive waypoints
    double x1 = path.poses[i].pose.position.x;
    double y1 = path.poses[i].pose.position.y;
    double x2 = path.poses[i + 1].pose.position.x;
    double y2 = path.poses[i + 1].pose.position.y;
    double dist = hypot(x1 - x2, y1 - y2);
    int num_wpts = dist * waypoints_per_meter_;

    temp_path.push_back(path.poses[i]);
    geometry_msgs::PoseStamped p = path.poses[i];
    for (int j = 1; j < num_wpts; j++)
    {
      p.pose.position.x = x1 + static_cast<double>(j) / num_wpts * (x2 - x1);
      p.pose.position.y = y1 + static_cast<double>(j) / num_wpts * (y2 - y1);
      temp_path.push_back(p);
    }
  }

  // update sequence of poses
  for (size_t i = 0; i < temp_path.size(); i++)
    temp_path[i].header.seq = static_cast<int>(i);

  temp_path.push_back(path.poses.back());
  // temp_path.push_back(path.poses.back());
  path.poses = temp_path;
}

void WaypointGlobalPlanner::createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path)
{
  // clear previous markers
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker marker;
  marker.header = path[0].header;
  marker.ns = "/move_base/waypoint_global_planner";
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.scale.x = 2 * param_goal_distance_tolerance;
  marker.scale.y = 2 * param_goal_distance_tolerance;
  marker.scale.z = 0.01;
  marker.color.a = 0.5;
  marker.color.g = 1.0;
  marker.id = 0;
  markers.markers.push_back(marker);
  waypoint_marker_pub_.publish(markers);
  marker.action = visualization_msgs::Marker::ADD;
  markers.markers.clear();

  for (size_t i = 0; i < path.size(); i++)
  {
    marker.id = i;
    marker.pose.position = path[i].pose.position;
    marker.pose.orientation.w = 1;
    markers.markers.push_back(marker);
  }

  waypoint_marker_pub_.publish(markers);
}

}  // namespace waypoint_global_planner
