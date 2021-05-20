/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <agribot_msgs/PerformAutonomousPathAction.h>
#include <agribot_msgs/AgribotNavigationManagerResult.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <robot_localization/navsat_conversions.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>

#include <memory>
#include <string>
#include <vector>

class NavigationManager
{
private:
  ros::NodeHandle node_handle_;
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> move_base_action_client_;
  ros::Publisher waypoint_marker_publisher_;
  ros::Publisher plan_publisher_;
  geometry_msgs::TransformStamped starting_map_utm_transform_;
  tf2_ros::Buffer tf_to_buffer_;
  tf2_ros::TransformListener tf_to_listener_;
  double ray_width_;

  std::unique_ptr<actionlib::SimpleActionServer<agribot_msgs::PerformAutonomousPathAction>> path_action_server_;
  agribot_msgs::PerformAutonomousPathResult path_action_result_;
  agribot_msgs::PerformAutonomousPathFeedback path_action_feedback_;

  // container for the manually inserted waypoints
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  // container for the generated interpolated path
  nav_msgs::Path path_;

  void createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path);  // NOLINT
  void publishWaypointsVisualization(const nav_msgs::Path& path);
  bool getMapUtmTransform(void);
  void processWaypoint(const sensor_msgs::NavSatFix& msg);
  geometry_msgs::PoseStamped navSatFixToUtm(const sensor_msgs::NavSatFix& msg);
  geometry_msgs::PoseStamped utmToMap(const geometry_msgs::PoseStamped& msg);
  bool monitorPathWaypoints(uint current_waypoint);
  void updateOrientations(nav_msgs::Path& path, const double ray_width) const;  // NOLINT(runtime/references)
  double distanceBetweenPoses(const geometry_msgs::Pose& target_pose, const geometry_msgs::Pose& current_pose) const;
  double angleBetweenPoses(const geometry_msgs::Pose& target_pose, const geometry_msgs::Pose& current_pose) const;
  double limitAngle(double angle) const;
  double calculateLinesSpan(const double target_line_angle, const double base_line_angle) const;
  void performAutonomousPathCallback(const agribot_msgs::PerformAutonomousPathGoalConstPtr& goal);

public:
  NavigationManager();
  void run();
};
