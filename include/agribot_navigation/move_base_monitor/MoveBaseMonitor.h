/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <actionlib/client/simple_action_client.h>
#include <agribot_msgs/AgribotMoveBaseCommand.h>
#include <agribot_msgs/AgribotMoveBaseMonitorResult.h>
#include <agribot_msgs/NavSatFixOrientation.h>
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

#include <string>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveBaseMonitor
{
private:
  ros::NodeHandle _nh;
  ros::ServiceClient _client_navigation;
  ros::ServiceClient _client_stop;
  std_srvs::Empty _reset_srv;
  ros::Subscriber _waypointsSub;
  ros::Subscriber _stateMachineSub;
  ros::Publisher _stateMachinePub;
  MoveBaseClient* ac;
  ros::Publisher waypoint_marker_pub_;
  ros::Publisher plan_pub_;
  geometry_msgs::TransformStamped _startingMapUTMTransform;
  tf2_ros::Buffer _tf2Buffer;
  tf2_ros::TransformListener _tf2Listener;
  double ray_width_;

  void createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path);  // NOLINT
  void publishWaypointsVisualization(const nav_msgs::Path& path);
  bool getMapUTMTransform(void);
  void waypointCallback(const agribot_msgs::NavSatFixOrientation& msg);
  geometry_msgs::PoseStamped navSatFixToUTM(const agribot_msgs::NavSatFixOrientation& msg);
  geometry_msgs::PoseStamped UTMToMap(const geometry_msgs::PoseStamped& msg);
  void stateMachineCallback(const agribot_msgs::AgribotMoveBaseCommand& command_msg);
  void isPaused();
  void monitorPathWaypoints();
  void updateOrientations(nav_msgs::Path& path, const double ray_width) const;  // NOLINT(runtime/references)
  double distanceBetweenPoses(const geometry_msgs::Pose& target_pose, const geometry_msgs::Pose& current_pose) const;
  double angleBetweenPoses(const geometry_msgs::Pose& target_pose, const geometry_msgs::Pose& current_pose) const;
  double limitAngle(double angle) const;
  double calculateLinesSpan(const double target_line_angle, const double base_line_angle) const;

  // containers
  //!< container for the manually inserted waypoints
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  nav_msgs::Path path_;  //!< container for the generated interpolated path

  // flags
  // !< flag indicating that the waypoint container must be cleared
  // to start anew
  bool clear_waypoints_;
  bool _pathLoaded;
  bool _pauseGoal;
  bool _pauseGoalLatch;
  bool _userAbort;
  agribot_msgs::AgribotMoveBaseMonitorResult resultMsg;

public:
  MoveBaseMonitor();
  void run();
};
