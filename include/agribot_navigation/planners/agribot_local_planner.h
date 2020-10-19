/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <base_local_planner/goal_functions.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <string>
#include <vector>

#include "agribot_navigation/planners/structures.h"
#include "agribot_navigation/planners/common_functions.h"
#include "agribot_navigation/planners/local_planners/i_local_planner.h"
#include "agribot_navigation/planners/local_planners/circles_planner.h"
#include "agribot_navigation/planners/local_planners/hermite_planner.h"
#include "agribot_navigation/planners/default_values.h"

namespace agribot_local_planner
{
class AgribotPlannerROS : public nav_core::BaseLocalPlanner
{
public:
  AgribotPlannerROS();
  ~AgribotPlannerROS();
  void initialize(std::string name, tf2_ros::Buffer* tfBuffer, costmap_2d::Costmap2DROS* costmapROS);
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);  // NOLINT(runtime/references)
  bool isGoalReached();

private:
  costmap_2d::Costmap2DROS* _costmap_ros;
  tf2_ros::Buffer* _tf_buffer;
  tf2_ros::TransformListener* _tf_listener;
  ros::Subscriber _odom_sub;
  ros::Publisher _path_pub;
  ros::Publisher _predicted_path_pub;
  ros::Publisher _goal_range_pub;
  ros::Publisher _next_goal_pub;
  ros::Publisher _local_path_pub;
  // contains the global plan
  std::vector<geometry_msgs::PoseStamped> _plan;
  std::vector<geometry_msgs::PoseStamped> _local_plan;
  std::vector<geometry_msgs::PoseStamped> _display_plan;
  nav_msgs::Path _local_path;
  geometry_msgs::Twist _velocities;  // contains the velocity
  visualization_msgs::Marker _traveled_points;
  geometry_msgs::PoseStamped _robot_center;   // present robot center frame
  geometry_msgs::PoseStamped _last_waypoint;  // last target frame
  // keeps track of the number of ommited frames
  int _missed_waypoints_count;
  // keeps track of the number for the next frame in the global plan
  int _waypoint_count;
  bool _goal_reached;
  bool _initialized;
  // time measuring for statistics
  double _stop_time, _start_time, _dlast_time, _actual_time;
  double _last_odom_stamp;
  // deviation measuring for statistics
  double _deviation_integral;
  double _traveled_distance;
  double _last_traveled_distance;
  bool _abort_executing;
  // params
  int param_missed_points_tolerance;         // n
  int param_prediction_time;                  // seconds
  int param_prediction_points_per_second;     // n/second
  double param_goal_distance_tolerance;       // metres
  double param_goal_range;                    // factor
  double param_chassis_length;                // metres
  double param_min_radius;                    // metres
  double param_dead_zone;                     // degrees
  double param_max_linear_vel;                // m/s
  double param_min_linear_vel;                // m/s
  double param_current_deviation_weight;      // factor
  double param_current_parallelness_weight;   // factor
  double param_deviation_weight_straight;     // factor
  double param_parallelness_weight_straight;  // factor
  double param_deviation_weight_arc;          // factor
  double param_parallelness_weight_arc;       // factor
  double param_epsilon;                       // unitless

  double param_design_dead_zone;             // degrees
  double param_design_arc_radius;            // metres
  double param_design_step_size;             // metres
  double param_hermite_magnitude;            // factor
  double param_odom_filter_time;             // seconds
  double param_missed_distance_coefficient;  // factor >1

  bool param_speed_limit;
  bool param_debug;
  bool param_statistics;
  bool param_take_front;

  bool xDesignLocalPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                        const geometry_msgs::PoseStamped& startPose, const geometry_msgs::PoseStamped& endPose,
                        ILocalPlanner* const planner);

  void vOdomCallback(const nav_msgs::Odometry& odometry_msg);
  pos pCalculateRobotFrontPosition(const pos& robotCenter);
  pos pGetFrame(int count);
  pos pGetWaypoint(int count);
  pos pCalculateGoalRange(const pos& nextWaypoint, const pos& robotCenter, double range);
  pos pCalculateProjection(const pos& lastWaypoint, const pos& nextWaypoint, const pos& robotCenter);
  geometry_msgs::PoseStamped xTransformPose(geometry_msgs::PoseStamped ps);
  geometry_msgs::Twist xSetVel(const pos& goal, const pos& nextWaypoint, const pos& lastWaypoint,
                               const pos& robotCenter);
  geometry_msgs::Twist xDeviationApproach(const pos& goal, const pos& lastWaypoint, const pos& robotCenter,
                                          geometry_msgs::Twist velocity);
  geometry_msgs::Twist xLinearSpeedLimitedApproach(const pos& goal, const pos& lastWaypoint, const pos& robotCenter);
  geometry_msgs::Twist xLimitVelocities(geometry_msgs::Twist Velcoities);
  geometry_msgs::Twist xStopRobot();
  geometry_msgs::Twist xFinishedPathBehaviour();
  void vTakeNextGoal(const pos& currentGoal, const pos& robotCenter);
  void vCalculateDeviationIntegral(const pos& lastWaypoint, const pos& nextWaypoint, const pos& robotCenter);
  void vChooseDeviationParallelnessParametres(int waypointNumber);
  double dCloseToZero(double value);
  double dCalculateRadius(double distanceRobotWaypoint, double angleRobotWaypoint);
  double dCalculateDeviation(const pos& lastWaypoint, const pos& nextWaypoint, const pos& robotCenter);
  bool bMissedTooManyWaypointsCondition(int missedWaypoints);
  bool bWaypointHitCondition(const pos& robotCenter, const pos& lastWaypoint, const pos& nextWaypoint);
  bool bWaypointMissedCondition(const pos& robotCenter, const pos& lastWaypoint, const pos& nextWaypoint);
  bool bTraveledDistanceBetweenWaypoints(pos robotFront, const pos& lastWaypoint, const pos& nextWaypoint);
  bool bWithinGoalRange(pos robotFront, const pos& nextWaypoint);
  bool bIsLastWaypointInPath(int waypointsCount);
  bool bCenterTraveledBeyondGoal(const pos& robotCenter, const pos& lastWaypoint, const pos& nextWaypoint);
  bool bCenterOutOfGoalRange(const pos& robotCenter, const pos& nextWaypoint);

  void vPathVisualization(const pos& robotCenter);
  void vPredictedPathVisualization(const geometry_msgs::Twist& velocity, const pos& robotCenter);
  void vNextGoalVisualization(const pos& lastWaypoint, const pos& goal);
  void vGoalRangeVisualization(const pos& goal, const pos& range);
  void vLocalPathVisualization(const std::vector<geometry_msgs::PoseStamped>& path);  // NOLINT(runtime/references)
  void vVisualizations(const pos& lastWaypoint, const pos& goal, const pos& nextWaypoint, const pos& robotCenter,
                       const geometry_msgs::Twist& velocity);
};
}  // namespace agribot_local_planner
