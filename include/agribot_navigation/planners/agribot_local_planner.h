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
#include <unordered_map>
#include <memory>

#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/common/common_functions.h"
#include "agribot_navigation/planners/local_planners/i_local_planner.h"
#include "agribot_navigation/planners/local_planners/circles_planner.h"
#include "agribot_navigation/planners/local_planners/hermite_planner.h"
#include "agribot_navigation/common/default_values.h"

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
  costmap_2d::Costmap2DROS* costmap_ros_;
  tf2_ros::Buffer* tf_buffer_;
  tf2_ros::TransformListener* tf_listener_;
  ros::Subscriber odom_sub_;
  ros::Publisher path_pub_;
  ros::Publisher predicted_path_pub_;
  ros::Publisher goal_range_pub_;
  ros::Publisher next_goal_pub_;
  ros::Publisher local_path_pub_;
  // contains the global plan
  std::vector<geometry_msgs::PoseStamped> plan_;
  std::vector<geometry_msgs::PoseStamped> local_plan_;
  std::vector<geometry_msgs::PoseStamped> display_plan_;
  nav_msgs::Path local_path_;
  geometry_msgs::Twist velocities_;  // contains the velocity
  visualization_msgs::Marker traveled_points_;
  geometry_msgs::PoseStamped robot_center_;   // present robot center frame
  geometry_msgs::PoseStamped last_waypoint_;  // last target frame
  std::unordered_map<std::string, std::unique_ptr<ILocalPlanner>> planners_;
  // keeps track of the number of ommited frames
  int missed_waypoints_count_;
  // keeps track of the number for the next frame in the global plan
  int waypoint_count_;
  bool goal_reached_;
  bool initialized_;
  // time measuring for statistics
  double stop_time_, start_time_, last_time_, actual_time_;
  double last_odom_stamp_;
  // deviation measuring for statistics
  double deviation_integral_;
  double traveled_distance_;
  double last_traveled_distance_;
  bool abort_executing_;
  // params
  int param_missed_points_tolerance_;          // n
  int param_prediction_time_;                  // seconds
  int param_prediction_points_per_second_;     // n/second
  double param_goal_distance_tolerance_;       // metres
  double param_goal_range_;                    // factor
  double param_chassis_length_;                // metres
  double param_min_radius_;                    // metres
  double param_dead_zone_;                     // degrees
  double param_max_linear_vel_;                // m/s
  double param_min_linear_vel_;                // m/s
  double param_current_deviation_weight_;      // factor
  double param_current_parallelness_weight_;   // factor
  double param_deviation_weight_straight_;     // factor
  double param_parallelness_weight_straight_;  // factor
  double param_deviation_weight_arc_;          // factor
  double param_parallelness_weight_arc_;       // factor
  double param_epsilon_;                       // unitless

  double param_design_dead_zone_;             // degrees
  double param_design_arc_radius_;            // metres
  double param_design_step_size_;             // metres
  double param_hermite_magnitude_;            // factor
  double param_odom_filter_time_;             // seconds
  double param_missed_distance_coefficient_;  // factor >1

  bool param_speed_limit_;
  bool param_debug_;
  bool param_statistics_;
  bool param_take_front_;

  void vOdomCallback(const nav_msgs::Odometry& odometry_msg);
  void createPlanners();
  pos pCalculateRobotFrontPosition(const pos& robot_center);
  pos pGetFrame(int count);
  pos pGetWaypoint(int count);
  pos pCalculateGoalRange(const pos& next_waypoint, const pos& robot_center, double range);
  pos pCalculateProjection(const pos& last_waypoint, const pos& next_waypoint, const pos& robot_center);
  geometry_msgs::PoseStamped xTransformPose(geometry_msgs::PoseStamped ps);
  geometry_msgs::Twist xSetVel(const pos& goal, const pos& next_waypoint, const pos& last_waypoint,
                               const pos& robot_center);
  geometry_msgs::Twist xDeviationApproach(const pos& goal, const pos& last_waypoint, const pos& robot_center,
                                          geometry_msgs::Twist velocity);
  geometry_msgs::Twist xLinearSpeedLimitedApproach(const pos& goal, const pos& last_waypoint, const pos& robot_center);
  geometry_msgs::Twist xLimitVelocities(geometry_msgs::Twist velocities);
  geometry_msgs::Twist xStopRobot();
  geometry_msgs::Twist xFinishedPathBehaviour();
  void vTakeNextGoal(const pos& current_goal, const pos& robot_center);
  void vCalculateDeviationIntegral(const pos& last_waypoint, const pos& next_waypoint, const pos& robot_center);
  void vChooseDeviationParallelnessParametres(int waypoint_number);
  double dCloseToZero(double value);
  double dCalculateRadius(double distance_robot_waypoint, double angle_robot_waypoint);
  double dCalculateDeviation(const pos& last_waypoint, const pos& next_waypoint, const pos& robot_center);
  bool bMissedTooManyWaypointsCondition(int missed_waypoints);
  bool bWaypointHitCondition(const pos& robot_center, const pos& last_waypoint, const pos& next_waypoint);
  bool bWaypointMissedCondition(const pos& robot_center, const pos& last_waypoint, const pos& next_waypoint);
  bool bTraveledDistanceBetweenWaypoints(pos robot_front, const pos& last_waypoint, const pos& next_waypoint);
  bool bWithinGoalRange(pos robot_front, const pos& next_waypoint);
  bool bIsLastWaypointInPath(int waypoints_count);
  bool bCenterTraveledBeyondGoal(const pos& robot_center, const pos& last_waypoint, const pos& next_waypoint);
  bool bCenterOutOfGoalRange(const pos& robot_center, const pos& next_waypoint);

  void vPathVisualization(const pos& robot_center);
  void vPredictedPathVisualization(const geometry_msgs::Twist& velocity, const pos& robot_center);
  void vNextGoalVisualization(const pos& last_waypoint, const pos& goal);
  void vGoalRangeVisualization(const pos& goal, const pos& range);
  void vLocalPathVisualization(const std::vector<geometry_msgs::PoseStamped>& path);  // NOLINT(runtime/references)
  void vVisualizations(const pos& last_waypoint, const pos& goal, const pos& next_waypoint, const pos& robot_center,
                       const geometry_msgs::Twist& velocity);
};
}  // namespace agribot_local_planner
