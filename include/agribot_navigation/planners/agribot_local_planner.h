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
#include "agribot_navigation/planners/local_planners/i_local_planner.h"
#include "agribot_navigation/planners/local_planners/circles_planner.h"
#include "agribot_navigation/planners/local_planners/polynomial_planner.h"

#define DEG_TO_RAD (M_PI / 180)
#define RAD_TO_DEG (180 / M_PI)

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
  costmap_2d::Costmap2DROS* _xCostmapROS;
  tf2_ros::Buffer* _xTFBuffer;
  tf2_ros::TransformListener* _xTFListener;
  ros::Subscriber _rOdomSub;
  ros::Publisher _rPathPub;
  ros::Publisher _rPredictedPathPub;
  ros::Publisher _rGoalRangePub;
  ros::Publisher _rNextGoalPub;
  ros::Publisher _rLocalPathPub;
  // contains the global plan
  std::vector<geometry_msgs::PoseStamped> _xPlan;
  std::vector<geometry_msgs::PoseStamped> _xLocalPlan;
  std::vector<geometry_msgs::PoseStamped> _xDisplayPlan;
  nav_msgs::Path _xLocalPath;
  geometry_msgs::Twist _xVelocities;  // contains the velocity
  visualization_msgs::Marker _traveledPoints;
  geometry_msgs::PoseStamped _pRobotCenter;   // present robot center frame
  geometry_msgs::PoseStamped _pLastWaypoint;  // last target frame
  int _iPlanLength;                           // number of frames in the global plan
  // keeps track of the number of ommited frames
  int _iMissedWaypointsCount;
  // keeps track of the number for the next frame in the global plan
  int _iWaypointCount;
  bool _bGoalReached;
  bool _bInitialized;
  // time measuring for statistics
  double _dStopTime, _dStartTime, _dLastTime, _dActualTime;
  double _dLastOdomStamp;
  // deviation measuring for statistics
  double _dDeviationIntegral;
  double _dTraveledDistance;
  double _dLastTraveledDistance;
  bool bAbortExecuting;
  // params
  int param_missed_points_tollerance;
  int param_prediction_time;
  int param_prediction_points_per_second;
  double param_goal_distance_tolerance;
  double param_goal_range;
  double param_chassis_length;
  double param_min_radius;
  double param_dead_zone;
  double param_max_linear_vel;
  double param_min_linear_vel;
  double param_current_deviation_weight;
  double param_current_parallelness_weight;
  double param_deviation_weight_straight;
  double param_parallelness_weight_straight;
  double param_deviation_weight_arch;
  double param_parallelness_weight_arch;
  double param_epsilon;

  double param_design_dead_zone;
  double param_design_arch_radius;
  double param_design_step_size;
  double param_odom_filter_time;
  double param_missed_distance_coefficient;

  bool param_speed_limit;
  bool param_debug;
  bool param_statistics;
  bool param_take_front;

  bool xDesignLocalPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                        geometry_msgs::PoseStamped startPose, geometry_msgs::PoseStamped endPose,
                        ILocalPlaner* planner);

  void vOdomCallback(const nav_msgs::Odometry odometry_msg);
  pos pCalculateRobotFrontPosition(pos robotCenter);
  pos pGetFrame(int count);
  pos pGetWaypoint(int count);
  pos pCalculateGoalRange(pos nextWaypoint, pos robotCenter, double range);
  pos pCalculateProjection(pos lastWaypoint, pos nextWaypoint, pos robotCenter);
  geometry_msgs::PoseStamped xTransformPose(geometry_msgs::PoseStamped ps);
  pos pFrameFromPoseStamped(geometry_msgs::PoseStamped ps);
  geometry_msgs::Twist xSetVel(pos goal, pos nextWaypoint, pos lastWaypoint, pos robotCenter);
  geometry_msgs::Twist xDeviationApproach(pos goal, pos lastWaipoint, pos robotCenter, geometry_msgs::Twist velocity);
  geometry_msgs::Twist xLinearSpeedLimitedApproach(pos goal, pos lastWaypoint, pos robotCenter);
  geometry_msgs::Twist xLimitVelocities(geometry_msgs::Twist Velcoities);
  geometry_msgs::Twist xStopRobot();
  geometry_msgs::Twist xFinishedPathBehaviour();
  void vTakeNextGoal(pos currentGoal, pos robotCenter);
  void vCalculateDeviationIntegral(pos LastWaypoint, pos nextWaypoint, pos robotCenter);
  void vChooseDeviationParallelnessParameters(int waypointNumber);
  double dCloseToZero(double value);
  double dGetYawFromQuaternion(geometry_msgs::Quaternion quat);
  double dLimitAngle(double angle);
  double dAngleBetweenFrames(pos targetFrame, pos currentFrame);
  double dDistanceBetweenFrames(pos targetFrame, pos currentFrame);
  double dCalculateRadius(double distanceRobotWaypoint, double angleRobotWaypoint);
  double dCalculateDeviation(pos lastWaypoint, pos nextWaypoint, pos robotCenter);
  bool bMissedTooManyWaypointsCondition(int missedWaypoints);
  bool bWaypointHitCondition(pos robotCenter, pos lastWaypoint, pos nextWaypoint);
  bool bWaypointMissedCondition(pos robotCenter, pos lastWaypoint, pos nextWaypoint);
  bool bTraveledDistanceBetweenWaypoints(pos robotFront, pos lastWaypoint, pos nextWaypoint);
  bool bWithinGoalRange(pos robotFront, pos nextWaypoint);
  bool bIsLastWaypointInPath(int waypointsCount);
  bool bCenterTraveledBeyondGoal(pos robotCenter, pos lastWaypoint, pos nextWaypoint);
  bool bCenterOutOfGoalRange(pos robotCenter, pos nextWaypoint);

  void vPathVisualization(pos robotCenter);
  void vPredictedPathVisualization(geometry_msgs::Twist velocity, pos robotCenter);
  void vNextGoalVisualization(pos lastWaypoint, pos goal);
  void vGoalRangeVisualization(pos goal, pos range);
  void vLocalPathVisualization(const std::vector<geometry_msgs::PoseStamped>& path);  // NOLINT(runtime/references)
  void vVisualizations(pos lastWaypoint, pos goal, pos nextWaypoint, pos robotCenter, geometry_msgs::Twist velocity);
};
}  // namespace agribot_local_planner
