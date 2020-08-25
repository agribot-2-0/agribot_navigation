/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/agribot_local_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(agribot_local_planner::AgribotPlannerROS, nav_core::BaseLocalPlanner)

namespace agribot_local_planner
{
AgribotPlannerROS::AgribotPlannerROS() : _xCostmapROS(NULL), _bInitialized(false)
{
}

AgribotPlannerROS::~AgribotPlannerROS()
{
}

void AgribotPlannerROS::initialize(std::string name, tf2_ros::Buffer* tfBuffer, costmap_2d::Costmap2DROS* costmapROS)
{
  if (!_bInitialized)
  {
    _xCostmapROS = costmapROS;
    _xTFBuffer = tfBuffer;
    _xTFListener = new tf2_ros::TransformListener(*tfBuffer);

    ros::NodeHandle gn;
    ros::NodeHandle pnh("~" + name);

    _rOdomSub = gn.subscribe("odom", 100, &AgribotPlannerROS::vOdomCallback, this);
    _rPathPub = pnh.advertise<visualization_msgs::Marker>("travelled_path", 10);
    _rPredictedPathPub = pnh.advertise<visualization_msgs::Marker>("predicted_path", 10);
    _rGoalRangePub = pnh.advertise<visualization_msgs::Marker>("goal_range", 10);
    _rNextGoalPub = pnh.advertise<visualization_msgs::Marker>("next_goal", 10);
    _rLocalPathPub = pnh.advertise<nav_msgs::Path>("local_path", 10);
    _xLocalPath.header.frame_id = "map";

    pnh.param("goal_distance_tolerance", param_goal_distance_tolerance, 0.75);
    pnh.param("goal_range", param_goal_range, 0.66);
    pnh.param("missed_points_tollerance", param_missed_points_tollerance, 10000);

    pnh.param("chassis_length", param_chassis_length, 2.1);
    pnh.param("min_radius", param_min_radius, 2.0);

    pnh.param("prediction_time", param_prediction_time, 60);
    pnh.param("prediction_points_per_second", param_prediction_points_per_second, 2);

    pnh.param("dead_zone", param_dead_zone, 3.0);
    pnh.param("max_linear_vel", param_max_linear_vel, 0.7);
    pnh.param("min_linear_vel", param_min_linear_vel, 0.1);

    pnh.param("deviation_weight_straight", param_deviation_weight_straight, 0.1);
    pnh.param("parallelness_weight_straight", param_parallelness_weight_straight, 10.0);
    pnh.param("deviation_weight_arch", param_deviation_weight_arch, 0.1);
    pnh.param("parallelness_weight_arch", param_parallelness_weight_arch, 10.0);
    pnh.param("epsilon", param_epsilon, 1e-3);

    pnh.param("design_dead_zone", param_local_dead_zone,
              0.0);  // degrees
    pnh.param("design_arch_radius", param_arch_radius,
              2.0);  // meters
    pnh.param("design_arch_step", param_arch_step,
              5.0);  // degrees

    pnh.param("odom_filter_time", param_odom_filter_time, 0.1);
    pnh.param("missed_distance_coefficient", param_missed_distance_coefficient, 1.1);

    // be sure that path planning arch radius is greater or equal to minimum turning radius
    if (param_arch_radius < param_min_radius)
    {
      param_arch_radius = param_min_radius;
    }

    pnh.param("speed_limit", param_speed_limit, true);
    pnh.param("debug", param_debug, false);
    pnh.param("statistics", param_statistics, true);
    pnh.param("take_front", param_take_front, false);

    _pRobotCenter.header.frame_id = "odom";
    _pLastWaypoint.header.frame_id = "odom";

    _bInitialized = true;

    if (param_statistics)
    {
      _dStopTime = 0;
      _dStartTime = 0;
      _dLastTime = 0;
      _dActualTime = 0;
    }

    ROS_DEBUG_STREAM("[AgribotPlanner] Plugin initialized.");
    gn.setParam("/agribot/agribot_local_planer/is_inited", true);
  }
  else
  {
    ROS_WARN_STREAM("[AgribotPlanner] This planner has already been "
                    << "initialized, doing nothing.");
  }
}

bool AgribotPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!_bInitialized)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }
  bAbortExecuting = false;
  _iWaypointCount = 1;
  _iMissedWaypointsCount = 0;
  _xPlan = orig_global_plan;
  _xLocalPlan = xDesignLocalPath(_pRobotCenter, _xPlan.back());
  _bGoalReached = false;
  _traveledPoints = visualization_msgs::Marker();
  _dLastOdomStamp = 0;

  _pLastWaypoint = _pRobotCenter;
  _dTraveledDistance = 0;
  _dLastTraveledDistance = 0;

  if (param_statistics)
  {
    _dDeviationIntegral = 0;

    _dStartTime = ros::Time::now().toSec();
    _dLastTime = 0;
    _dActualTime = 0;
  }
  ROS_INFO_STREAM("[AgribotPlanner] Received new plan, "
                  << "plan length = " << _iPlanLength);
  return true;
}

bool AgribotPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  geometry_msgs::Twist velocity;
  pos goal;
  // _xLocalPlan = xDesignLocalPath(_pRobotCenter, _xPlan.back());

  vLocalPathVisualization(_xLocalPlan);

  if (!_bInitialized)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }
  if (_xLocalPlan.size() != 0)
  {
    // Check where robot is
    pos nextWaypoint = pGetWaypoint(_iWaypointCount);
    pos lastWaypoint = pFrameFromPoseStamped(xTransformPose(_pLastWaypoint));
    pos robotCenter = pFrameFromPoseStamped(xTransformPose(_pRobotCenter));

    goal = nextWaypoint;

    // Decide whether following straight or arch piece of path
    vChooseDeviationParallelnessParameters(_iWaypointCount);

    // Deviation error for statistics
    if (param_statistics)
    {
      vCalculateDeviationIntegral(lastWaypoint, nextWaypoint, robotCenter);
    }

    // Increment missed points number
    if (bWaypointMissedCondition(robotCenter, lastWaypoint, nextWaypoint))
    {
      ROS_ERROR_STREAM("[AgribotPlanner] OUT OF PATH!");
      _iMissedWaypointsCount++;
    }

    // Next goal condition
    if (!bIsLastWaypointInPath(_iWaypointCount) && (bWaypointHitCondition(robotCenter, lastWaypoint, nextWaypoint) ||
                                                    bWaypointMissedCondition(robotCenter, lastWaypoint, nextWaypoint)))
    {
      if (param_statistics)
      {
        ROS_INFO_STREAM("[AgribotPlanner] NEXT GOAL - Traveled " << _dTraveledDistance);
      }
      vTakeNextGoal(goal, robotCenter);
      nextWaypoint = pGetWaypoint(_iWaypointCount);
    }

    // Calculate velocities
    velocity = xSetVel(goal, nextWaypoint, lastWaypoint, robotCenter);

    velocity.linear.x = dCloseToZero(velocity.linear.x);
    velocity.linear.y = dCloseToZero(velocity.linear.y);
    velocity.linear.z = dCloseToZero(velocity.linear.z);
    velocity.angular.x = dCloseToZero(velocity.angular.x);
    velocity.angular.y = dCloseToZero(velocity.angular.y);
    velocity.angular.z = dCloseToZero(velocity.angular.z);

    // Stop condition
    if (bMissedTooManyWaypointsCondition(_iMissedWaypointsCount) ||
        (bIsLastWaypointInPath(_iWaypointCount) && bWaypointHitCondition(robotCenter, lastWaypoint, nextWaypoint)) ||
        (bIsLastWaypointInPath(_iWaypointCount) && bWaypointMissedCondition(robotCenter, lastWaypoint, nextWaypoint)))
    {
      velocity = xFinishedPathBehaviour();
    }

    // Visualization
    vVisualizations(lastWaypoint, goal, nextWaypoint, robotCenter, velocity);
  }
  if (bAbortExecuting)
  {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    ROS_ERROR("[AgribotPlanner] Abort plan");
    return false;
  }
  else
  {
    cmd_vel = velocity;
    return true;
  }
}

std::vector<geometry_msgs::PoseStamped> AgribotPlannerROS::xDesignLocalPath(geometry_msgs::PoseStamped startPose,
                                                                            geometry_msgs::PoseStamped endPose)
{
  std::vector<geometry_msgs::PoseStamped> localPlan;

  pos startPoint = pFrameFromPoseStamped(startPose);
  pos endPoint = pFrameFromPoseStamped(endPose);
  pos robotCenter = pFrameFromPoseStamped(xTransformPose(_pRobotCenter));

  double path_angle = dAngleBetweenFrames(endPoint, startPoint);
  double reverse_path_angle = dAngleBetweenFrames(robotCenter, endPoint);
  // double reverse_path_angle = path_angle;
  double end_orientation = endPoint.az;
  double robot_orientation = robotCenter.az - M_PI;
  // double robot_orientation = robotCenter.az;
  double end_angle_difference = dLimitAngle((end_orientation - path_angle));
  double start_angle_difference = dLimitAngle((robot_orientation - reverse_path_angle));

  pos end_c2_center =
      pCalculateC2Center(endPoint, end_orientation, end_angle_difference, param_arch_radius, param_local_dead_zone);
  pos end_c1_center =
      pCalculateC1Center(end_c2_center, endPoint, path_angle, end_angle_difference, param_arch_radius, param_local_dead_zone);
  pos end_tangent_point =
      pCalculateTangentPoint(end_c1_center, path_angle, end_angle_difference, param_arch_radius, param_local_dead_zone);

  pos start_c2_center =
      pCalculateC2Center(robotCenter, robot_orientation, start_angle_difference, param_arch_radius, param_local_dead_zone);
  pos start_c1_center =
      pCalculateC1Center(start_c2_center, robotCenter, reverse_path_angle, start_angle_difference, param_arch_radius, param_local_dead_zone);
  pos start_tangent_point =
      pCalculateTangentPoint(start_c1_center, reverse_path_angle, start_angle_difference, param_arch_radius, param_local_dead_zone);

  ROS_ERROR_STREAM("[AgribotPlanner] start_c1_center x\t"
                   <<start_c1_center.x << "\ty\t" << start_c1_center.y);
  ROS_ERROR_STREAM("[AgribotPlanner] start_c2_center x\t"
                   <<start_c2_center.x << "\ty\t" << start_c2_center.y);
  vPathVisualization(start_c1_center);
  vPathVisualization(start_c2_center);
  // vPathVisualization(start_tangent_point);
  // vPathVisualization(start_c1_center);

  localPlan = xAssemblePath(startPose, endPose, end_tangent_point, end_c1_center, end_c2_center);

  return localPlan;
}

pos AgribotPlannerROS::pCalculateC2Center(pos endPoint, double end_orientation, double angle_difference,
                                          double arch_radius, double dead_zone)
{
  // determine center of circle tangent to orientation vector
  // as a vector of length equals to given radius
  // perpendicular to the orientation vector

  pos c2_center;
  if (angle_difference < -dead_zone * DEG_TO_RAD)
  {
    double perpendicular = end_orientation - M_PI / 2;
    c2_center.x = endPoint.x + arch_radius * cos(perpendicular);
    c2_center.y = endPoint.y + arch_radius * sin(perpendicular);
  }
  else if (angle_difference > dead_zone * DEG_TO_RAD)
  {
    double perpendicular = end_orientation + M_PI / 2;
    c2_center.x = endPoint.x + arch_radius * cos(perpendicular);
    c2_center.y = endPoint.y + arch_radius * sin(perpendicular);
  }
  else
  {
    c2_center.x = std::numeric_limits<double>::quiet_NaN();
    c2_center.y = std::numeric_limits<double>::quiet_NaN();
  }
  return c2_center;
}

pos AgribotPlannerROS::pCalculateC1Center(pos c2_center, pos endPoint, double path_angle, double angle_difference,
                                          double arch_radius, double dead_zone)
{
  // determine center of circle tangent to original path
  // as a interception of circle with radius twice as big as given radius
  // and line parallel to original path
  // distance from line and path equals given radius

  pos c1_center;
  pos point_on_parallel;
  double dx, dy, a, b, c, discriminant, t1, t2, diameter;
  diameter = arch_radius * 2.0;

  dx = cos(path_angle);
  dy = sin(path_angle);

  if (angle_difference < -dead_zone * DEG_TO_RAD)
  {
    double perpendicular = path_angle + 90 * DEG_TO_RAD;
    point_on_parallel.x = endPoint.x + arch_radius * cos(perpendicular);
    point_on_parallel.y = endPoint.y + arch_radius * sin(perpendicular);
    a = dx * dx + dy * dy;
    b = 2 * (dx * (point_on_parallel.x - c2_center.x) + dy * (point_on_parallel.y - c2_center.y));
    c = (point_on_parallel.x - c2_center.x) * (point_on_parallel.x - c2_center.x) +
        (point_on_parallel.y - c2_center.y) * (point_on_parallel.y - c2_center.y) - diameter * diameter;
    discriminant = b * b - 4 * a * c;
    if (discriminant >= 0)
    {
      t1 = (-b + sqrt(discriminant)) / (2 * a);
      t2 = (-b - sqrt(discriminant)) / (2 * a);
      c1_center.x = dx * t2 + point_on_parallel.x;
      c1_center.y = dy * t2 + point_on_parallel.y;
    }
    else
    {
      c1_center.x = std::numeric_limits<double>::quiet_NaN();
      c1_center.y = std::numeric_limits<double>::quiet_NaN();
    }
  }
  else if (angle_difference > dead_zone * DEG_TO_RAD)
  {
    double perpendicular = path_angle - 90 * DEG_TO_RAD;
    point_on_parallel.x = endPoint.x + arch_radius * cos(perpendicular);
    point_on_parallel.y = endPoint.y + arch_radius * sin(perpendicular);
    a = dx * dx + dy * dy;
    b = 2 * (dx * (point_on_parallel.x - c2_center.x) + dy * (point_on_parallel.y - c2_center.y));
    c = (point_on_parallel.x - c2_center.x) * (point_on_parallel.x - c2_center.x) +
        (point_on_parallel.y - c2_center.y) * (point_on_parallel.y - c2_center.y) - diameter * diameter;
    discriminant = b * b - 4 * a * c;
    if (discriminant >= 0)
    {
      t1 = (-b + sqrt(discriminant)) / (2 * a);
      t2 = (-b - sqrt(discriminant)) / (2 * a);
      c1_center.x = dx * t2 + point_on_parallel.x;
      c1_center.y = dy * t2 + point_on_parallel.y;
    }
    else
    {
      c1_center.x = std::numeric_limits<double>::quiet_NaN();
      c1_center.y = std::numeric_limits<double>::quiet_NaN();
    }
  }
  else
  {
    c1_center.x = std::numeric_limits<double>::quiet_NaN();
    c1_center.y = std::numeric_limits<double>::quiet_NaN();
  }
  return c1_center;
}

pos AgribotPlannerROS::pCalculateTangentPoint(pos c1_center, double path_angle, double angle_difference,
                                              double arch_radius, double dead_zone)
{
  // determine the point of the intersection of
  // the circle C2 and the original path

  pos tangent_point;
  if (angle_difference < -dead_zone * DEG_TO_RAD)
  {
    double perpendicular = path_angle - M_PI / 2;
    tangent_point.x = c1_center.x + arch_radius * cos(perpendicular);
    tangent_point.y = c1_center.y + arch_radius * sin(perpendicular);
  }
  else if (angle_difference > dead_zone * DEG_TO_RAD)
  {
    double perpendicular = path_angle + M_PI / 2;
    tangent_point.x = c1_center.x + arch_radius * cos(perpendicular);
    tangent_point.y = c1_center.y + arch_radius * sin(perpendicular);
  }
  else
  {
    tangent_point.x = std::numeric_limits<double>::quiet_NaN();
    tangent_point.y = std::numeric_limits<double>::quiet_NaN();
  }
  return tangent_point;
}

std::vector<geometry_msgs::PoseStamped> AgribotPlannerROS::xAssemblePath(geometry_msgs::PoseStamped startPose,
                                                                         geometry_msgs::PoseStamped endPose,
                                                                         pos tangent_point, pos c1_center,
                                                                         pos c2_center)
{
  std::vector<geometry_msgs::PoseStamped> plan;

  pos startPoint = pFrameFromPoseStamped(startPose);
  pos endPoint = pFrameFromPoseStamped(endPose);

  double path_angle = dAngleBetweenFrames(endPoint, startPoint);
  double end_orientation = endPoint.az;
  double angle_difference = dLimitAngle((end_orientation - path_angle));

  // calculate angles and spans for archs
  double a_p1_ps = dAngleBetweenFrames(tangent_point, c1_center);
  double a_p1_p2 = dAngleBetweenFrames(c2_center, c1_center);
  double c1_span = dLimitAngle(a_p1_p2 - a_p1_ps);
  double a_p2_p1 = dAngleBetweenFrames(c1_center, c2_center);
  double a_p2_pk = dAngleBetweenFrames(endPoint, c2_center);
  double c2_span = a_p2_pk - a_p2_p1;

  // if the final orientation relative to path is positive
  // (robot turns left), circle C2 is followed counterclockwise
  // C2 is followed clockwise in right turn case
  if (angle_difference < 0)
  {
    if (c2_span > 0)
    {
      c2_span = fmod(c2_span - 360 * DEG_TO_RAD, 360 * DEG_TO_RAD);
    }
  }
  else
  {
    if (c2_span < 0)
    {
      c2_span = fmod(c2_span + 360 * DEG_TO_RAD, 360 * DEG_TO_RAD);
    }
  }

  if ((dDistanceBetweenFrames(startPoint, endPoint) < dDistanceBetweenFrames(tangent_point, endPoint)) ||
      isnan(c2_center.x) || isnan(c1_center.x) || isnan(tangent_point.x))
  {
    ROS_WARN_STREAM("[AgribotPlanner]\nDistance of path\t" << dDistanceBetweenFrames(startPoint, endPoint)
                                                           << "\nDistance needed\t\t"
                                                           << dDistanceBetweenFrames(tangent_point, endPoint));
    straightPath(plan, startPoint, endPoint);
  }
  else
  {
    straightPath(plan, startPoint, tangent_point);

    // appednd arch from circle C1
    arcPath(plan, tangent_point, c1_center, c1_span);

    // appednd arch from circle C2
    arcPath(plan, c1_center, c2_center, c2_span);
  }

  // // append final pose
  plan.push_back(endPose);

  // add proper orientation to each point
  for (int i = 0; i < plan.size(); i++)
  {
    plan[i].header.frame_id = startPose.header.frame_id;
    plan[i].header.seq = i;
    if (i != plan.size())
    {
      double angle = std::atan2(plan[i + 1].pose.position.y - plan[i].pose.position.y,
                                plan[i + 1].pose.position.x - plan[i].pose.position.x);
      plan[i].pose.orientation.x = 0;
      plan[i].pose.orientation.y = 0;
      plan[i].pose.orientation.z = sin(angle / 2.0);
      plan[i].pose.orientation.w = cos(angle / 2.0);
    }
  }

  return plan;
}

void AgribotPlannerROS::straightPath(std::vector<geometry_msgs::PoseStamped>& path, pos start_point, pos end_point)
{
  double step_size = 180 / (param_arch_step * M_PI * param_arch_radius);
  int steps = dDistanceBetweenFrames(start_point, end_point) * step_size;

  if (steps > 0)
  {
    for (int i = 0; i < steps; i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = start_point.x + static_cast<double>(i) / steps * (end_point.x - start_point.x);
      pose.pose.position.y = start_point.y + static_cast<double>(i) / steps * (end_point.y - start_point.y);
      path.push_back(pose);
    }
  }
}

void AgribotPlannerROS::arcPath(std::vector<geometry_msgs::PoseStamped>& path, pos circle_start, pos circle_center,
                                double angle_span)
{
  double angle_start_center = dAngleBetweenFrames(circle_start, circle_center);

  int steps = abs(angle_span * RAD_TO_DEG) / param_arch_step;
  double step_size = angle_span / steps;

  if (steps > 0)
  {
    for (int i = 0; i < steps; i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x =
          circle_center.x + param_arch_radius * cos(angle_start_center + static_cast<double>(i) * step_size);
      pose.pose.position.y =
          circle_center.y + param_arch_radius * sin(angle_start_center + static_cast<double>(i) * step_size);
      path.push_back(pose);
    }
  }
}

void AgribotPlannerROS::vChooseDeviationParallelnessParameters(int waypointNumber)
{
  // check if it is not the first waypoint in path
  if (waypointNumber > 1 && waypointNumber < _iPlanLength - 1)
  {
    // get yaws from poses
    geometry_msgs::PoseStamped currentPose = _xLocalPlan[waypointNumber];
    geometry_msgs::PoseStamped currentPoseTransformed;
    currentPoseTransformed = xTransformPose(currentPose);

    geometry_msgs::PoseStamped lastPose = _xLocalPlan[waypointNumber - 1];
    geometry_msgs::PoseStamped lastPoseTransformed;
    lastPoseTransformed = xTransformPose(lastPose);

    double current_waypoint_orientation = dGetYawFromQuaternion(currentPoseTransformed.pose.orientation);
    double last_waypoint_orientation = dGetYawFromQuaternion(lastPoseTransformed.pose.orientation);
    // check if there is difference between yaws
    if (abs(current_waypoint_orientation - last_waypoint_orientation) > param_epsilon)
    {
      // piece of path is an arch
      param_current_deviation_weight = param_deviation_weight_arch;
      param_current_parallelness_weight = param_parallelness_weight_arch;
    }
    else
    {
      // piece of path is a straight line
      param_current_deviation_weight = param_deviation_weight_straight;
      param_current_parallelness_weight = param_parallelness_weight_straight;
    }
  }
  else
  {
    param_current_deviation_weight = param_deviation_weight_straight;
    param_current_parallelness_weight = param_parallelness_weight_straight;
  }
}

double AgribotPlannerROS::dCloseToZero(double value)
{
  if (fabs(value) < param_epsilon)
  {
    return 0.0;
  }
  else
  {
    return value;
  }
}

bool AgribotPlannerROS::bMissedTooManyWaypointsCondition(int missedWaypoints)
{
  return missedWaypoints > param_missed_points_tollerance;
}

bool AgribotPlannerROS::bWaypointHitCondition(pos robotCenter, pos lastWaypoint, pos nextWaypoint)
{
  if (param_take_front)
  {
    return (bTraveledDistanceBetweenWaypoints(pCalculateRobotFrontPosition(robotCenter), lastWaypoint, nextWaypoint) &&
            bWithinGoalRange(pCalculateRobotFrontPosition(robotCenter), nextWaypoint)) ||
           (bTraveledDistanceBetweenWaypoints(robotCenter, lastWaypoint, nextWaypoint) &&
            bWithinGoalRange(robotCenter, nextWaypoint));
  }
  else
  {
    return (bTraveledDistanceBetweenWaypoints(robotCenter, lastWaypoint, nextWaypoint) &&
            bWithinGoalRange(robotCenter, nextWaypoint));
  }
}

bool AgribotPlannerROS::bWaypointMissedCondition(pos robotCenter, pos lastWaypoint, pos nextWaypoint)
{
  return bCenterTraveledBeyondGoal(robotCenter, lastWaypoint, nextWaypoint) &&
         bCenterOutOfGoalRange(robotCenter, nextWaypoint);
}

bool AgribotPlannerROS::bTraveledDistanceBetweenWaypoints(pos robotFront, pos lastWaypoint, pos nextWaypoint)
{
  double distanceWaypoints = dDistanceBetweenFrames(nextWaypoint, lastWaypoint);
  double traveledDistance = dDistanceBetweenFrames(lastWaypoint, robotFront);
  return (traveledDistance >= distanceWaypoints - param_goal_distance_tolerance);
}

bool AgribotPlannerROS::bWithinGoalRange(pos robotFront, pos nextWaypoint)
{
  double distanceToGoal = dDistanceBetweenFrames(nextWaypoint, robotFront);
  return (distanceToGoal <= param_goal_distance_tolerance);
}

bool AgribotPlannerROS::bIsLastWaypointInPath(int waypointsCount)
{
  return (waypointsCount >= (_xLocalPlan.size() - 1));
}

bool AgribotPlannerROS::bCenterTraveledBeyondGoal(pos robotCenter, pos lastWaypoint, pos nextWaypoint)
{
  double distanceWaypoints = dDistanceBetweenFrames(nextWaypoint, lastWaypoint);
  double traveledDistance = _dTraveledDistance - _dLastTraveledDistance;
  return (traveledDistance >= distanceWaypoints * param_missed_distance_coefficient);
}

bool AgribotPlannerROS::bCenterOutOfGoalRange(pos robotCenter, pos nextWaypoint)
{
  double distanceToGoal = dDistanceBetweenFrames(nextWaypoint, robotCenter);
  return (distanceToGoal > param_goal_distance_tolerance);
}

void AgribotPlannerROS::vTakeNextGoal(pos currentGoal, pos robotCenter)
{
  _iWaypointCount++;
  _dLastTraveledDistance = _dTraveledDistance;

  pos nextWaypoint = pGetFrame(_iWaypointCount);
  if (bIsLastWaypointInPath(_iWaypointCount + 1))
  {
    nextWaypoint.az = dAngleBetweenFrames(nextWaypoint, pGetFrame(_iWaypointCount - 1));
    if (param_take_front)
    {
      nextWaypoint = pCalculateRobotFrontPosition(nextWaypoint);
    }
  }

  if (dDistanceBetweenFrames(nextWaypoint, currentGoal) > param_chassis_length)
  {
    _pLastWaypoint = _xLocalPlan[_iWaypointCount - 1];
  }
  else
  {
    _pLastWaypoint = _pRobotCenter;
  }
}

void AgribotPlannerROS::vCalculateDeviationIntegral(pos LastWaypoint, pos nextWaypoint, pos robotCenter)
{
  double deviation = dCalculateDeviation(LastWaypoint, nextWaypoint, robotCenter);
  if (_dActualTime > 0)
  {
    if (!isnan(deviation))
    {
      _dLastTime = _dActualTime;
      _dActualTime = ros::Time::now().toSec();
      _dDeviationIntegral += fabs(deviation) * (_dActualTime - _dLastTime);
    }
  }
  else
  {
    _dActualTime = ros::Time::now().toSec();
  }
}

geometry_msgs::Twist AgribotPlannerROS::xFinishedPathBehaviour()
{
  _bGoalReached = true;
  _iPlanLength = 0;
  if (param_statistics)
  {
    _dStopTime = ros::Time::now().toSec();
    ROS_INFO("[AgribotPlanner] T = %f;l¸= %f;Er = %f;n = %d;x = %d", _dStopTime - _dStartTime, _dTraveledDistance,
             _dDeviationIntegral, _iWaypointCount, _iMissedWaypointsCount);
  }
  return xStopRobot();
}

bool AgribotPlannerROS::isGoalReached()
{
  if (!_bInitialized)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }
  return _bGoalReached;
}

void AgribotPlannerROS::vOdomCallback(const nav_msgs::Odometry odometry_msg)
{
  // prevent from too frequent odom reading
  double odom_time_stamp = odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nsec / 10e9;
  if (odom_time_stamp - _dLastOdomStamp > param_odom_filter_time)
  {
    _dLastOdomStamp = odom_time_stamp;
    pos lastRobotPosition = pFrameFromPoseStamped(_pRobotCenter);
    geometry_msgs::PoseStamped robotPose;

    robotPose.header = odometry_msg.header;
    robotPose.pose = odometry_msg.pose.pose;
    _pRobotCenter = xTransformPose(robotPose);

    _dTraveledDistance += dDistanceBetweenFrames(lastRobotPosition, pFrameFromPoseStamped(_pRobotCenter));
  }
}

pos AgribotPlannerROS::pCalculateRobotFrontPosition(pos robotCenter)
{
  pos frontPos;
  frontPos.x = robotCenter.x + cos(robotCenter.az) * param_chassis_length / 2;
  frontPos.y = robotCenter.y + sin(robotCenter.az) * param_chassis_length / 2;
  frontPos.az = robotCenter.az;
  return frontPos;
}

double AgribotPlannerROS::dGetYawFromQuaternion(geometry_msgs::Quaternion quat)
{
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::Twist AgribotPlannerROS::xSetVel(pos goal, pos nextWaypoint, pos lastWaypoint, pos robotCenter)
{
  geometry_msgs::Twist velocity;
  if (param_speed_limit)
  {
    velocity.linear.x = xLinearSpeedLimitedApproach(goal, lastWaypoint, robotCenter).linear.x;
  }
  else
  {
    velocity.linear.x = param_max_linear_vel;
  }

  velocity.angular.z = xDeviationApproach(nextWaypoint, lastWaypoint, robotCenter, velocity).angular.z;

  // Velocity limiter
  return xLimitVelocities(velocity);
}

geometry_msgs::Twist AgribotPlannerROS::xLimitVelocities(geometry_msgs::Twist Velcoities)
{
  if (Velcoities.linear.x > param_max_linear_vel)
  {
    Velcoities.linear.x = param_max_linear_vel;
  }
  if (Velcoities.linear.x < param_min_linear_vel)
  {
    Velcoities.linear.x = param_min_linear_vel;
  }
  if (Velcoities.angular.z > Velcoities.linear.x / param_min_radius)
  {
    Velcoities.angular.z = Velcoities.linear.x / param_min_radius;
  }
  if (Velcoities.angular.z < -Velcoities.linear.x / param_min_radius)
  {
    Velcoities.angular.z = -Velcoities.linear.x / param_min_radius;
  }
  return Velcoities;
}

geometry_msgs::Twist AgribotPlannerROS::xDeviationApproach(pos goal, pos lastWaypoint, pos robotCenter,
                                                           geometry_msgs::Twist velocity)
{
  double deviation = dCalculateDeviation(lastWaypoint, goal, robotCenter);

  double factorDeviation = param_current_deviation_weight * deviation / (1 + deviation);
  double factorParallelness =
      param_current_parallelness_weight * (1 - fabs(cos(dAngleBetweenFrames(goal, lastWaypoint) - robotCenter.az)));
  if (dLimitAngle(dAngleBetweenFrames(goal, robotCenter) - dAngleBetweenFrames(goal, lastWaypoint)) < 0)
  {
    factorDeviation *= -1;
  }
  if (dLimitAngle(dAngleBetweenFrames(goal, lastWaypoint) - robotCenter.az) < 0)
  {
    factorParallelness *= -1;
  }

  velocity.angular.z = (factorDeviation + factorParallelness) * param_max_linear_vel / param_min_radius;
  return velocity;
}

geometry_msgs::Twist AgribotPlannerROS::xLinearSpeedLimitedApproach(pos goal, pos lastWaypoint, pos robotCenter)
{
  geometry_msgs::Twist velocity;
  double distanceWaypoints = dDistanceBetweenFrames(goal, lastWaypoint);
  double distanceToNext = dDistanceBetweenFrames(goal, robotCenter);
  double waypointsDistanceFactor = distanceWaypoints / (1 + distanceWaypoints);
  double deviation = dCalculateDeviation(lastWaypoint, goal, robotCenter);
  double parallelnessFactor = fabs(cos(dAngleBetweenFrames(goal, lastWaypoint) - robotCenter.az));
  double deviationFactor = 1 / (1 + deviation);
  double distanceFactor = 1 - fabs((distanceWaypoints - distanceToNext) / distanceWaypoints);
  double multiplier = (1 - deviationFactor) * (1 - distanceFactor);
  velocity.linear.x =
      param_max_linear_vel * (deviationFactor * (1 - multiplier)) * waypointsDistanceFactor * parallelnessFactor;
  if (isnan(velocity.linear.x))
  {
    velocity.linear.x = 0.0;
  }
  return velocity;
}

double AgribotPlannerROS::dCalculateRadius(double distanceRobotWaypoint, double angleRobotWaypoint)
{
  return distanceRobotWaypoint / 2 / sin((angleRobotWaypoint));
}

geometry_msgs::Twist AgribotPlannerROS::xStopRobot()
{
  geometry_msgs::Twist velocity;
  velocity.linear.x = 0;
  velocity.linear.y = 0;
  velocity.angular.z = 0;
  return velocity;
}

pos AgribotPlannerROS::pGetFrame(int count)
{
  geometry_msgs::PoseStamped nextPose = _xLocalPlan[count];
  geometry_msgs::PoseStamped nextPoseTransformed;
  nextPoseTransformed = xTransformPose(nextPose);

  return pFrameFromPoseStamped(nextPoseTransformed);
}

pos AgribotPlannerROS::pGetWaypoint(int count)
{
  pos waypoint = pGetFrame(count);
  if (bIsLastWaypointInPath(count + 1))
  {
    if (count - 1 > 0)
    {
      waypoint.az = dAngleBetweenFrames(waypoint, pGetFrame(count - 1));
    }

    if (param_take_front)
    {
      waypoint = pCalculateRobotFrontPosition(waypoint);
    }
  }
  return waypoint;
}

geometry_msgs::PoseStamped AgribotPlannerROS::xTransformPose(geometry_msgs::PoseStamped ps)
{
  geometry_msgs::PoseStamped poseTransformed;
  poseTransformed.header.frame_id = "map";

  try
  {
    ps.header.stamp = ros::Time::now();
    _xTFBuffer->transform(ps, poseTransformed, "map", ros::Duration(0.2));
  }
  catch (tf2::ExtrapolationException e)
  {
    ROS_ERROR_STREAM("[LocalPlanner] ExtrapolationException occur");
    bAbortExecuting = true;
  }
  return poseTransformed;
}

pos AgribotPlannerROS::pFrameFromPoseStamped(geometry_msgs::PoseStamped ps)
{
  pos frame;
  frame.x = ps.pose.position.x;
  frame.y = ps.pose.position.y;
  frame.az = dGetYawFromQuaternion(ps.pose.orientation);
  return frame;
}

double AgribotPlannerROS::dLimitAngle(double angle)
{
  if (angle > 180 * DEG_TO_RAD)
  {
    angle -= 360 * DEG_TO_RAD;
  }
  if (angle < -180 * DEG_TO_RAD)
  {
    angle += 360 * DEG_TO_RAD;
  }
  return angle;
}

double AgribotPlannerROS::dAngleBetweenFrames(pos targetFrame, pos currentFrame)
{
  return std::atan2(targetFrame.y - currentFrame.y, targetFrame.x - currentFrame.x);
}

double AgribotPlannerROS::dDistanceBetweenFrames(pos targetFrame, pos currentFrame)
{
  return std::sqrt(std::pow(targetFrame.y - currentFrame.y, 2) + std::pow(targetFrame.x - currentFrame.x, 2));
}

double AgribotPlannerROS::dCalculateDeviation(pos lastWaypoint, pos nextWaypoint, pos robotCenter)
{
  if (dDistanceBetweenFrames(lastWaypoint, nextWaypoint) > 0)
  {
    pos projection = pCalculateProjection(lastWaypoint, nextWaypoint, robotCenter);
    return dDistanceBetweenFrames(robotCenter, projection);
  }
  else
  {
    return 0.0;
  }
}

pos AgribotPlannerROS::pCalculateProjection(pos lastWaypoint, pos nextWaypoint, pos robotCenter)
{
  pos projection;
  double a = dDistanceBetweenFrames(nextWaypoint, lastWaypoint);
  double b = dDistanceBetweenFrames(lastWaypoint, robotCenter);
  double c = dDistanceBetweenFrames(nextWaypoint, robotCenter);
  double dx = nextWaypoint.x - lastWaypoint.x;
  double dy = nextWaypoint.y - lastWaypoint.y;
  // taken from cosine law
  // cosine = (a*a + b*b - c*c) / (2*a*b)
  // a1 = b * cosine
  // ratio = a1 / a
  double ratio = (a * a + b * b - c * c) / (2 * a * a);
  projection.x = lastWaypoint.x + ratio * dx;
  projection.y = lastWaypoint.y + ratio * dy;
  projection.az = dAngleBetweenFrames(nextWaypoint, lastWaypoint);
  return projection;
}

pos AgribotPlannerROS::pCalculateGoalRange(pos nextWaypoint, pos robotCenter, double range)
{
  pos goalRange;
  double deltaX, deltaY;
  double angleRobotNextWaypoint = dLimitAngle(dAngleBetweenFrames(nextWaypoint, robotCenter));

  if (!bIsLastWaypointInPath(_iWaypointCount))
  {
    // Perpendicular to robot-next_frame line
    deltaX = sin(angleRobotNextWaypoint);
    deltaY = -cos(angleRobotNextWaypoint);
  }
  else
  {
    deltaX = sin(nextWaypoint.az);
    deltaY = -cos(nextWaypoint.az);
  }
  goalRange.x = deltaX * range;
  goalRange.y = deltaY * range;
  goalRange.az = dLimitAngle(dAngleBetweenFrames(nextWaypoint, goalRange));

  return goalRange;
}

void AgribotPlannerROS::vPathVisualization(pos robotCenter)
{
  _traveledPoints.header.frame_id = "map";
  _traveledPoints.header.stamp = ros::Time::now();
  _traveledPoints.ns = "travelled_path";
  _traveledPoints.action = visualization_msgs::Marker::ADD;
  _traveledPoints.id = 0;
  _traveledPoints.type = visualization_msgs::Marker::SPHERE_LIST;
  // _traveledPoints.scale.x = 0.1;
  _traveledPoints.scale.x = 2*param_arch_radius;
  _traveledPoints.scale.y = 2*param_arch_radius;
  _traveledPoints.scale.z = 0.01;
  _traveledPoints.color.r = 0.1;
  _traveledPoints.color.g = 0.1;
  _traveledPoints.color.b = 0.1;
  _traveledPoints.color.a = 0.5;
  _traveledPoints.pose.orientation.w = 1;
  geometry_msgs::Point p;
  p.x = robotCenter.x;
  p.y = robotCenter.y;
  p.z = 0.1;
  _traveledPoints.points.push_back(p);
  _rPathPub.publish(_traveledPoints);

  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "map";
  // marker.header.stamp = ros::Time::now();
  // marker.ns = "travelled_path";
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.id = 0;
  // marker.type = visualization_msgs::Marker::SPHERE;
  // marker.color.r = 0.1;
  // marker.color.g = 0.1;
  // marker.color.b = 0.1;
  // marker.color.a = 0.25;

  // geometry_msgs::Point p;
  // marker.scale.x = 2*param_arch_radius;
  // marker.scale.y = 2*param_arch_radius;
  // marker.scale.z = 0.01;
  // marker.pose.position.x = robotCenter.x;
  // marker.pose.position.y = robotCenter.y;
  // marker.pose.position.z = 0;
  // marker.pose.orientation.w = 1;
  // _rPathPub.publish(marker);
}

void AgribotPlannerROS::vPredictedPathVisualization(geometry_msgs::Twist velocity, pos robotCenter)
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
    // draw arch
    geometry_msgs::Point p, tmp;
    for (int i = 0; i < param_prediction_points_per_second * param_prediction_time; i++)
    {
      double theta = i * velocity.angular.z / param_prediction_points_per_second;
      tmp.x = radius * cos(theta - 90 * DEG_TO_RAD) - radius * cos(-90 * DEG_TO_RAD);
      tmp.y = radius * sin(theta - 90 * DEG_TO_RAD) - radius * sin(-90 * DEG_TO_RAD);

      p.x = robotCenter.x + tmp.x * cos(robotCenter.az) - tmp.y * sin(robotCenter.az);
      p.y = robotCenter.y + tmp.x * sin(robotCenter.az) + tmp.y * cos(robotCenter.az);

      p.z = 0.1;
      futurePoints.points.push_back(p);
    }
  }
  else
  {
    // draw straight line
    geometry_msgs::Point p;
    for (int i = 0; i < param_prediction_points_per_second * param_prediction_time; i++)
    {
      p.x = robotCenter.x + i * velocity.linear.x / param_prediction_points_per_second * cos(robotCenter.az);
      p.y = robotCenter.y + i * velocity.linear.x / param_prediction_points_per_second * sin(robotCenter.az);
      p.z = 0.1;
      futurePoints.points.push_back(p);
    }
  }
  _rPredictedPathPub.publish(futurePoints);
}

void AgribotPlannerROS::vNextGoalVisualization(pos lastWaypoint, pos goal)
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
  p.x = lastWaypoint.x;
  p.y = lastWaypoint.y;
  p.z = 0;
  marker.points.push_back(p);

  p.x = goal.x;
  p.y = goal.y;
  p.z = 0;
  marker.points.push_back(p);

  _rNextGoalPub.publish(marker);
}

void AgribotPlannerROS::vGoalRangeVisualization(pos goal, pos range)
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
  marker.scale.x = 2 * param_goal_distance_tolerance;
  marker.scale.y = 2 * param_goal_distance_tolerance;
  marker.scale.z = 0.01;
  marker.pose.position.x = goal.x;
  marker.pose.position.y = goal.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1;

  _rGoalRangePub.publish(marker);
}

void AgribotPlannerROS::vLocalPathVisualization(const std::vector<geometry_msgs::PoseStamped>& path)
{
  _xLocalPath.header.stamp = ros::Time::now();
  _xLocalPath.poses.clear();
  _xLocalPath.poses = path;
  _rLocalPathPub.publish(_xLocalPath);
}

void AgribotPlannerROS::vVisualizations(pos lastWaypoint, pos goal, pos nextWaypoint, pos robotCenter,
                                        geometry_msgs::Twist velocity)
{
  vNextGoalVisualization(lastWaypoint, goal);
  vGoalRangeVisualization(
      nextWaypoint, pCalculateGoalRange(nextWaypoint, robotCenter, param_goal_range * param_goal_distance_tolerance));
  //    vPathVisualization(robotCenter);
  vPredictedPathVisualization(velocity, robotCenter);
}
}  // namespace agribot_local_planner
