/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/agribot_local_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(agribot_local_planner::AgribotPlannerROS, nav_core::BaseLocalPlanner)

namespace agribot_local_planner
{
AgribotPlannerROS::AgribotPlannerROS() : _costmap_ros(NULL), _initialized(false)
{
}

AgribotPlannerROS::~AgribotPlannerROS()
{
}

void AgribotPlannerROS::initialize(std::string name, tf2_ros::Buffer* tfBuffer, costmap_2d::Costmap2DROS* costmapROS)
{
  if (!_initialized)
  {
    _costmap_ros = costmapROS;
    _tf_buffer = tfBuffer;
    _tf_listener = new tf2_ros::TransformListener(*tfBuffer);

    ros::NodeHandle gn;
    ros::NodeHandle pnh("~" + name);

    _odom_sub = gn.subscribe("odom", 100, &AgribotPlannerROS::vOdomCallback, this);
    _path_pub = pnh.advertise<visualization_msgs::Marker>("travelled_path", 10);
    _predicted_path_pub = pnh.advertise<visualization_msgs::Marker>("predicted_path", 10);
    _goal_range_pub = pnh.advertise<visualization_msgs::Marker>("goal_range", 10);
    _next_goal_pub = pnh.advertise<visualization_msgs::Marker>("next_goal", 10);
    _local_path_pub = pnh.advertise<nav_msgs::Path>("local_path", 10);
    _local_path.header.frame_id = "map";

    pnh.param("goal_distance_tolerance", param_goal_distance_tolerance, default_goal_distance_tolerance);
    pnh.param("goal_range", param_goal_range, default_goal_range);
    pnh.param("missed_points_tolerance", param_missed_points_tolerance, default_missed_points_tolerance);

    pnh.param("chassis_length", param_chassis_length, default_chassis_length);
    pnh.param("min_radius", param_min_radius, default_min_radius);

    pnh.param("prediction_time", param_prediction_time, default_prediction_time);
    pnh.param("prediction_points_per_second", param_prediction_points_per_second, default_prediction_points_per_second);

    pnh.param("dead_zone", param_dead_zone, default_dead_zone);
    pnh.param("max_linear_vel", param_max_linear_vel, default_max_linear_vel);
    pnh.param("min_linear_vel", param_min_linear_vel, default_min_linear_vel);

    pnh.param("deviation_weight_straight", param_deviation_weight_straight, default_deviation_weight_straight);
    pnh.param("parallelness_weight_straight", param_parallelness_weight_straight, default_parallelness_weight_straight);
    pnh.param("deviation_weight_arc", param_deviation_weight_arc, default_deviation_weight_arc);
    pnh.param("parallelness_weight_arc", param_parallelness_weight_arc, default_parallelness_weight_arc);
    pnh.param("epsilon", param_epsilon, default_epsilon);

    pnh.param("design_dead_zone", param_design_dead_zone, default_design_dead_zone);
    pnh.param("design_arc_radius", param_design_arc_radius, default_design_arc_radius);
    pnh.param("design_step_size", param_design_step_size, default_step_size);
    pnh.param("hermite_magnitude", param_hermite_magnitude, default_hermite_magnitude);

    pnh.param("odom_filter_time", param_odom_filter_time, default_odom_filter_time);
    pnh.param("missed_distance_coefficient", param_missed_distance_coefficient, default_missed_distance_coefficient);

    // be sure that path planning arc radius is greater or equal to minimum turning radius
    if (param_design_arc_radius < param_min_radius)
    {
      ROS_WARN_STREAM("[AgribotLocalPlanner] param design_arc_radius to small. Taking param_min_radius = "
                      << param_min_radius << " instead");
      param_design_arc_radius = param_min_radius;
    }

    pnh.param("speed_limit", param_speed_limit, true);
    pnh.param("debug", param_debug, false);
    pnh.param("statistics", param_statistics, true);
    pnh.param("take_front", param_take_front, false);

    _robot_center.header.frame_id = "odom";
    _last_waypoint.header.frame_id = "odom";

    _initialized = true;

    if (param_statistics)
    {
      _stop_time = 0;
      _start_time = 0;
      _dlast_time = 0;
      _actual_time = 0;
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
  if (!_initialized)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }
  _abort_executing = false;
  _waypoint_count = 1;
  _missed_waypoints_count = 0;
  _plan = orig_global_plan;
  _local_plan.clear();

  CirclesPlanner circles_planner = CirclesPlanner(_robot_center.header.frame_id);
  circles_planner.setArcRadius(param_design_arc_radius);
  circles_planner.setDeadZone(param_design_dead_zone);
  circles_planner.setStepSize(param_design_step_size);

  HermitePlanner hermite_planner = HermitePlanner(_robot_center.header.frame_id);
  hermite_planner.setStepSize(param_design_step_size);
  hermite_planner.setMagnitudeStart(param_hermite_magnitude);
  hermite_planner.setMagnitudeEnd(param_hermite_magnitude);

  if (!xDesignLocalPath(_local_plan, _robot_center, _plan.back(), &circles_planner))
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Local planner unable to create path ");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("[AgribotPlanner] Local planner created, path length = " << _local_plan.size());
  }

  _goal_reached = false;
  _traveled_points = visualization_msgs::Marker();
  _last_odom_stamp = 0;

  _last_waypoint = _robot_center;
  _traveled_distance = 0;
  _last_traveled_distance = 0;

  if (param_statistics)
  {
    _deviation_integral = 0;

    _start_time = ros::Time::now().toSec();
    _dlast_time = 0;
    _actual_time = 0;
  }

  return true;
}

bool AgribotPlannerROS::xDesignLocalPath(std::vector<geometry_msgs::PoseStamped>& path,
                                         const geometry_msgs::PoseStamped& start_pose,
                                         const geometry_msgs::PoseStamped& end_pose, ILocalPlanner* const planner)
{
  return planner->designPath(path, start_pose, end_pose);
}

bool AgribotPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  geometry_msgs::Twist velocity;
  pos goal;

  vLocalPathVisualization(_local_plan);

  if (!_initialized)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }
  if (_local_plan.size() != 0)
  {
    // Check where robot is
    pos next_waypoint = pGetWaypoint(_waypoint_count);
    pos last_waypoint = CommonFunctions::pCoordinatesFromPoseStamped(xTransformPose(_last_waypoint));
    pos robot_center = CommonFunctions::pCoordinatesFromPoseStamped(xTransformPose(_robot_center));

    goal = next_waypoint;

    // Decide whether following straight or arc piece of path
    vChooseDeviationParallelnessParametres(_waypoint_count);

    // Deviation error for statistics
    if (param_statistics)
    {
      vCalculateDeviationIntegral(last_waypoint, next_waypoint, robot_center);
    }

    // Increment missed points number
    if (bWaypointMissedCondition(robot_center, last_waypoint, next_waypoint))
    {
      ROS_ERROR_STREAM("[AgribotPlanner] OUT OF PATH!");
      _missed_waypoints_count++;
    }

    // Next goal condition
    if (!bIsLastWaypointInPath(_waypoint_count) &&
        (bWaypointHitCondition(robot_center, last_waypoint, next_waypoint) ||
         bWaypointMissedCondition(robot_center, last_waypoint, next_waypoint)))
    {
      if (param_statistics)
      {
        ROS_INFO_STREAM("[AgribotPlanner] NEXT GOAL - Traveled " << _traveled_distance);
      }
      vTakeNextGoal(goal, robot_center);
      next_waypoint = pGetWaypoint(_waypoint_count);
    }

    // Calculate velocities
    velocity = xSetVel(goal, next_waypoint, last_waypoint, robot_center);

    velocity.linear.x = dCloseToZero(velocity.linear.x);
    velocity.linear.y = dCloseToZero(velocity.linear.y);
    velocity.linear.z = dCloseToZero(velocity.linear.z);
    velocity.angular.x = dCloseToZero(velocity.angular.x);
    velocity.angular.y = dCloseToZero(velocity.angular.y);
    velocity.angular.z = dCloseToZero(velocity.angular.z);

    // Stop condition
    if (bMissedTooManyWaypointsCondition(_missed_waypoints_count) ||
        (bIsLastWaypointInPath(_waypoint_count) && bWaypointHitCondition(robot_center, last_waypoint, next_waypoint)) ||
        (bIsLastWaypointInPath(_waypoint_count) &&
         bWaypointMissedCondition(robot_center, last_waypoint, next_waypoint)))
    {
      velocity = xFinishedPathBehaviour();
    }

    // Visualization
    vVisualizations(last_waypoint, goal, next_waypoint, robot_center, velocity);
  }
  if (_abort_executing)
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

void AgribotPlannerROS::vChooseDeviationParallelnessParametres(int waypoint_number)
{
  // check if it is not the first waypoint in path
  if (waypoint_number > 1 && waypoint_number < _local_plan.size() - 1)
  {
    // get yaws from poses
    geometry_msgs::PoseStamped currentPose = _local_plan[waypoint_number];
    geometry_msgs::PoseStamped currentPoseTransformed;
    currentPoseTransformed = xTransformPose(currentPose);

    geometry_msgs::PoseStamped lastPose = _local_plan[waypoint_number - 1];
    geometry_msgs::PoseStamped lastPoseTransformed;
    lastPoseTransformed = xTransformPose(lastPose);

    double current_waypoint_orientation =
        CommonFunctions::dGetYawFromQuaternion(currentPoseTransformed.pose.orientation);
    double last_waypoint_orientation = CommonFunctions::dGetYawFromQuaternion(lastPoseTransformed.pose.orientation);
    // check if there is difference between yaws
    if (abs(current_waypoint_orientation - last_waypoint_orientation) > param_epsilon)
    {
      // piece of path is an arc
      param_current_deviation_weight = param_deviation_weight_arc;
      param_current_parallelness_weight = param_parallelness_weight_arc;
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

bool AgribotPlannerROS::bMissedTooManyWaypointsCondition(int missed_waypoints)
{
  return missed_waypoints > param_missed_points_tolerance;
}

bool AgribotPlannerROS::bWaypointHitCondition(const pos& robot_center, const pos& last_waypoint,
                                              const pos& next_waypoint)
{
  if (param_take_front)
  {
    return (bTraveledDistanceBetweenWaypoints(pCalculateRobotFrontPosition(robot_center), last_waypoint,
                                              next_waypoint) &&
            bWithinGoalRange(pCalculateRobotFrontPosition(robot_center), next_waypoint)) ||
           (bTraveledDistanceBetweenWaypoints(robot_center, last_waypoint, next_waypoint) &&
            bWithinGoalRange(robot_center, next_waypoint));
  }
  else
  {
    return (bTraveledDistanceBetweenWaypoints(robot_center, last_waypoint, next_waypoint) &&
            bWithinGoalRange(robot_center, next_waypoint));
  }
}

bool AgribotPlannerROS::bWaypointMissedCondition(const pos& robot_center, const pos& last_waypoint,
                                                 const pos& next_waypoint)
{
  return bCenterTraveledBeyondGoal(robot_center, last_waypoint, next_waypoint) &&
         bCenterOutOfGoalRange(robot_center, next_waypoint);
}

bool AgribotPlannerROS::bTraveledDistanceBetweenWaypoints(pos robot_front, const pos& last_waypoint,
                                                          const pos& next_waypoint)
{
  double distanceWaypoints = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, last_waypoint);
  double traveledDistance = CommonFunctions::dDistanceBetweenCoordinates(last_waypoint, robot_front);
  return (traveledDistance >= distanceWaypoints - param_goal_distance_tolerance);
}

bool AgribotPlannerROS::bWithinGoalRange(pos robot_front, const pos& next_waypoint)
{
  double distanceToGoal = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, robot_front);
  return (distanceToGoal <= param_goal_distance_tolerance);
}

bool AgribotPlannerROS::bIsLastWaypointInPath(int waypoints_count)
{
  return (waypoints_count >= (_local_plan.size() - 1));
}

bool AgribotPlannerROS::bCenterTraveledBeyondGoal(const pos& robot_center, const pos& last_waypoint,
                                                  const pos& next_waypoint)
{
  double distanceWaypoints = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, last_waypoint);
  double traveledDistance = _traveled_distance - _last_traveled_distance;
  return (traveledDistance >= distanceWaypoints * param_missed_distance_coefficient);
}

bool AgribotPlannerROS::bCenterOutOfGoalRange(const pos& robot_center, const pos& next_waypoint)
{
  double distanceToGoal = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, robot_center);
  return (distanceToGoal > param_goal_distance_tolerance);
}

void AgribotPlannerROS::vTakeNextGoal(const pos& current_goal, const pos& robot_center)
{
  _waypoint_count++;
  _last_traveled_distance = _traveled_distance;

  pos next_waypoint = pGetFrame(_waypoint_count);
  if (bIsLastWaypointInPath(_waypoint_count + 1))
  {
    next_waypoint.az = CommonFunctions::dAngleBetweenCoordinates(next_waypoint, pGetFrame(_waypoint_count - 1));
    if (param_take_front)
    {
      next_waypoint = pCalculateRobotFrontPosition(next_waypoint);
    }
  }

  if (CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, current_goal) > param_chassis_length)
  {
    _last_waypoint = _local_plan[_waypoint_count - 1];
  }
  else
  {
    _last_waypoint = _robot_center;
  }
}

void AgribotPlannerROS::vCalculateDeviationIntegral(const pos& last_waypoint, const pos& next_waypoint,
                                                    const pos& robot_center)
{
  double deviation = dCalculateDeviation(last_waypoint, next_waypoint, robot_center);
  if (_actual_time > 0)
  {
    if (!isnan(deviation))
    {
      _dlast_time = _actual_time;
      _actual_time = ros::Time::now().toSec();
      _deviation_integral += fabs(deviation) * (_actual_time - _dlast_time);
    }
  }
  else
  {
    _actual_time = ros::Time::now().toSec();
  }
}

geometry_msgs::Twist AgribotPlannerROS::xFinishedPathBehaviour()
{
  _goal_reached = true;
  _local_plan.clear();
  if (param_statistics)
  {
    _stop_time = ros::Time::now().toSec();
    ROS_INFO("[AgribotPlanner] T = %f;l¸= %f;Er = %f;n = %d;x = %d", _stop_time - _start_time, _traveled_distance,
             _deviation_integral, _waypoint_count, _missed_waypoints_count);
  }
  return xStopRobot();
}

bool AgribotPlannerROS::isGoalReached()
{
  if (!_initialized)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }
  return _goal_reached;
}

void AgribotPlannerROS::vOdomCallback(const nav_msgs::Odometry& odometry_msg)
{
  // prevent from too frequent odom reading
  double odom_time_stamp = odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nsec / 10e9;
  if (odom_time_stamp - _last_odom_stamp > param_odom_filter_time)
  {
    _last_odom_stamp = odom_time_stamp;
    pos lastRobotPosition = CommonFunctions::pCoordinatesFromPoseStamped(_robot_center);
    geometry_msgs::PoseStamped robotPose;

    robotPose.header = odometry_msg.header;
    robotPose.pose = odometry_msg.pose.pose;
    _robot_center = xTransformPose(robotPose);

    _traveled_distance += CommonFunctions::dDistanceBetweenCoordinates(
        lastRobotPosition, CommonFunctions::pCoordinatesFromPoseStamped(_robot_center));
  }
}

pos AgribotPlannerROS::pCalculateRobotFrontPosition(const pos& robot_center)
{
  pos frontPos;
  frontPos.x = robot_center.x + cos(robot_center.az) * param_chassis_length / 2;
  frontPos.y = robot_center.y + sin(robot_center.az) * param_chassis_length / 2;
  frontPos.az = robot_center.az;
  return frontPos;
}

geometry_msgs::Twist AgribotPlannerROS::xSetVel(const pos& goal, const pos& next_waypoint, const pos& last_waypoint,
                                                const pos& robot_center)
{
  geometry_msgs::Twist velocity;
  if (param_speed_limit)
  {
    velocity.linear.x = xLinearSpeedLimitedApproach(goal, last_waypoint, robot_center).linear.x;
  }
  else
  {
    velocity.linear.x = param_max_linear_vel;
  }

  velocity.angular.z = xDeviationApproach(next_waypoint, last_waypoint, robot_center, velocity).angular.z;

  // Velocity limiter
  return xLimitVelocities(velocity);
}

geometry_msgs::Twist AgribotPlannerROS::xLimitVelocities(geometry_msgs::Twist velocities)
{
  if (velocities.linear.x > param_max_linear_vel)
  {
    velocities.linear.x = param_max_linear_vel;
  }
  if (velocities.linear.x < param_min_linear_vel)
  {
    velocities.linear.x = param_min_linear_vel;
  }
  if (velocities.angular.z > velocities.linear.x / param_min_radius)
  {
    velocities.angular.z = velocities.linear.x / param_min_radius;
  }
  if (velocities.angular.z < -velocities.linear.x / param_min_radius)
  {
    velocities.angular.z = -velocities.linear.x / param_min_radius;
  }
  return velocities;
}

geometry_msgs::Twist AgribotPlannerROS::xDeviationApproach(const pos& goal, const pos& last_waypoint,
                                                           const pos& robot_center, geometry_msgs::Twist velocity)
{
  double deviation = dCalculateDeviation(last_waypoint, goal, robot_center);

  double factorDeviation = param_current_deviation_weight * deviation / (1 + deviation);
  double factorParallelness =
      param_current_parallelness_weight *
      (1 - fabs(cos(CommonFunctions::dAngleBetweenCoordinates(goal, last_waypoint) - robot_center.az)));
  if (CommonFunctions::dLimitAngle(CommonFunctions::dAngleBetweenCoordinates(goal, robot_center) -
                                   CommonFunctions::dAngleBetweenCoordinates(goal, last_waypoint)) < 0)
  {
    factorDeviation *= -1;
  }
  if (CommonFunctions::dLimitAngle(CommonFunctions::dAngleBetweenCoordinates(goal, last_waypoint) - robot_center.az) <
      0)
  {
    factorParallelness *= -1;
  }

  velocity.angular.z = (factorDeviation + factorParallelness) * param_max_linear_vel / param_min_radius;
  return velocity;
}

geometry_msgs::Twist AgribotPlannerROS::xLinearSpeedLimitedApproach(const pos& goal, const pos& last_waypoint,
                                                                    const pos& robot_center)
{
  geometry_msgs::Twist velocity;
  double distanceWaypoints = CommonFunctions::dDistanceBetweenCoordinates(goal, last_waypoint);
  double distanceToNext = CommonFunctions::dDistanceBetweenCoordinates(goal, robot_center);
  double waypointsDistanceFactor = distanceWaypoints / (1 + distanceWaypoints);
  double deviation = dCalculateDeviation(last_waypoint, goal, robot_center);
  double parallelnessFactor =
      fabs(cos(CommonFunctions::dAngleBetweenCoordinates(goal, last_waypoint) - robot_center.az));
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

double AgribotPlannerROS::dCalculateRadius(double distance_robot_waypoint, double angle_robot_waypoint)
{
  return distance_robot_waypoint / 2 / sin((angle_robot_waypoint));
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
  geometry_msgs::PoseStamped nextPose = _local_plan[count];
  geometry_msgs::PoseStamped nextPoseTransformed;
  nextPoseTransformed = xTransformPose(nextPose);

  return CommonFunctions::pCoordinatesFromPoseStamped(nextPoseTransformed);
}

pos AgribotPlannerROS::pGetWaypoint(int count)
{
  pos waypoint = pGetFrame(count);
  if (bIsLastWaypointInPath(count + 1))
  {
    if (count - 1 > 0)
    {
      waypoint.az = CommonFunctions::dAngleBetweenCoordinates(waypoint, pGetFrame(count - 1));
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
    _tf_buffer->transform(ps, poseTransformed, "map", ros::Duration(0.2));
  }
  catch (tf2::ExtrapolationException e)
  {
    ROS_ERROR_STREAM("[LocalPlanner] ExtrapolationException occur");
    _abort_executing = true;
  }
  return poseTransformed;
}

double AgribotPlannerROS::dCalculateDeviation(const pos& last_waypoint, const pos& next_waypoint,
                                              const pos& robot_center)
{
  if (CommonFunctions::dDistanceBetweenCoordinates(last_waypoint, next_waypoint) > 0)
  {
    pos projection = pCalculateProjection(last_waypoint, next_waypoint, robot_center);
    return CommonFunctions::dDistanceBetweenCoordinates(robot_center, projection);
  }
  else
  {
    return 0.0;
  }
}

pos AgribotPlannerROS::pCalculateProjection(const pos& last_waypoint, const pos& next_waypoint, const pos& robot_center)
{
  pos projection;
  double a = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, last_waypoint);
  double b = CommonFunctions::dDistanceBetweenCoordinates(last_waypoint, robot_center);
  double c = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, robot_center);
  double dx = next_waypoint.x - last_waypoint.x;
  double dy = next_waypoint.y - last_waypoint.y;
  // taken from cosine law
  // cosine = (a*a + b*b - c*c) / (2*a*b)
  // a1 = b * cosine
  // ratio = a1 / a
  double ratio = (a * a + b * b - c * c) / (2 * a * a);
  projection.x = last_waypoint.x + ratio * dx;
  projection.y = last_waypoint.y + ratio * dy;
  projection.az = CommonFunctions::dAngleBetweenCoordinates(next_waypoint, last_waypoint);
  return projection;
}

pos AgribotPlannerROS::pCalculateGoalRange(const pos& next_waypoint, const pos& robot_center, double range)
{
  pos goalRange;
  double deltaX, deltaY;
  double angleRobotNextWaypoint =
      CommonFunctions::dLimitAngle(CommonFunctions::dAngleBetweenCoordinates(next_waypoint, robot_center));

  if (!bIsLastWaypointInPath(_waypoint_count))
  {
    // Perpendicular to robot-next_frame line
    deltaX = sin(angleRobotNextWaypoint);
    deltaY = -cos(angleRobotNextWaypoint);
  }
  else
  {
    deltaX = sin(next_waypoint.az);
    deltaY = -cos(next_waypoint.az);
  }
  goalRange.x = deltaX * range;
  goalRange.y = deltaY * range;
  goalRange.az = CommonFunctions::dLimitAngle(CommonFunctions::dAngleBetweenCoordinates(next_waypoint, goalRange));

  return goalRange;
}

void AgribotPlannerROS::vPathVisualization(const pos& robot_center)
{
  _traveled_points.header.frame_id = "map";
  _traveled_points.header.stamp = ros::Time::now();
  _traveled_points.ns = "travelled_path";
  _traveled_points.action = visualization_msgs::Marker::ADD;
  _traveled_points.id = 0;
  _traveled_points.type = visualization_msgs::Marker::LINE_STRIP;
  _traveled_points.scale.x = 0.1;
  _traveled_points.color.b = 1.0;
  _traveled_points.color.a = 1.0;
  _traveled_points.pose.orientation.w = 1;
  geometry_msgs::Point p;
  p.x = robot_center.x;
  p.y = robot_center.y;
  p.z = 0.1;
  _traveled_points.points.push_back(p);
  _path_pub.publish(_traveled_points);
}

void AgribotPlannerROS::vPredictedPathVisualization(const geometry_msgs::Twist& velocity, const pos& robot_center)
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
    for (int i = 0; i < param_prediction_points_per_second * param_prediction_time; i++)
    {
      double theta = i * velocity.angular.z / param_prediction_points_per_second;
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
    for (int i = 0; i < param_prediction_points_per_second * param_prediction_time; i++)
    {
      p.x = robot_center.x + i * velocity.linear.x / param_prediction_points_per_second * cos(robot_center.az);
      p.y = robot_center.y + i * velocity.linear.x / param_prediction_points_per_second * sin(robot_center.az);
      p.z = 0.1;
      futurePoints.points.push_back(p);
    }
  }
  _predicted_path_pub.publish(futurePoints);
}

void AgribotPlannerROS::vNextGoalVisualization(const pos& last_waypoint, const pos& goal)
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

  _next_goal_pub.publish(marker);
}

void AgribotPlannerROS::vGoalRangeVisualization(const pos& goal, const pos& range)
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

  _goal_range_pub.publish(marker);
}

void AgribotPlannerROS::vLocalPathVisualization(const std::vector<geometry_msgs::PoseStamped>& path)
{
  _local_path.header.stamp = ros::Time::now();
  _local_path.poses.clear();
  _local_path.poses = path;
  _local_path_pub.publish(_local_path);
}

void AgribotPlannerROS::vVisualizations(const pos& last_waypoint, const pos& goal, const pos& next_waypoint,
                                        const pos& robot_center, const geometry_msgs::Twist& velocity)
{
  vNextGoalVisualization(last_waypoint, goal);
  vGoalRangeVisualization(next_waypoint, pCalculateGoalRange(next_waypoint, robot_center,
                                                             param_goal_range * param_goal_distance_tolerance));
  //    vPathVisualization(robot_center);
  vPredictedPathVisualization(velocity, robot_center);
}
}  // namespace agribot_local_planner
