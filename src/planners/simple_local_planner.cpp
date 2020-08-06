/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/simple_local_planner.h"

#include <pluginlib/class_list_macros.h>
#include <cmath>

PLUGINLIB_EXPORT_CLASS(simple_local_planner::SimplePlannerROS, nav_core::BaseLocalPlanner)

namespace simple_local_planner
{
SimplePlannerROS::SimplePlannerROS() : _xCostmapROS(NULL), _bInitialized(false)
{
}

SimplePlannerROS::~SimplePlannerROS()
{
}

void SimplePlannerROS::initialize(std::string name, tf2_ros::Buffer* tfBuffer, costmap_2d::Costmap2DROS* costmapROS)
{
  if (!_bInitialized)
  {
    _xCostmapROS = costmapROS;
    _xTFBuffer = tfBuffer;
    _xTFListener = new tf2_ros::TransformListener(*tfBuffer);

    ros::NodeHandle gn;
    ros::NodeHandle pnh("~" + name);

    _rOdomSub = gn.subscribe("odom", 100, &SimplePlannerROS::vOdomCallback, this);
    _rPathPub = pnh.advertise<visualization_msgs::Marker>("travelled_path", 10);
    _rPredictedPathPub = pnh.advertise<visualization_msgs::Marker>("predicted_path", 10);
    _rGoalRangePub = pnh.advertise<visualization_msgs::Marker>("goal_range", 10);
    _rNextGoalPub = pnh.advertise<visualization_msgs::Marker>("next_goal", 10);

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

    pnh.param("arch_approach", param_arch_approach, false);
    pnh.param("speed_limit", param_speed_limit, true);
    pnh.param("debug", param_debug, false);
    pnh.param("statistics", param_statistics, true);
    pnh.param("take_front", param_take_front, false);

    _bInitialized = true;

    if (param_statistics)
    {
      _dStopTime = 0;
      _dStartTime = 0;
      _dLastTime = 0;
      _dActualTime = 0;
    }

    ROS_DEBUG_STREAM("[SimplePlanner] Plugin initialized.");
    gn.setParam("/agribot/simple_local_planer/is_inited", true);
  }
  else
  {
    ROS_WARN_STREAM("[SimplePlanner] This planner has already been "
                    << "initialized, doing nothing.");
  }
}

bool SimplePlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!_bInitialized)
  {
    ROS_ERROR_STREAM("[SimplePlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }
  bAbortExecuting = false;
  _iWaypointCount = 1;
  _iMissedWaypointsCount = 0;
  _xLocalPlan = orig_global_plan;
  _iPlanLength = _xLocalPlan.size();
  _bGoalReached = false;
  _traveledPoints = visualization_msgs::Marker();

  _pLastWaypoint = _pRobotCenter;

  if (param_statistics)
  {
    _dDeviationIntegral = 0;
    _dTraveledDistance = 0;

    _dStartTime = ros::Time::now().toSec();
    _dLastTime = 0;
    _dActualTime = 0;
  }
  ROS_INFO_STREAM("[SimplePlanner] Received new plan, "
                  << "plan length = " << _iPlanLength);
  return true;
}

bool SimplePlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  geometry_msgs::Twist velocity;
  pos goal;

  if (!_bInitialized)
  {
    ROS_ERROR_STREAM("[SimplePlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }
  if (_iPlanLength != 0)
  {
    // Check where robot is
    pos nextWaypoint = pGetWaypoint(_iWaypointCount);
    pos lastWaypoint = pFrameFromPoseStamped(xTransformPose(_pLastWaypoint));
    pos robotCenter = pFrameFromPoseStamped(xTransformPose(_pRobotCenter));

    goal = pCalculateGoal(nextWaypoint, robotCenter);

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
      ROS_ERROR_STREAM("[SimplePlanner] OUT OF PATH!");
      _iMissedWaypointsCount++;
    }

    // Next goal condition
    if (!bIsLastWaypointInPath(_iWaypointCount) && (bWaypointHitCondition(robotCenter, lastWaypoint, nextWaypoint) ||
                                                    bWaypointMissedCondition(robotCenter, lastWaypoint, nextWaypoint)))
    {
      if (param_statistics)
      {
        ROS_INFO_STREAM("[SimplePlanner] NEXT GOAL - Traveled " << _dTraveledDistance);
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
    ROS_ERROR("[SimplePlanner] Abort plan");
    return false;
  }
  else
  {
    cmd_vel = velocity;
    return true;
  }
}

void SimplePlannerROS::vChooseDeviationParallelnessParameters(int waypointNumber)
{
  // check if it is not the first waypoint in path
  if (waypointNumber > 0)
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

double SimplePlannerROS::dCloseToZero(double value)
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

bool SimplePlannerROS::bMissedTooManyWaypointsCondition(int missedWaypoints)
{
  return missedWaypoints > param_missed_points_tollerance;
}

bool SimplePlannerROS::bWaypointHitCondition(pos robotCenter, pos lastWaypoint, pos nextWaypoint)
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

bool SimplePlannerROS::bWaypointMissedCondition(pos robotCenter, pos lastWaypoint, pos nextWaypoint)
{
  return bCenterTraveledBeyondGoal(robotCenter, lastWaypoint, nextWaypoint) &&
         bCenterOutOfGoalRange(robotCenter, nextWaypoint);
}

bool SimplePlannerROS::bTraveledDistanceBetweenWaypoints(pos robotFront, pos lastWaypoint, pos nextWaypoint)
{
  double distanceWaypoints = dDistanceBetweenFrames(nextWaypoint, lastWaypoint);
  double traveledDistance = dDistanceBetweenFrames(lastWaypoint, robotFront);
  return (traveledDistance >= distanceWaypoints - param_goal_distance_tolerance);
}

bool SimplePlannerROS::bWithinGoalRange(pos robotFront, pos nextWaypoint)
{
  double distanceToGoal = dDistanceBetweenFrames(nextWaypoint, robotFront);
  return (distanceToGoal <= param_goal_distance_tolerance);
}

bool SimplePlannerROS::bIsLastWaypointInPath(int waypointsCount)
{
  return (waypointsCount >= (_iPlanLength - 1));
}

bool SimplePlannerROS::bCenterTraveledBeyondGoal(pos robotCenter, pos lastWaypoint, pos nextWaypoint)
{
  double distanceWaypoints = dDistanceBetweenFrames(nextWaypoint, lastWaypoint);
  double traveledDistance = dDistanceBetweenFrames(lastWaypoint, robotCenter);
  return (traveledDistance >= distanceWaypoints);
}

bool SimplePlannerROS::bCenterOutOfGoalRange(pos robotCenter, pos nextWaypoint)
{
  double distanceToGoal = dDistanceBetweenFrames(nextWaypoint, robotCenter);
  return (distanceToGoal > param_goal_distance_tolerance);
}

void SimplePlannerROS::vTakeNextGoal(pos currentGoal, pos robotCenter)
{
  _iWaypointCount++;

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

void SimplePlannerROS::vCalculateDeviationIntegral(pos LastWaypoint, pos nextWaypoint, pos robotCenter)
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

geometry_msgs::Twist SimplePlannerROS::xFinishedPathBehaviour()
{
  _bGoalReached = true;
  _iPlanLength = 0;
  if (param_statistics)
  {
    _dStopTime = ros::Time::now().toSec();
    ROS_INFO("[SimplePlanner] T = %f;l¸= %f;Er = %f;n = %d;x = %d", _dStopTime - _dStartTime, _dTraveledDistance,
             _dDeviationIntegral, _iWaypointCount, _iMissedWaypointsCount);
  }
  return xStopRobot();
}

bool SimplePlannerROS::isGoalReached()
{
  if (!_bInitialized)
  {
    ROS_ERROR_STREAM("[SimplePlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }
  return _bGoalReached;
}

void SimplePlannerROS::vOdomCallback(const nav_msgs::Odometry odometry_msg)
{
  pos lastRobotPosition = pFrameFromPoseStamped(_pRobotCenter);
  geometry_msgs::PoseStamped robotPose;

  robotPose.header = odometry_msg.header;
  robotPose.pose = odometry_msg.pose.pose;
  _pRobotCenter = xTransformPose(robotPose);

  if (param_statistics)
  {
    _dTraveledDistance += dDistanceBetweenFrames(lastRobotPosition, pFrameFromPoseStamped(_pRobotCenter));
  }
}

pos SimplePlannerROS::pCalculateRobotFrontPosition(pos robotCenter)
{
  pos frontPos;
  frontPos.x = robotCenter.x + cos(robotCenter.az) * param_chassis_length / 2;
  frontPos.y = robotCenter.y + sin(robotCenter.az) * param_chassis_length / 2;
  frontPos.az = robotCenter.az;
  return frontPos;
}

double SimplePlannerROS::dGetYawFromQuaternion(geometry_msgs::Quaternion quat)
{
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::Twist SimplePlannerROS::xSetVel(pos goal, pos nextWaypoint, pos lastWaypoint, pos robotCenter)
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

  if (param_arch_approach)
  {
    velocity.angular.z = xArchApproach(goal, robotCenter, velocity).angular.z;
  }
  else
  {
    velocity.angular.z = xDeviationApproach(nextWaypoint, lastWaypoint, robotCenter, velocity).angular.z;
  }
  // Velocity limiter
  return xLimitVelocities(velocity);
}

geometry_msgs::Twist SimplePlannerROS::xLimitVelocities(geometry_msgs::Twist Velcoities)
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

geometry_msgs::Twist SimplePlannerROS::xDeviationApproach(pos goal, pos lastWaypoint, pos robotCenter,
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

geometry_msgs::Twist SimplePlannerROS::xLinearSpeedLimitedApproach(pos goal, pos lastWaypoint, pos robotCenter)
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

geometry_msgs::Twist SimplePlannerROS::xArchApproach(pos goal, pos robotCenter, geometry_msgs::Twist velocity)
{
  double angleRobotGoal = dLimitAngle(dAngleBetweenFrames(goal, robotCenter) - robotCenter.az);
  double distanceRobotGoal = dDistanceBetweenFrames(goal, robotCenter);
  if (fabs(angleRobotGoal) > param_dead_zone * DEG_TO_RAD)
  {
    // Compute arch only outside of dead zone
    if (angleRobotGoal > 0)
    {
      if (angleRobotGoal < 180 * DEG_TO_RAD)
      {
        velocity.angular.z = velocity.linear.x / dCalculateRadius(distanceRobotGoal, angleRobotGoal);
      }
      else
      {
        velocity.angular.z = -2 * velocity.linear.x / param_chassis_length;
      }
    }
    else
    {
      if (angleRobotGoal > -180 * DEG_TO_RAD)
      {
        velocity.angular.z = velocity.linear.x / dCalculateRadius(distanceRobotGoal, angleRobotGoal);
      }
      else
      {
        velocity.angular.z = 2 * velocity.linear.x / param_chassis_length;
      }
    }
  }
  else
  {
    // Inside dead zone - straight line
    velocity.angular.z = 0.0;
  }
  return velocity;
}

double SimplePlannerROS::dCalculateRadius(double distanceRobotWaypoint, double angleRobotWaypoint)
{
  return distanceRobotWaypoint / 2 / sin((angleRobotWaypoint));
}

geometry_msgs::Twist SimplePlannerROS::xStopRobot()
{
  geometry_msgs::Twist velocity;
  velocity.linear.x = 0;
  velocity.linear.y = 0;
  velocity.angular.z = 0;
  return velocity;
}

pos SimplePlannerROS::pGetFrame(int count)
{
  geometry_msgs::PoseStamped nextPose = _xLocalPlan[count];
  geometry_msgs::PoseStamped nextPoseTransformed;
  nextPoseTransformed = xTransformPose(nextPose);

  return pFrameFromPoseStamped(nextPoseTransformed);
}

pos SimplePlannerROS::pGetWaypoint(int count)
{
  pos waypoint = pGetFrame(count);
  if (bIsLastWaypointInPath(count + 1))
  {
    waypoint.az = dAngleBetweenFrames(waypoint, pGetFrame(count - 1));
    if (param_take_front)
    {
      waypoint = pCalculateRobotFrontPosition(waypoint);
    }
  }
  return waypoint;
}

geometry_msgs::PoseStamped SimplePlannerROS::xTransformPose(geometry_msgs::PoseStamped ps)
{
  geometry_msgs::PoseStamped poseTransformed;
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

pos SimplePlannerROS::pFrameFromPoseStamped(geometry_msgs::PoseStamped ps)
{
  pos frame;
  frame.x = ps.pose.position.x;
  frame.y = ps.pose.position.y;
  frame.az = dGetYawFromQuaternion(ps.pose.orientation);
  return frame;
}

double SimplePlannerROS::dLimitAngle(double angle)
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

double SimplePlannerROS::dAngleBetweenFrames(pos targetFrame, pos currentFrame)
{
  return std::atan2(targetFrame.y - currentFrame.y, targetFrame.x - currentFrame.x);
}

double SimplePlannerROS::dDistanceBetweenFrames(pos targetFrame, pos currentFrame)
{
  return std::sqrt(std::pow(targetFrame.y - currentFrame.y, 2) + std::pow(targetFrame.x - currentFrame.x, 2));
}

double SimplePlannerROS::dCalculateDeviation(pos lastWaypoint, pos nextWaypoint, pos robotCenter)
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

pos SimplePlannerROS::pCalculateProjection(pos lastWaypoint, pos nextWaypoint, pos robotCenter)
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

pos SimplePlannerROS::pCalculateGoal(pos nextWaypoint, pos robotCenter)
{
  if (param_arch_approach)
  {
    pos goal_min, goal_max, goal;
    pos goalRange = pCalculateGoalRange(nextWaypoint, robotCenter, param_goal_range * param_goal_distance_tolerance);

    goal_min.x = nextWaypoint.x - goalRange.x;
    goal_min.y = nextWaypoint.y - goalRange.y;
    goal_max.x = nextWaypoint.x + goalRange.x;
    goal_max.y = nextWaypoint.y + goalRange.y;

    // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

    goal.x = (((robotCenter.x * pCalculateRobotFrontPosition(robotCenter).y -
                robotCenter.y * pCalculateRobotFrontPosition(robotCenter).x) *
               (goal_min.x - goal_max.x)) -
              ((robotCenter.x - pCalculateRobotFrontPosition(robotCenter).x) *
               (goal_min.x * goal_max.y - goal_min.y * goal_max.x))) /
             (((robotCenter.x - pCalculateRobotFrontPosition(robotCenter).x) * (goal_min.y - goal_max.y)) -
              ((robotCenter.y - pCalculateRobotFrontPosition(robotCenter).y) * (goal_min.x - goal_max.x)));

    goal.y = (((robotCenter.x * pCalculateRobotFrontPosition(robotCenter).y -
                robotCenter.y * pCalculateRobotFrontPosition(robotCenter).x) *
               (goal_min.y - goal_max.y)) -
              ((robotCenter.y - pCalculateRobotFrontPosition(robotCenter).y) *
               (goal_min.x * goal_max.y - goal_min.y * goal_max.x))) /
             (((robotCenter.x - pCalculateRobotFrontPosition(robotCenter).x) * (goal_min.y - goal_max.y)) -
              ((robotCenter.y - pCalculateRobotFrontPosition(robotCenter).y) * (goal_min.x - goal_max.x)));

    // Check if goal lays on the line segment
    // between goal_min and goal_max

    if (dDistanceBetweenFrames(goal, goal_min) > dDistanceBetweenFrames(goal_max, goal_min))
    {
      goal = goal_min;
    }
    else if (dDistanceBetweenFrames(goal, goal_max) > dDistanceBetweenFrames(goal_max, goal_min))
    {
      goal = goal_max;
    }

    goal.az = dLimitAngle(dAngleBetweenFrames(nextWaypoint, goal));

    return goal;
  }
  else
  {
    return nextWaypoint;
  }
}

pos SimplePlannerROS::pCalculateGoalRange(pos nextWaypoint, pos robotCenter, double range)
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

void SimplePlannerROS::vPathVisualization(pos robotCenter)
{
  _traveledPoints.header.frame_id = "map";
  _traveledPoints.header.stamp = ros::Time::now();
  _traveledPoints.ns = "travelled_path";
  _traveledPoints.action = visualization_msgs::Marker::ADD;
  _traveledPoints.id = 0;
  _traveledPoints.type = visualization_msgs::Marker::LINE_STRIP;
  _traveledPoints.scale.x = 0.1;
  _traveledPoints.color.b = 1.0;
  _traveledPoints.color.a = 1.0;
  _traveledPoints.pose.orientation.w = 1;
  geometry_msgs::Point p;
  p.x = robotCenter.x;
  p.y = robotCenter.y;
  p.z = 0.1;
  _traveledPoints.points.push_back(p);
  _rPathPub.publish(_traveledPoints);
}

void SimplePlannerROS::vPredictedPathVisualization(geometry_msgs::Twist velocity, pos robotCenter)
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

void SimplePlannerROS::vNextGoalVisualization(pos lastWaypoint, pos goal)
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

void SimplePlannerROS::vGoalRangeVisualization(pos goal, pos range)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "goal_range";
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 2;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.pose.orientation.w = 1;

  geometry_msgs::Point p;
  p.x = goal.x - range.x;
  p.y = goal.y - range.y;
  p.z = 0;

  marker.points.push_back(p);

  p.x = goal.x + range.x;
  p.y = goal.y + range.y;
  p.z = 0;

  marker.points.push_back(p);

  _rGoalRangePub.publish(marker);
}

void SimplePlannerROS::vVisualizations(pos lastWaypoint, pos goal, pos nextWaypoint, pos robotCenter,
                                       geometry_msgs::Twist velocity)
{
  vNextGoalVisualization(lastWaypoint, goal);
  vGoalRangeVisualization(
      nextWaypoint, pCalculateGoalRange(nextWaypoint, robotCenter, param_goal_range * param_goal_distance_tolerance));
  //    vPathVisualization(robotCenter);
  vPredictedPathVisualization(velocity, robotCenter);
}
}  // namespace simple_local_planner
