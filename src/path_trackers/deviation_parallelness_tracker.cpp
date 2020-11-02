/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_trackers/deviation_parallelness_tracker.h"

namespace agribot_path_trackers
{
DeviationParallelnessTracker::DeviationParallelnessTracker() : IPathTracker()
{
  param_epsilon = default_epsilon;
  param_max_linear_vel = default_max_linear_vel;
  param_min_linear_vel = default_min_linear_vel;
  param_min_radius = default_min_radius;
  param_current_deviation_weight = default_deviation_weight_straight;
  param_current_parallelness_weight = default_parallelness_weight_straight;
  param_speed_limiting = true;
}

DeviationParallelnessTracker::~DeviationParallelnessTracker()
{
}

void DeviationParallelnessTracker::setSpeedLimiting(bool limiting)
{
  param_speed_limiting = limiting;
}

void DeviationParallelnessTracker::setParallelnessWeight(double parallelness_weight)
{
  param_current_parallelness_weight = parallelness_weight;
}

void DeviationParallelnessTracker::setDeviationWeight(double deviation_weight)
{
  param_current_deviation_weight = deviation_weight;
}

void DeviationParallelnessTracker::setMinRadius(double min_radius)
{
  param_min_radius = min_radius;
}

void DeviationParallelnessTracker::setMaxLinearVelocity(double max_velocity)
{
  param_max_linear_vel = max_velocity;
}

void DeviationParallelnessTracker::setMinLinearVelocity(double min_velocity)
{
  param_max_linear_vel = min_velocity;
}

void DeviationParallelnessTracker::setEpsilon(double epsilon)
{
  param_epsilon = epsilon;
}

bool DeviationParallelnessTracker::computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                                                  const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                                                  const geometry_msgs::PoseStamped& robot_pose)
{
  if (path_to_track.size() < 2)
  {
    ROS_ERROR("[DeviationParallelnessController] Plan to track too short, cannot compute velocity");
    return false;
  }

  agribot_local_planner::pos next_waypoint = CommonFunctions::pCoordinatesFromPoseStamped(path_to_track.back());
  agribot_local_planner::pos last_waypoint = CommonFunctions::pCoordinatesFromPoseStamped(path_to_track.front());
  agribot_local_planner::pos robot_center = CommonFunctions::pCoordinatesFromPoseStamped(robot_pose);

  xSetVel(cmd_vel, next_waypoint, last_waypoint, robot_center);

  return true;
}

double DeviationParallelnessTracker::dCloseToZero(double value)
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

void DeviationParallelnessTracker::xSetVel(geometry_msgs::Twist& velocity,
                                           const agribot_local_planner::pos& next_waypoint,
                                           const agribot_local_planner::pos& last_waypoint,
                                           const agribot_local_planner::pos& robot_center)
{
  if (param_speed_limiting)
  {
    xLinearSpeedLimitedApproach(velocity, next_waypoint, last_waypoint, robot_center);
  }
  else
  {
    velocity.linear.x = param_max_linear_vel;
  }

  xDeviationApproach(velocity, next_waypoint, last_waypoint, robot_center);

  // Velocity limiter
  xLimitVelocities(velocity);
}

void DeviationParallelnessTracker::xLimitVelocities(geometry_msgs::Twist& velocities)
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
}

void DeviationParallelnessTracker::xDeviationApproach(geometry_msgs::Twist& velocity,
                                                      const agribot_local_planner::pos& next_waypoint,
                                                      const agribot_local_planner::pos& last_waypoint,
                                                      const agribot_local_planner::pos& robot_center)
{
  double deviation = dCalculateDeviation(last_waypoint, next_waypoint, robot_center);
  double factorDeviation = param_current_deviation_weight * deviation / (1 + deviation);

  double parallelness = dCalculateParallelness(last_waypoint, next_waypoint, robot_center);
  double factorParallelness = param_current_parallelness_weight * (1 - parallelness);

  if (CommonFunctions::dLimitAngle(CommonFunctions::dAngleBetweenCoordinates(next_waypoint, robot_center) -
                                   CommonFunctions::dAngleBetweenCoordinates(next_waypoint, last_waypoint)) < 0)
  {
    factorDeviation *= -1;
  }

  if (CommonFunctions::dLimitAngle(CommonFunctions::dAngleBetweenCoordinates(next_waypoint, last_waypoint) -
                                   robot_center.az) < 0)
  {
    factorParallelness *= -1;
  }

  velocity.angular.z = (factorDeviation + factorParallelness) * param_max_linear_vel / param_min_radius;
}

void DeviationParallelnessTracker::xLinearSpeedLimitedApproach(geometry_msgs::Twist& velocity,
                                                               const agribot_local_planner::pos& next_waypoint,
                                                               const agribot_local_planner::pos& last_waypoint,
                                                               const agribot_local_planner::pos& robot_center)
{
  double deviation = dCalculateDeviation(last_waypoint, next_waypoint, robot_center);
  double deviationFactor = 1 / (1 + deviation);

  double distanceWaypoints = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, last_waypoint);
  double distanceToNext = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, robot_center);
  double distanceFactor = 1 - fabs((distanceWaypoints - distanceToNext) / distanceWaypoints);
  double waypointsDistanceFactor = distanceWaypoints / (1 + distanceWaypoints);

  double parallelnessFactor = dCalculateParallelness(last_waypoint, next_waypoint, robot_center);

  double multiplier = (1 - deviationFactor) * (1 - distanceFactor);

  velocity.linear.x =
      param_max_linear_vel * (deviationFactor * (1 - multiplier)) * waypointsDistanceFactor * parallelnessFactor;
  if (isnan(velocity.linear.x))
  {
    velocity.linear.x = 0.0;
  }
}

void DeviationParallelnessTracker::xStopRobot(geometry_msgs::Twist& velocity)
{
  velocity.linear.x = 0.0;
  velocity.linear.y = 0.0;
  velocity.angular.z = 0.0;
}

double DeviationParallelnessTracker::dCalculateDeviation(const agribot_local_planner::pos& last_waypoint,
                                                         const agribot_local_planner::pos& next_waypoint,
                                                         const agribot_local_planner::pos& robot_center)
{
  if (CommonFunctions::dDistanceBetweenCoordinates(last_waypoint, next_waypoint) > 0)
  {
    agribot_local_planner::pos projection =
        CommonFunctions::pCalculateProjection(last_waypoint, next_waypoint, robot_center);
    return CommonFunctions::dDistanceBetweenCoordinates(robot_center, projection);
  }
  else
  {
    return 0.0;
  }
}

double DeviationParallelnessTracker::dCalculateParallelness(const agribot_local_planner::pos& last_waypoint,
                                                            const agribot_local_planner::pos& next_waypoint,
                                                            const agribot_local_planner::pos& robot_center)
{
  return fabs(cos(CommonFunctions::dAngleBetweenCoordinates(next_waypoint, last_waypoint) - robot_center.az));
}

}  // namespace agribot_path_trackers
