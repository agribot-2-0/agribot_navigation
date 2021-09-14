/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_controllers/deviation_parallelness_controller.h"

namespace agribot_path_controllers
{
DeviationParallelnessController::DeviationParallelnessController(const std::string& frame) : IPathController()
{
  param_speed_limiting = true;
  param_current_parallelness_weight = default_parallelness_weight_straight;
  param_current_deviation_weight = default_deviation_weight_straight;
  param_min_radius = default_min_radius;
  param_max_linear_vel = default_max_linear_vel;
  param_min_linear_vel = default_min_linear_vel;
  param_epsilon = default_epsilon;
  frame_id_ = frame;
}

void DeviationParallelnessController::setSpeedLimiting(const bool limiting)
{
  param_speed_limiting = limiting;
}

void DeviationParallelnessController::setParallelnessWeight(const double parallelness_weight)
{
  param_current_parallelness_weight = parallelness_weight;
}

void DeviationParallelnessController::setDeviationWeight(const double deviation_weight)
{
  param_current_deviation_weight = deviation_weight;
}

void DeviationParallelnessController::setMinRadius(const double min_radius)
{
  param_min_radius = min_radius;
}

void DeviationParallelnessController::setMaxLinearVelocity(const double max_velocity)
{
  param_max_linear_vel = max_velocity;
}

void DeviationParallelnessController::setMinLinearVelocity(const double min_velocity)
{
  param_max_linear_vel = min_velocity;
}

void DeviationParallelnessController::setEpsilon(const double epsilon)
{
  param_epsilon = epsilon;
}

bool DeviationParallelnessController::computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
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

void DeviationParallelnessController::reset()
{
}

std::string DeviationParallelnessController::getFrameId()
{
  return frame_id_;
}

void DeviationParallelnessController::setVelocityLinear(const double velocity)
{
  param_max_linear_vel = velocity;
}

double DeviationParallelnessController::dCloseToZero(const double value)
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

void DeviationParallelnessController::xSetVel(geometry_msgs::Twist& velocity,
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

void DeviationParallelnessController::xLimitVelocities(geometry_msgs::Twist& velocities)
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

void DeviationParallelnessController::xDeviationApproach(geometry_msgs::Twist& velocity,
                                                         const agribot_local_planner::pos& next_waypoint,
                                                         const agribot_local_planner::pos& last_waypoint,
                                                         const agribot_local_planner::pos& robot_center)
{
  double deviation = calculateDeviation(last_waypoint, next_waypoint, robot_center);
  double factorDeviation = param_current_deviation_weight * deviation / (1 + deviation);

  double parallelness = calculateParallelness(last_waypoint, next_waypoint, robot_center);
  double factorParallelness = param_current_parallelness_weight * (1 - parallelness);

  if (CommonFunctions::limitAngle(CommonFunctions::dAngleBetweenCoordinates(next_waypoint, robot_center) -
                                  CommonFunctions::dAngleBetweenCoordinates(next_waypoint, last_waypoint)) < 0)
  {
    factorDeviation *= -1;
  }

  if (CommonFunctions::limitAngle(CommonFunctions::dAngleBetweenCoordinates(next_waypoint, last_waypoint) -
                                  robot_center.az) < 0)
  {
    factorParallelness *= -1;
  }

  velocity.angular.z = (factorDeviation + factorParallelness) * param_max_linear_vel / param_min_radius;
}

void DeviationParallelnessController::xLinearSpeedLimitedApproach(geometry_msgs::Twist& velocity,
                                                                  const agribot_local_planner::pos& next_waypoint,
                                                                  const agribot_local_planner::pos& last_waypoint,
                                                                  const agribot_local_planner::pos& robot_center)
{
  double distanceWaypoints =
      CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, last_waypoint);  // in range <0, inf)
  if (distanceWaypoints < param_epsilon)
  {
    velocity.linear.x = 0.0;
  }
  else
  {
    double distanceToNext =
        CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, robot_center);               // in range <0, inf)
    double distanceFactor = 1 - fabs((distanceWaypoints - distanceToNext) / distanceWaypoints);  // in range (-inf , 1>
    double waypointsDistanceFactor = distanceWaypoints / (1 + distanceWaypoints);                // in range (0, 1>

    double parallelnessFactor = calculateParallelness(last_waypoint, next_waypoint, robot_center);  // in range <0, 1>

    double deviation = calculateDeviation(last_waypoint, next_waypoint, robot_center);  // in range <0, inf>
    double deviationFactor = 1 / (1 + deviation);                                       // in range (0, 1>

    double multiplier = (1 - deviationFactor) * (1 - distanceFactor);  // in range <0, inf>

    velocity.linear.x =
        param_max_linear_vel * (deviationFactor * (1 - multiplier)) * waypointsDistanceFactor * parallelnessFactor;
  }
}

void DeviationParallelnessController::xStopRobot(geometry_msgs::Twist& velocity)
{
  velocity.linear.x = 0.0;
  velocity.linear.y = 0.0;
  velocity.angular.z = 0.0;
}

double DeviationParallelnessController::calculateDeviation(const agribot_local_planner::pos& last_waypoint,
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

double DeviationParallelnessController::calculateParallelness(const agribot_local_planner::pos& last_waypoint,
                                                              const agribot_local_planner::pos& next_waypoint,
                                                              const agribot_local_planner::pos& robot_center)
{
  return fabs(cos(CommonFunctions::dAngleBetweenCoordinates(next_waypoint, last_waypoint) - robot_center.az));
}

}  // namespace agribot_path_controllers
