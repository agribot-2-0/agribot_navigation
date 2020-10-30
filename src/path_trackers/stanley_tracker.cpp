/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_trackers/stanley_tracker.h"

namespace agribot_path_trackers
{
StanleyTracker::StanleyTracker() : IPathTracker()
{
  double lin_vel_ = default_max_linear_vel;
  double error_gain_ = default_stanley_error_gain;
  double theta_limit_ = asin(default_chassis_length / 2 / default_min_radius);
}

StanleyTracker::~StanleyTracker()
{
}

void StanleyTracker::setVelocityLinear(const double velocity)
{
  lin_vel_ = velocity;
}

void StanleyTracker::setErrorGain(const double gain)
{
  error_gain_ = gain;
}

void StanleyTracker::setThetaLimit(const double limit)
{
  theta_limit_ = limit;
}

bool StanleyTracker::computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                                    const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                                    const geometry_msgs::PoseStamped& robot_pose)
{
  agribot_local_planner::pos next_waypoint = CommonFunctions::pCoordinatesFromPoseStamped(path_to_track[-1]);
  agribot_local_planner::pos last_waypoint = CommonFunctions::pCoordinatesFromPoseStamped(path_to_track[0]);
  agribot_local_planner::pos robot_center_base_footprint = CommonFunctions::pCoordinatesFromPoseStamped(robot_pose);

  agribot_local_planner::pos projection =
      CommonFunctions::pCalculateProjection(last_waypoint, next_waypoint, robot_center_base_footprint);
  double error = CommonFunctions::dDistanceBetweenCoordinates(robot_center_base_footprint, projection);
  double angle = projection.az - robot_center_base_footprint.az;
  double crosstrack_factor = atan2(error_gain_ * error, lin_vel_);
  if (angle < 0)
  {
    crosstrack_factor *= -1;
  }

  double theta = angle + crosstrack_factor;
  if (theta > theta_limit_)
  {
    theta = theta_limit_;
  }
  if (theta < -theta_limit_)
  {
    theta = -theta_limit_;
  }

  cmd_vel.linear.x = lin_vel_;
  cmd_vel.angular.z = sin(theta) * lin_vel_;

  return true;
}
}  // namespace agribot_path_trackers
