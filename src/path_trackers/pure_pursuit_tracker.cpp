/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_trackers/pure_pursuit_tracker.h"

namespace agribot_path_trackers
{
PurePursuitTracker::PurePursuitTracker() : IPathTracker()
{
  lin_vel_ = default_max_linear_vel;
  curvature_ = default_pure_pursuit_curvature;
  rabbit_distance_ = default_pure_pursuit_rabbit_distance;
  rabbit_count_ = 0;
}

PurePursuitTracker::~PurePursuitTracker()
{
}

void PurePursuitTracker::setVelocityLinear(const double velocity)
{
  lin_vel_ = velocity;
}

void PurePursuitTracker::setCurvature(const double curvature)
{
  curvature_ = curvature;
}

void PurePursuitTracker::setRabbitDistance(const double distance)
{
  rabbit_distance_ = distance;
}

void PurePursuitTracker::resetRabbit()
{
  rabbit_count_ = 0;
}

bool PurePursuitTracker::computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                                        const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                                        const geometry_msgs::PoseStamped& robot_pose)
{
  agribot_local_planner::pos rabbit_pos = CommonFunctions::pCoordinatesFromPoseStamped(path_to_track[rabbit_count_]);
  agribot_local_planner::pos robot_center_base_footprint = CommonFunctions::pCoordinatesFromPoseStamped(robot_pose);

  computeRabbit(rabbit_pos, robot_center_base_footprint, path_to_track.size());

  double alpha = CommonFunctions::dAngleBetweenCoordinates(rabbit_pos, robot_center_base_footprint);

  double ld = CommonFunctions::dDistanceBetweenCoordinates(rabbit_pos, robot_center_base_footprint);
  double theta = asin(2 * sin(alpha) / curvature_ / lin_vel_);

  if (isnan(theta))
  {
    theta = 0.0;
  }

  cmd_vel.linear.x = lin_vel_;
  cmd_vel.angular.z = sin(theta) * lin_vel_;
  ROS_INFO_STREAM("pure theta " << theta);
  ROS_INFO_STREAM("pure angular " << cmd_vel.angular.z);
  return true;
}

void PurePursuitTracker::computeRabbit(const agribot_local_planner::pos& rabbit_pos,
                                       const agribot_local_planner::pos& robot_center, const int path_size)
{
  if (CommonFunctions::dDistanceBetweenCoordinates(rabbit_pos, robot_center) < rabbit_distance_)
  {
    rabbit_count_++;
    if (rabbit_count_ >= path_size)
      rabbit_count_ = path_size - 1;
  }
}
}  // namespace agribot_path_trackers
