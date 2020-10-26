/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_trackers/pure_pursuit_tracker.h"

namespace agribot_path_trackers
{
PurePursuitTracker::PurePursuitTracker() : IPathTracker()
{
}

PurePursuitTracker::~PurePursuitTracker()
{
}

bool PurePursuitTracker::computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                                        const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                                        const geometry_msgs::PoseStamped& robot_pose)
{
  geometry_msgs::Twist cmd_vel_tmp;
  double vf = 0.3;
  double r = 0.16;
  double l = 1.21;
  double k = 10;
  uint rabbit_count = 0;

  agribot_local_planner::pos rabbit_pos = CommonFunctions::pCoordinatesFromPoseStamped(path_to_track[rabbit_count]);
  agribot_local_planner::pos robot_center_base_footprint = CommonFunctions::pCoordinatesFromPoseStamped(robot_pose);

  if (CommonFunctions::dDistanceBetweenCoordinates(rabbit_pos, robot_center_base_footprint) < 5.0)
  {
    rabbit_count++;
    if (rabbit_count >= path_to_track.size())
      rabbit_count = path_to_track.size() - 1;
  }
  double alpha = CommonFunctions::dAngleBetweenCoordinates(rabbit_pos, robot_center_base_footprint);

  double ld = CommonFunctions::dDistanceBetweenCoordinates(rabbit_pos, robot_center_base_footprint);
  double theta = asin(2 * sin(alpha) / k / vf);

  if (isnan(theta))
  {
    theta = 0.0;
  }

  cmd_vel.linear.x = vf;
  cmd_vel.angular.z = sin(theta) * vf;
  // ROS_INFO_STREAM("goal_pose x " << goal_pose.x << " goal_pose y " << goal_pose.y << " goal_pose az " <<
  // goal_pose.az); ROS_INFO_STREAM("robot_pose x " << robot_pose.x << " robot_pose y " << robot_pose.y << "
  // robot_pose
  // az "
  //                                 << robot_pose.az);
  ROS_INFO_STREAM("pure theta " << theta);
  ROS_INFO_STREAM("pure angular " << cmd_vel.angular.z);
  return true;
}
}  // namespace agribot_path_trackers
