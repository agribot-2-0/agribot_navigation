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
                                        pos nextWaypoint, pos lastWaypoint, pos robot_pose)
{
  geometry_msgs::Twist cmd_vel_tmp;
  double vf = 0.3;
  double r = 0.16;
  double l = 1.21;
  double k = 10;
  double alpha = dAngleBetweenFrames(nextWaypoint, robot_pose);
  double ld = dDistanceBetweenFrames(nextWaypoint, robot_pose);
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
