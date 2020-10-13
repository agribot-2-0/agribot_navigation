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
  double vf = 0.5;
  double k = 0.1;
  pos projection = pCalculateProjection(lastWaypoint, nextWaypoint, robot_pose);
  // projection.az = dAngleBetweenFrames(goal_pose, lastWaypoint);
  // vNextGoalVisualization(projection);
  double error = dDistanceBetweenFrames(robot_pose, projection);
  double angle = projection.az - robot_pose.az;
  double crosstrack_factor = atan2(k * error, vf);
  if (angle < 0)
  {
    crosstrack_factor *= -1;
  }
  ROS_INFO_STREAM("stanley crosstrack error " << error);
  ROS_INFO_STREAM("stanley crosstrack factor " << crosstrack_factor);
  ROS_INFO_STREAM("stanley angle " << angle);
  double theta = angle + crosstrack_factor;
  if (theta > M_PI / 4)
  {
    theta = M_PI / 4;
  }
  if (theta < -M_PI / 4)
  {
    theta = -M_PI / 4;
  }
  cmd_vel.linear.x = vf;
  cmd_vel.angular.z = sin(theta) * vf;
  ROS_INFO_STREAM("stanley theta " << theta);
  ROS_INFO_STREAM("stanley angular " << cmd_vel.angular.z);
  return true;
}
}  // namespace agribot_path_trackers
