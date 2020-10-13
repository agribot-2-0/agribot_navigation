/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <cmath>
#include <string>
#include <vector>

namespace agribot_path_trackers
{
struct pos
{
  double x, y, az;
};

class IPathTracker
{
public:
  virtual bool computeControl(geometry_msgs::Twist& cmd_vel, pos nextWaypoint,  // NOLINT(runtime/references)
                              pos lastWaypoint, pos robot_pose) = 0;

protected:
  static constexpr double deg_to_rad = (M_PI / 180);
  static constexpr double rad_to_deg = (180 / M_PI);

  double dGetYawFromQuaternion(const geometry_msgs::Quaternion& quat) const
  {
    tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }

  double dLimitAngle(double angle) const
  {
    if (angle > 180 * deg_to_rad)
    {
      angle -= 360 * deg_to_rad;
    }
    if (angle < -180 * deg_to_rad)
    {
      angle += 360 * deg_to_rad;
    }
    return angle;
  }

  double dAngleBetweenFrames(const pos& targetFrame, const pos& currentFrame) const
  {
    return std::atan2(targetFrame.y - currentFrame.y, targetFrame.x - currentFrame.x);
  }

  double dDistanceBetweenFrames(const pos& targetFrame, const pos& currentFrame) const
  {
    return std::sqrt(std::pow(targetFrame.y - currentFrame.y, 2) + std::pow(targetFrame.x - currentFrame.x, 2));
  }

  pos pFrameFromPoseStamped(const geometry_msgs::PoseStamped& ps) const
  {
    pos frame;
    frame.x = ps.pose.position.x;
    frame.y = ps.pose.position.y;
    frame.az = dGetYawFromQuaternion(ps.pose.orientation);
    return frame;
  }

  pos pCalculateProjection(pos lastWaypoint, pos nextWaypoint, pos robotCenter)
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
};
}  // namespace agribot_path_trackers
