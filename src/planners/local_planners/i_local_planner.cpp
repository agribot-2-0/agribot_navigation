#include "agribot_navigation/planners/local_planners/i_local_planner.h"

namespace agribot_local_planner
{
ILocalPlaner::ILocalPlaner()
{
}

ILocalPlaner::~ILocalPlaner()
{
}

double ILocalPlaner::dGetYawFromQuaternion(const geometry_msgs::Quaternion quat) const
{
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double ILocalPlaner::dLimitAngle(double angle) const
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

double ILocalPlaner::dAngleBetweenFrames(const pos targetFrame, const pos currentFrame) const
{
  return std::atan2(targetFrame.y - currentFrame.y, targetFrame.x - currentFrame.x);
}

double ILocalPlaner::dDistanceBetweenFrames(const pos targetFrame, const pos currentFrame) const
{
  return std::sqrt(std::pow(targetFrame.y - currentFrame.y, 2) + std::pow(targetFrame.x - currentFrame.x, 2));
}

pos ILocalPlaner::pFrameFromPoseStamped(const geometry_msgs::PoseStamped ps) const
{
  pos frame;
  frame.x = ps.pose.position.x;
  frame.y = ps.pose.position.y;
  frame.az = dGetYawFromQuaternion(ps.pose.orientation);
  return frame;
}

void ILocalPlaner::updateOrientation(std::vector<geometry_msgs::PoseStamped>& path, const int start_index,
                                     const int end_index) const
{
  // add proper orientation to each point
  for (int i = start_index; i < end_index; i++)
  {
    double angle = std::atan2(path[i + 1].pose.position.y - path[i].pose.position.y,
                              path[i + 1].pose.position.x - path[i].pose.position.x);
    path[i].pose.orientation.x = 0;
    path[i].pose.orientation.y = 0;
    path[i].pose.orientation.z = sin(angle / 2.0);
    path[i].pose.orientation.w = cos(angle / 2.0);
  }
}
}  // namespace agribot_local_planner