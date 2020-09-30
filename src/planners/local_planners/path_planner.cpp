#include "agribot_navigation/planners/local_planners/path_planner.h"

namespace agribot_local_planner
{
PathPlanner::PathPlanner()
{
  step_size = 1.0;
}

PathPlanner::~PathPlanner()
{
}

void PathPlanner::setStepSize(double param_step_size)
{
  step_size = param_step_size;
}

bool PathPlanner::designPath(std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::PoseStamped startPose,
                             geometry_msgs::PoseStamped endPose)
{
  ROS_INFO_STREAM("[PathPlanner] XD");
  header = endPose.header;
  return straightPath(path, pFrameFromPoseStamped(startPose), pFrameFromPoseStamped(endPose), step_size);
}

double PathPlanner::dGetYawFromQuaternion(geometry_msgs::Quaternion quat)
{
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double PathPlanner::dLimitAngle(double angle)
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

double PathPlanner::dAngleBetweenFrames(pos targetFrame, pos currentFrame)
{
  return std::atan2(targetFrame.y - currentFrame.y, targetFrame.x - currentFrame.x);
}

double PathPlanner::dDistanceBetweenFrames(pos targetFrame, pos currentFrame)
{
  return std::sqrt(std::pow(targetFrame.y - currentFrame.y, 2) + std::pow(targetFrame.x - currentFrame.x, 2));
}

pos PathPlanner::pFrameFromPoseStamped(geometry_msgs::PoseStamped ps)
{
  pos frame;
  frame.x = ps.pose.position.x;
  frame.y = ps.pose.position.y;
  frame.az = dGetYawFromQuaternion(ps.pose.orientation);
  return frame;
}

bool PathPlanner::straightPath(std::vector<geometry_msgs::PoseStamped>& path, pos start_point, pos end_point,
                               double step_size)
{
  int steps = dDistanceBetweenFrames(start_point, end_point) * step_size;

  if (steps > 0)
  {
    for (int i = 0; i < steps; i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = header.frame_id;
      pose.header.seq = path.size() + 1;
      pose.pose.position.x = start_point.x + static_cast<double>(i) / steps * (end_point.x - start_point.x);
      pose.pose.position.y = start_point.y + static_cast<double>(i) / steps * (end_point.y - start_point.y);
      path.push_back(pose);
    }
    return true;
  }
  else
  {
    return false;
  }
}

void PathPlanner::updateOrientation(std::vector<geometry_msgs::PoseStamped>& path)
{
  // add proper orientation to each point
  for (int i = 0; i < path.size(); i++)
  {
    if (i != path.size())
    {
      double angle = std::atan2(path[i + 1].pose.position.y - path[i].pose.position.y,
                                path[i + 1].pose.position.x - path[i].pose.position.x);
      path[i].pose.orientation.x = 0;
      path[i].pose.orientation.y = 0;
      path[i].pose.orientation.z = sin(angle / 2.0);
      path[i].pose.orientation.w = cos(angle / 2.0);
    }
  }
}
}