/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/local_planners/hermite_planner.h"

constexpr double cot(double alfa)
{
  return 1.0 / tan(alfa);
}

namespace agribot_local_planner
{
HermitePlanner::HermitePlanner() : ILocalPlanner()
{
  step_size = default_step_size;
  magnitude_start = default_hermite_magnitude;
  magnitude_end = default_hermite_magnitude;
}

HermitePlanner::~HermitePlanner()
{
}

bool HermitePlanner::designPath(std::vector<geometry_msgs::PoseStamped>& path,
                                const geometry_msgs::PoseStamped startPose, const geometry_msgs::PoseStamped endPose)
{
  pos startPoint = pFrameFromPoseStamped(startPose);
  pos endPoint = pFrameFromPoseStamped(endPose);

  header = endPose.header;
  path_start_index = path.size();

  if (hermitePath(path, startPoint, endPoint, magnitude_start, magnitude_end))
  {
    path.push_back(endPose);
    updateOrientation(path, path_start_index, path.size() - 1);
    return true;
  }
  else
  {
    return false;
  }
}

void HermitePlanner::setStepSize(const double param_step_size)
{
  step_size = param_step_size;
}

void HermitePlanner::setMagnitudeStart(const double param_magnitude)
{
  magnitude_start = param_magnitude;
}

void HermitePlanner::setMagnitudeEnd(const double param_magnitude)
{
  magnitude_end = param_magnitude;
}

bool HermitePlanner::hermitePath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                                 const pos start_point, const pos end_point, double m_start, double m_end) const
{
  pos point;
  pos last_point = start_point;
  double t_delta = 0.0001;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = header.frame_id;
  pose.header.seq = path.size() + 1;
  pose.pose.position.x = start_point.x;
  pose.pose.position.y = start_point.y;
  path.push_back(pose);

  pos start_derivative = calculateDerivatives(start_point.az, m_start);
  pos end_derivative = calculateDerivatives(end_point.az, m_end);

  //  debug
  ROS_ERROR_STREAM("\nstart\t" << start_point.az * rad_to_deg << "\t" << start_derivative.x / start_derivative.y << "\t"
                               << tan(start_point.az) << "\nend\t" << end_point.az * rad_to_deg << "\t"
                               << end_derivative.x / end_derivative.y << "\t" << tan(end_point.az));

  for (double t = 0.0; t <= 1.0; t += t_delta)
  {
    double H1 = 2 * pow(t, 3) - 3 * pow(t, 2) + 1;
    double H2 = pow(t, 3) - 2 * pow(t, 2) + t;
    double H3 = -2 * pow(t, 3) + 3 * pow(t, 2);
    double H4 = pow(t, 3) - pow(t, 2);
    point.x = H1 * start_point.x + H2 * start_derivative.x + H3 * end_point.x + H4 * end_derivative.x;
    point.y = H1 * start_point.y + H2 * start_derivative.y + H3 * end_point.y + H4 * end_derivative.y;
    if (dDistanceBetweenFrames(last_point, point) >= step_size)
    {
      last_point = point;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      path.push_back(pose);
    }
  }

  return true;
}

pos HermitePlanner::calculateDerivatives(const double angle, const double magnitude) const
{
  pos derivative;
  if (abs(tan(angle)) < abs(cot(angle)))
  {
    derivative.x = magnitude;
    derivative.y = magnitude * tan(angle);
  }
  else
  {
    derivative.x = magnitude * cot(angle);
    derivative.y = magnitude;
  }
  return derivative;
}
}  // namespace agribot_local_planner
