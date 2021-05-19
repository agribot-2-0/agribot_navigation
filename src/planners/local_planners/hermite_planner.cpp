/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/local_planners/hermite_planner.h"

constexpr double distance_power_coefficient = 0.25;
constexpr double tangent_coefficient = 1.8;

namespace agribot_local_planner
{
HermitePlanner::HermitePlanner() : ILocalPlanner()
{
  step_size = default_step_size;
  base_magnitude_start = default_hermite_magnitude;
  base_magnitude_end = default_hermite_magnitude;
}

HermitePlanner::HermitePlanner(const std::string& frame) : ILocalPlanner()
{
  step_size = default_step_size;
  base_magnitude_start = default_hermite_magnitude;
  base_magnitude_end = default_hermite_magnitude;
  setFrameId(frame);
}

HermitePlanner::~HermitePlanner()
{
}

bool HermitePlanner::designPath(std::vector<geometry_msgs::PoseStamped>& path,
                                const geometry_msgs::PoseStamped& start_pose,
                                const geometry_msgs::PoseStamped& end_pose)
{
  pos startPoint = CommonFunctions::pCoordinatesFromPoseStamped(start_pose);
  pos endPoint = CommonFunctions::pCoordinatesFromPoseStamped(end_pose);

  int path_start_index = path.size();

  if (!hermitePath(path, startPoint, endPoint, base_magnitude_start, base_magnitude_end))
  {
    ROS_ERROR_STREAM("[HermitePlanner] Unable to create proper path");
    return false;
  }

  path.push_back(end_pose);
  CommonFunctions::updateOrientationInPath(path, path_start_index, path.size() - 1);
  return true;
}

void HermitePlanner::addObstacle(const ObstacleDetection::Obstacle& obstacle)
{
}

/**
 * Magnitude is proportional to distance between start and end
 * and additionally proportional to difference
 * between path angle and orientation angle
 */
double HermitePlanner::calculateMagnitude(const pos& start, const pos& end, const double base_magnitude,
                                          const bool calculate_start) const
{
  double magnitude = base_magnitude;

  magnitude *= pow(CommonFunctions::dDistanceBetweenCoordinates(end, start), distance_power_coefficient);

  double path_angle = CommonFunctions::dAngleBetweenCoordinates(end, start);
  double angle_difference;
  if (calculate_start)
  {
    angle_difference = CommonFunctions::limitAngle((path_angle - start.az));
  }
  else
  {
    angle_difference = CommonFunctions::limitAngle((end.az - path_angle));
  }
  magnitude *= (M_PI - tangent_coefficient * atan(M_PI - angle_difference));
  return magnitude;
}

void HermitePlanner::setStepSize(const double param_step_size)
{
  step_size = param_step_size;
}

void HermitePlanner::setMagnitudeStart(const double param_magnitude)
{
  base_magnitude_start = param_magnitude;
}

void HermitePlanner::setMagnitudeEnd(const double param_magnitude)
{
  base_magnitude_end = param_magnitude;
}

bool HermitePlanner::hermitePath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                                 const pos& start_point, const pos& end_point, double m_start, double m_end) const
{
  pos point;
  pos last_point = start_point;
  double t_delta = 0.0001;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.header.seq = path.size() + 1;
  pose.pose.position.x = start_point.x;
  pose.pose.position.y = start_point.y;
  path.push_back(pose);

  pos start_derivative = calculateDerivatives(start_point.az, m_start);
  pos end_derivative = calculateDerivatives(end_point.az, m_end);

  for (double t = 0.0; t <= 1.0; t += t_delta)
  {
    double H1 = 2 * pow(t, 3) - 3 * pow(t, 2) + 1;
    double H2 = pow(t, 3) - 2 * pow(t, 2) + t;
    double H3 = -2 * pow(t, 3) + 3 * pow(t, 2);
    double H4 = pow(t, 3) - pow(t, 2);
    point.x = H1 * start_point.x + H2 * start_derivative.x + H3 * end_point.x + H4 * end_derivative.x;
    point.y = H1 * start_point.y + H2 * start_derivative.y + H3 * end_point.y + H4 * end_derivative.y;
    if (CommonFunctions::dDistanceBetweenCoordinates(last_point, point) >= step_size)
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
  if (abs(tan(angle)) < abs(CommonFunctions::cot(angle)))
  {
    derivative.x = magnitude;
    derivative.y = magnitude * tan(angle);
  }
  else
  {
    derivative.x = magnitude * CommonFunctions::cot(angle);
    derivative.y = magnitude;
  }

  // take negative derivatives for angles between 135 and -45 (315) degrees
  if (CommonFunctions::toDegrees(angle) < -45 || CommonFunctions::toDegrees(angle) > 135)
  {
    derivative.x *= -1;
    derivative.y *= -1;
  }
  return derivative;
}
}  // namespace agribot_local_planner
