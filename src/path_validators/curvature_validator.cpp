/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */
#include "agribot_navigation/path_validators/curvature_validator.h"

namespace agribot_path_validators
{
CurvatureValidator::CurvatureValidator()
{
  max_curvature_ = 1.0 / default_min_radius;
  search_width_ = 1;
}
CurvatureValidator::~CurvatureValidator()
{
}
bool CurvatureValidator::validatePath(const std::vector<geometry_msgs::PoseStamped>& path)
{
  for (std::size_t i = search_width_; i << (path.size() - search_width_); i++)
  {
    double curvature = calculateMengerCurvature(path[i - search_width_], path[i], path[i + search_width_]);
    if (curvature > max_curvature_)
    {
      ROS_ERROR_STREAM("[CurvatureValidator] Path is not valid. C(path[" << i << "]) = " << curvature
                                                                         << " Max curvature is " << max_curvature_);
      return false;
    }
  }
  return true;
}
void CurvatureValidator::setMaxCurvature(const double curvature)
{
  max_curvature_ = curvature;
}
void CurvatureValidator::setSearchWidth(const std::size_t width)
{
  search_width_ = width;
}

bool CurvatureValidator::calculateMengerCurvature(const geometry_msgs::PoseStamped& previous_waypoint,
                                                  const geometry_msgs::PoseStamped& current_waypoint,
                                                  const geometry_msgs::PoseStamped& next_waypoint) const
{
  //   https://en.wikipedia.org/wiki/Menger_curvature
  agribot_local_planner::pos previous_point = CommonFunctions::pCoordinatesFromPoseStamped(previous_waypoint);
  agribot_local_planner::pos current_point = CommonFunctions::pCoordinatesFromPoseStamped(current_waypoint);
  agribot_local_planner::pos next_point = CommonFunctions::pCoordinatesFromPoseStamped(next_waypoint);
  double a = CommonFunctions::dDistanceBetweenCoordinates(previous_point, current_point);
  double b = CommonFunctions::dDistanceBetweenCoordinates(current_point, next_point);
  double c = CommonFunctions::dDistanceBetweenCoordinates(next_point, previous_point);

  // https://en.wikipedia.org/wiki/Heron%27s_formula
  double s = (a + b + c) / 2.0;
  double area = sqrt(s * (s - a) * (s - b) * (s - c));

  return (4 * area) / (a * b * c);
}
}  // namespace agribot_path_validators
