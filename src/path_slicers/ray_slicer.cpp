/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_slicers/ray_slicer.h"

namespace agribot_path_slicers
{
RaySlicer::RaySlicer()
{
  ray_length_ = 0;
  ray_width_ = 0;
  reset();
}
RaySlicer::~RaySlicer()
{
}

void RaySlicer::reset()
{
  search_start_index_ = 0;
}

bool RaySlicer::findPathSlice(std::vector<geometry_msgs::PoseStamped>& path_slice,  // NOLINT(runtime/references)
                              const std::vector<geometry_msgs::PoseStamped>& local_path,
                              const geometry_msgs::PoseStamped& robot_pose)
{
  path_slice.clear();
  std::size_t* start_index = nullptr;

  //   find index of start of segment covered by ray
  for (auto waypoint = local_path.cbegin() + search_start_index_; waypoint != local_path.cend(); ++waypoint)
  {
    if (checkWaypoint(robot_pose, (*waypoint)))
    {
      *start_index = std::distance(local_path.cbegin(), waypoint);
    }
  }

  if (start_index)
  {
    // remember index of start of the found segment
    search_start_index_ = *start_index;

    // iterate to push whole segment covered by a ray;
    auto waypoint = local_path.cbegin() + *start_index;
    while (checkWaypoint(robot_pose, *waypoint) && waypoint != local_path.cend())
    {
      path_slice.push_back(*waypoint);
      ++waypoint;
    }
  }

  // not enough waypoints found
  if (path_slice.size() < 2)
  {
    return false;
  }

  return true;
}

void RaySlicer::setRayLength(const double length)
{
  ray_length_ = length;
}
void RaySlicer::setRayWidth(const double width)
{
  ray_width_ = width;
}

bool RaySlicer::checkWaypoint(const geometry_msgs::PoseStamped& robot_pose,
                              const geometry_msgs::PoseStamped& waypoint_pose) const
{
  agribot_local_planner::pos waypoint = CommonFunctions::pCoordinatesFromPoseStamped(waypoint_pose);
  agribot_local_planner::pos robot_center = CommonFunctions::pCoordinatesFromPoseStamped(robot_pose);
  return (withinRayDistance(robot_center, waypoint) && withinRayAngle(robot_center, waypoint));
}

bool RaySlicer::withinRayDistance(const agribot_local_planner::pos& robot_center,
                                  const agribot_local_planner::pos& waypoint) const
{
  return (CommonFunctions::dDistanceBetweenCoordinates(waypoint, robot_center) <= ray_length_);
}

bool RaySlicer::withinRayAngle(const agribot_local_planner::pos& robot_center,
                               const agribot_local_planner::pos& waypoint) const
{
  double left_ray_angle = CommonFunctions::dLimitAngle(robot_center.az + M_PI_2);
  double right_ray_angle = CommonFunctions::dLimitAngle(robot_center.az - M_PI_2);
  double robot_waypoint_angle = CommonFunctions::dAngleBetweenCoordinates(waypoint, robot_center);
  return ((calculateLinesSpan(robot_waypoint_angle, left_ray_angle) <= ray_width_ / 2.0) ||
          (calculateLinesSpan(robot_waypoint_angle, right_ray_angle) <= ray_width_ / 2.0));
}

bool RaySlicer::calculateLinesSpan(const double target_line_angle, const double base_line_angle) const
{
  return abs(CommonFunctions::dLimitAngle(target_line_angle - base_line_angle));
}
}  // namespace agribot_path_slicers
