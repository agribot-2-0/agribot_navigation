/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_trackers/ray_tracker.h"

namespace agribot_path_trackers
{
RayTracker::RayTracker()
{
  ray_length_ = default_tracker_side_ray_length;
  ray_width_ = default_tracker_side_ray_width;
  side_tracking_ = true;
  reset();
}
RayTracker::~RayTracker()
{
}

void RayTracker::reset()
{
}

bool RayTracker::findPathSlice(std::vector<geometry_msgs::PoseStamped>& path_slice,  // NOLINT(runtime/references)
                               const std::vector<geometry_msgs::PoseStamped>& local_path,
                               const geometry_msgs::PoseStamped& robot_pose)
{
  agribot_local_planner::pos robot_coordinates = CommonFunctions::pCoordinatesFromPoseStamped(robot_pose);
  path_slice.clear();
  std::size_t start_index;
  bool start_index_found = false;
  double distance_to_first_waypoint = DBL_MAX;
  //   find index of start of segment covered by ray
  for (auto waypoint = local_path.cbegin(); waypoint != local_path.cend(); ++waypoint)
  {
    agribot_local_planner::pos waypoint_coordinates = CommonFunctions::pCoordinatesFromPoseStamped(*waypoint);
    if (!start_index_found && checkWaypoint(robot_coordinates, waypoint_coordinates))
    {
      start_index_found = true;
      double distance = CommonFunctions::dDistanceBetweenCoordinates(waypoint_coordinates, robot_coordinates);
      if (distance < distance_to_first_waypoint)
      {
        start_index = std::distance(local_path.cbegin(), waypoint);
        distance_to_first_waypoint = distance;
      }
    }
  }

  if (start_index_found)
  {
    // iterate to push whole segment covered by a ray;
    auto waypoint = local_path.cbegin() + start_index;
    while (checkWaypoint(robot_coordinates, CommonFunctions::pCoordinatesFromPoseStamped(*waypoint)) &&
           waypoint != local_path.cend())
    {
      // ROS_ERROR_STREAM("\tWAYPOINT NO " << std::distance(local_path.cbegin(), waypoint));
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

void RayTracker::setRayLength(const double length)
{
  ray_length_ = length;
}
void RayTracker::setRayWidth(const double width)
{
  ray_width_ = width;
}
void RayTracker::setTrackingMethod(const bool side_tracking)
{
  side_tracking_ = side_tracking;
}

bool RayTracker::checkWaypoint(const agribot_local_planner::pos& robot_coordinates,
                               const agribot_local_planner::pos& waypoint_coordinates) const
{
  if (side_tracking_)
  {
    return (withinRayDistance(robot_coordinates, waypoint_coordinates) &&
            withinSideRayAngle(robot_coordinates, waypoint_coordinates));
  }
  else
  {
    return (withinRayDistance(robot_coordinates, waypoint_coordinates) &&
            withinFrontRayAngle(robot_coordinates, waypoint_coordinates));
  }
}

bool RayTracker::withinRayDistance(const agribot_local_planner::pos& robot_center,
                                   const agribot_local_planner::pos& waypoint) const
{
  return (CommonFunctions::dDistanceBetweenCoordinates(waypoint, robot_center) <= ray_length_);
}

bool RayTracker::withinSideRayAngle(const agribot_local_planner::pos& robot_center,
                                    const agribot_local_planner::pos& waypoint) const
{
  double left_ray_angle = CommonFunctions::limitAngle(robot_center.az + M_PI_2);
  double right_ray_angle = CommonFunctions::limitAngle(robot_center.az - M_PI_2);
  double robot_waypoint_angle =
      CommonFunctions::limitAngle(CommonFunctions::dAngleBetweenCoordinates(waypoint, robot_center));
  return ((calculateLinesSpan(robot_waypoint_angle, left_ray_angle) <= ray_width_ / 2.0) ||
          (calculateLinesSpan(robot_waypoint_angle, right_ray_angle) <= ray_width_ / 2.0));
}

bool RayTracker::withinFrontRayAngle(const agribot_local_planner::pos& robot_center,
                                     const agribot_local_planner::pos& waypoint) const
{
  double front_ray_angle = robot_center.az;
  double robot_waypoint_angle =
      CommonFunctions::limitAngle(CommonFunctions::dAngleBetweenCoordinates(waypoint, robot_center));
  return (calculateLinesSpan(robot_waypoint_angle, front_ray_angle) <= ray_width_ / 2.0);
}

double RayTracker::calculateLinesSpan(const double target_line_angle, const double base_line_angle) const
{
  return CommonFunctions::toDegrees(abs(CommonFunctions::limitAngle(target_line_angle - base_line_angle)));
}
}  // namespace agribot_path_trackers
