/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_slicers/folllowing_slicer.h"

namespace agribot_path_slicers
{
FollowingSlicer::FollowingSlicer()
{
  param_missed_distance_coefficient_ = default_missed_distance_coefficient;
  param_goal_distance_tolerance_ = default_goal_distance_tolerance;
  reset();
}
FollowingSlicer::~FollowingSlicer()
{
}

bool FollowingSlicer::findPathSlice(std::vector<geometry_msgs::PoseStamped>& path_slice,  // NOLINT(runtime/references)
                                    const std::vector<geometry_msgs::PoseStamped>& local_path,
                                    const geometry_msgs::PoseStamped& robot_pose)
{
  agribot_local_planner::pos robot_center = CommonFunctions::pCoordinatesFromPoseStamped(robot_pose);
  agribot_local_planner::pos next_waypoint = CommonFunctions::pCoordinatesFromPoseStamped(local_path[waypoint_count_]);
  agribot_local_planner::pos last_waypoint =
      CommonFunctions::pCoordinatesFromPoseStamped(local_path[waypoint_count_ - 1]);

  if (is_last_robot_center_initialized_)
  {
    traveled_distance_ += calculateTraveledDistance(robot_center, last_robot_center_);
    last_robot_center_ = robot_center;
  }
  else
  {
    last_robot_center_ = robot_center;
    is_last_robot_center_initialized_ = true;
  }

  if (!isLastWaypointInPath(waypoint_count_, local_path.size()) &&
      (waypointHitCondition(robot_center, last_waypoint, next_waypoint) ||
       waypointMissedCondition(robot_center, last_waypoint, next_waypoint)))
  {
    takeNextGoal();
  }

  path_slice.clear();
  path_slice.push_back(local_path[waypoint_count_ - 1]);
  path_slice.push_back(local_path[waypoint_count_]);
  return true;
}

void FollowingSlicer::reset()
{
  waypoint_count_ = 1;
  last_traveled_distance_ = 0.0;
  traveled_distance_ = 0.0;
  last_robot_center_ = agribot_local_planner::pos();
  is_last_robot_center_initialized_ = false;
}

void FollowingSlicer::setMissedDistanceCoefficient(double coefficient)
{
  param_missed_distance_coefficient_ = coefficient;
}
void FollowingSlicer::setGoalDistanceTolerance(double tolerance)
{
  param_goal_distance_tolerance_ = tolerance;
}

void FollowingSlicer::takeNextGoal()
{
  waypoint_count_++;
  last_traveled_distance_ = traveled_distance_;
}

bool FollowingSlicer::isLastWaypointInPath(std::size_t waypoints_count, std::size_t path_length)
{
  return (waypoints_count >= (path_length - 1));
}

bool FollowingSlicer::waypointHitCondition(const agribot_local_planner::pos& robot_center,
                                            const agribot_local_planner::pos& last_waypoint,
                                            const agribot_local_planner::pos& next_waypoint)
{
  {
    return (traveledDistanceBetweenWaypoints(robot_center, last_waypoint, next_waypoint) &&
            withinGoalRange(robot_center, next_waypoint));
  }
}

bool FollowingSlicer::waypointMissedCondition(const agribot_local_planner::pos& robot_center,
                                               const agribot_local_planner::pos& last_waypoint,
                                               const agribot_local_planner::pos& next_waypoint)
{
  return centerTraveledBeyondGoal(robot_center, last_waypoint, next_waypoint) &&
         !withinGoalRange(robot_center, next_waypoint);
}

bool FollowingSlicer::traveledDistanceBetweenWaypoints(const agribot_local_planner::pos& robot_center,
                                                        const agribot_local_planner::pos& last_waypoint,
                                                        const agribot_local_planner::pos& next_waypoint)
{
  double distance_waypoints = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, last_waypoint);
  double traveled_distance = CommonFunctions::dDistanceBetweenCoordinates(last_waypoint, robot_center);
  return (traveled_distance >= distance_waypoints - param_goal_distance_tolerance_);
}

bool FollowingSlicer::withinGoalRange(const agribot_local_planner::pos& robot_center,
                                       const agribot_local_planner::pos& next_waypoint)
{
  double distance_to_goal = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, robot_center);
  return (distance_to_goal <= param_goal_distance_tolerance_);
}

bool FollowingSlicer::centerTraveledBeyondGoal(const agribot_local_planner::pos& robot_center,
                                                const agribot_local_planner::pos& last_waypoint,
                                                const agribot_local_planner::pos& next_waypoint)
{
  double distance_waypoints = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, last_waypoint);
  double traveled_distance = traveled_distance_ - last_traveled_distance_;
  return (traveled_distance >= distance_waypoints * param_missed_distance_coefficient_);
}

double FollowingSlicer::calculateTraveledDistance(const agribot_local_planner::pos& current_position,
                                                  const agribot_local_planner::pos& last_position)
{
  return CommonFunctions::dDistanceBetweenCoordinates(last_position, current_position);
}

}  // namespace agribot_path_slicers
