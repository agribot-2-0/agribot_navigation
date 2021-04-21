/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/local_planners/narrow_planner.h"

namespace agribot_local_planner
{
NarrowCorridorPlanner::NarrowCorridorPlanner() : ILocalPlanner()
{
}

NarrowCorridorPlanner::NarrowCorridorPlanner(const std::string& frame) : ILocalPlanner()
{
  setFrameId(frame);
}

NarrowCorridorPlanner::~NarrowCorridorPlanner()
{
}

bool NarrowCorridorPlanner::designPath(std::vector<geometry_msgs::PoseStamped>& path,
                                       const geometry_msgs::PoseStamped& start_pose,
                                       const geometry_msgs::PoseStamped& end_pose)
{
  auto line_left = obstacles_left_.toPath();
  auto line_right = obstacles_right_.toPath();

  path.clear();
  auto end_point = calculateMidpoint(line_left[0], line_right[0]);
  // straightPath(path, start_pose, end_point, 0.5);

  size_t path_size = line_left.size() < line_right.size() ? line_left.size() : line_right.size();

  for (size_t position = 1; position < path_size; position++)
  {
    // ROS_WARN_STREAM("in_loop " << position);

    auto mid_point = calculateMidpoint(line_left[position], line_right[position]);
    mid_point.header.seq = position;
    mid_point.header.frame_id = frame_id;
    mid_point.header.stamp = ros::Time::now();

    // ROS_WARN_STREAM("in_loop " << mid_point);
    path.push_back(mid_point);
  }

  // straightPath(path, path[-1], end_pose, 0.5);

  CommonFunctions::updateOrientationInPath(path, 0, path.size());

  return true;
}

bool NarrowCorridorPlanner::straightPath(std::vector<geometry_msgs::PoseStamped>& path,
                                         const geometry_msgs::PoseStamped& start_point,
                                         const geometry_msgs::PoseStamped& end_point, const double step_size)
{
  int steps = CommonFunctions::distanceBetweenPoses(start_point, end_point) / step_size;

  if (steps <= 0)
  {
    ROS_ERROR_STREAM("[Obstacle] Could not assemble straight path");
    return false;
  }

  for (int i = 0; i < steps; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.seq = path.size() + 1;
    pose.pose.position.x = start_point.pose.position.x +
                           static_cast<double>(i) / steps * (end_point.pose.position.x - start_point.pose.position.x);
    pose.pose.position.y = start_point.pose.position.y +
                           static_cast<double>(i) / steps * (end_point.pose.position.y - start_point.pose.position.y);
    path.push_back(pose);
  }
  return true;
}

void NarrowCorridorPlanner::addObstacle(const ObstacleDetection::Obstacle& obstacle)
{
  if (obstacle.getMinY().y() > 0)
  {
    if (obstacles_left_.getVerticesNumber() <= 0)
    {
      obstacles_left_ = obstacle;
    }
    else
    {
      obstacles_left_ += obstacle;
    }
  }
  else
  {
    if (obstacles_right_.getVerticesNumber() <= 0)
    {
      obstacles_right_ = obstacle;
    }
    else
    {
      obstacles_right_ += obstacle;
    }
  }
}

std::vector<ObstacleDetection::Obstacle> NarrowCorridorPlanner::getObstacles(void)
{
  obstacles_.clear();
  obstacles_.push_back(obstacles_left_);
  obstacles_.push_back(obstacles_right_);
  return obstacles_;
}

geometry_msgs::PoseStamped NarrowCorridorPlanner::calculateMidpoint(const geometry_msgs::PoseStamped& point1,
                                                                    const geometry_msgs::PoseStamped& point2)
{
  geometry_msgs::PoseStamped midpoint;

  midpoint.pose.position.x = (point1.pose.position.x + point2.pose.position.x) / 2;
  midpoint.pose.position.y = (point1.pose.position.y + point2.pose.position.y) / 2;

  return midpoint;
}

}  // namespace agribot_local_planner
