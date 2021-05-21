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
  (void)end_pose;
  path.push_back(start_pose);
  try
  {
    auto line_left = obstacles_left_.toPath(step_size_);
    auto line_right = obstacles_right_.toPath(step_size_);
    if (!line_left.size() || !line_right.size() || !path.size())
    {
      return false;
    }
    auto start_obstacle_pose = calculateMidpose(line_left[0], line_right[0]);

    CommonFunctions::straightPath(path, start_pose, start_obstacle_pose, frame_id, step_size_);

    size_t path_size = line_left.size() < line_right.size() ? line_left.size() : line_right.size();

    for (size_t position = 1; position < path_size; position++)
    {
      auto mid_point = calculateMidpose(line_left[position], line_right[position]);
      mid_point.header.seq = path.size() + 1;
      path.push_back(mid_point);
    }
    CommonFunctions::updateOrientationInPath(path, 0, path.size() - 1);
  }
  catch (const std::exception& e)
  {
    throw std::invalid_argument("[NarrowPlanner]" + std::string(e.what()));
  }

  return true;
}

bool NarrowCorridorPlanner::straightPath(std::vector<geometry_msgs::PoseStamped>& path,
                                         const geometry_msgs::PoseStamped& start_point,
                                         const geometry_msgs::PoseStamped& end_point, const double step_size)
{
  int steps = CommonFunctions::distanceBetweenPoses(start_point, end_point) / step_size;

  if (steps <= 0)
  {
    ROS_ERROR_STREAM("[NarrowCorridorPlanner] Could not assemble straight path");
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
  if (obstacle.getMinX().x() < (chassis_length_ / 2))
  {
    return;
  }

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

geometry_msgs::PoseStamped NarrowCorridorPlanner::calculateMidpose(const geometry_msgs::PoseStamped& point1,
                                                                   const geometry_msgs::PoseStamped& point2)
{
  geometry_msgs::PoseStamped midpoint;
  midpoint.header.frame_id = frame_id;
  midpoint.header.stamp = ros::Time::now();

  midpoint.pose.position.x = (point1.pose.position.x + point2.pose.position.x) / 2;
  midpoint.pose.position.y = (point1.pose.position.y + point2.pose.position.y) / 2;

  midpoint.pose.orientation.w = 1;

  return midpoint;
}

}  // namespace agribot_local_planner
