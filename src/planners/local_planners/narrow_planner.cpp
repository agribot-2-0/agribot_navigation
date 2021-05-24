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
  static_cast<void>(end_pose);
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
      auto midpose = calculateMidpose(line_left[position], line_right[position]);
      midpose.header.seq = path.size() + 1;
      path.push_back(midpose);
    }
    CommonFunctions::updateOrientationInPath(path, 0, path.size() - 1);
  }
  catch (const std::exception& e)
  {
    throw std::invalid_argument("[NarrowPlanner]" + std::string(e.what()));
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
  geometry_msgs::PoseStamped midpose;
  midpose.header.frame_id = frame_id;
  midpose.header.stamp = ros::Time::now();

  midpose.pose.position.x = (point1.pose.position.x + point2.pose.position.x) / 2;
  midpose.pose.position.y = (point1.pose.position.y + point2.pose.position.y) / 2;

  midpose.pose.orientation.w = 1;

  return midpose;
}

void NarrowCorridorPlanner::clearObstacles(void)
{
  obstacles_left_.initializeEmpty();
  obstacles_right_.initializeEmpty();
  obstacles_.clear();
}

void NarrowCorridorPlanner::setChassisLength(double chassis_length)
{
  chassis_length_ = chassis_length;
}
void NarrowCorridorPlanner::setStepSize(const double param_step_size)
{
  step_size_ = param_step_size;
}

}  // namespace agribot_local_planner
