/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/local_planners/narrow_planner.h"

namespace agribot_local_planner
{
NarrowCorridorPlanner::NarrowCorridorPlanner(const std::string& frame) : ILocalPlanner()
{
  frame_id = frame;
}

bool NarrowCorridorPlanner::designPath(std::vector<geometry_msgs::PoseStamped>& path,
                                       const geometry_msgs::PoseStamped& start_pose,
                                       const geometry_msgs::PoseStamped& end_pose)
{
  try
  {
    if (!line_left_.size() || !line_right_.size() || !path.size())
    {
      return false;
    }
    path.clear();
    path.push_back(start_pose);
    auto start_obstacle_pose = calculateMidpose(line_left_.front(), line_right_.front());

    // if (start_obstacle_pose.pose.position.x < end_pose.pose.position.x)
    // {
    //   CommonFunctions::straightPath(path, start_pose, start_obstacle_pose, frame_id, step_size_);
    // }

    size_t path_size = line_left_.size() < line_right_.size() ? line_left_.size() : line_right_.size();

    for (size_t position = 1; position < path_size; position++)
    {
      auto midpose = calculateMidpose(line_left_[position], line_right_[position]);
      if (midpose.pose.position.x < end_pose.pose.position.x)
      {
        midpose.header.seq = path.size() + 1;
        path.push_back(midpose);
      }
    }

    auto midpose_last = calculateMidpose(line_left_.back(), line_right_.back());
    // ROS_WARN_STREAM(midpose_last.pose.position.x);
    // ROS_WARN_STREAM(end_pose.pose.position.x);
    // if (midpose_last.pose.position.x < end_pose.pose.position.x)
    // {
    //   CommonFunctions::straightPath(path, midpose_last, end_pose, frame_id, step_size_);
    // }

    path.back().pose.orientation.x = 0;
    path.back().pose.orientation.y = 0;
    path.back().pose.orientation.z = 0;
    path.back().pose.orientation.w = 1;

    CommonFunctions::updateOrientationInPath(path, 1, path.size() - 1);
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

void NarrowCorridorPlanner::setLineLeft(const nav_msgs::Path& line)
{
  // ROS_WARN_STREAM("set line left " << line.poses.size());
  // ROS_WARN_STREAM("set line left " << line.poses.size());
  if (line.poses.empty())
  {
    return;
  }
  line_left_.clear();
  CommonFunctions::straightPath(line_left_, line.poses.front(), line.poses.back(), frame_id, step_size_);
}

void NarrowCorridorPlanner::setLineRight(const nav_msgs::Path& line)
{
  // ROS_WARN_STREAM("set line right");
  // ROS_WARN_STREAM("set line right " << line.poses.size());
  if (line.poses.empty())
  {
    return;
  }
  line_right_.clear();
  CommonFunctions::straightPath(line_right_, line.poses.front(), line.poses.back(), frame_id, step_size_);
}

}  // namespace agribot_local_planner
