/*
 * Copyright (C) 2021 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/local_planners/straight_path_planner.h"

namespace agribot_local_planner
{
StraightPathPlanner::StraightPathPlanner() : ILocalPlanner()
{
}

StraightPathPlanner::StraightPathPlanner(const std::string& frame) : ILocalPlanner()
{
  setFrameId(frame);
}

StraightPathPlanner::~StraightPathPlanner()
{
}

bool StraightPathPlanner::designPath(std::vector<geometry_msgs::PoseStamped>& path,
                                     const geometry_msgs::PoseStamped& start_pose,
                                     const geometry_msgs::PoseStamped& end_pose)
{
  try
  {
    CommonFunctions::straightPath(path, start_pose, end_pose, frame_id, step_size_);
    CommonFunctions::updateOrientationInPath(path, 0, path.size());
  }
  catch (const std::exception& e)
  {
    throw std::invalid_argument("[Obstacle] " + std::string(e.what()));
  }
  return true;
}

void StraightPathPlanner::addObstacle(const ObstacleDetection::Obstacle& obstacle)
{
  obstacles_.push_back(obstacle);
}

std::vector<ObstacleDetection::Obstacle> StraightPathPlanner::getObstacles(void)
{
  return obstacles_;
}

void StraightPathPlanner::clearObstacles(void)
{
  obstacles_.clear();
}

void StraightPathPlanner::setChassisLength(double chassis_length)
{
  chassis_length_ = chassis_length;
}
void StraightPathPlanner::setStepSize(const double param_step_size)
{
  step_size_ = param_step_size;
}

}  // namespace agribot_local_planner
