/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/local_planners/i_local_planner.h"

namespace agribot_local_planner
{
ILocalPlanner::ILocalPlanner()
{
  functions = CommonFunctions();
}

void ILocalPlanner::setHeader(const std::string frame)
{
  frame_id = frame;
}

void ILocalPlanner::updateOrientation(std::vector<geometry_msgs::PoseStamped>& path, const int start_index,
                                      const int end_index) const
{
  // add proper orientation to each point
  for (int i = start_index; i < end_index; i++)
  {
    double angle = std::atan2(path[i + 1].pose.position.y - path[i].pose.position.y,
                              path[i + 1].pose.position.x - path[i].pose.position.x);
    path[i].pose.orientation.x = 0;
    path[i].pose.orientation.y = 0;
    path[i].pose.orientation.z = sin(angle / 2.0);
    path[i].pose.orientation.w = cos(angle / 2.0);
  }
}
}  // namespace agribot_local_planner
