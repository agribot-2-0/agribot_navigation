/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_slicers/ray_slicer.h"

namespace agribot_path_slicers
{
RaySlicer::RaySlicer()
{
}
RaySlicer::~RaySlicer()
{
}
bool RaySlicer::findPathSlice(std::vector<geometry_msgs::PoseStamped>& path_slice,  // NOLINT(runtime/references)
                              const std::vector<geometry_msgs::PoseStamped>& local_path,
                              const geometry_msgs::PoseStamped& robot_pose)
{
  return true;
}
}  // namespace agribot_path_slicers