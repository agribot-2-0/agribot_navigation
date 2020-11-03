/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_slicers/folllowing_slicer.h"

namespace agribot_path_slicers
{
FollowingSlicer::FollowingSlicer()
{
}
FollowingSlicer::~FollowingSlicer()
{
}
bool FollowingSlicer::findPathSlice(std::vector<geometry_msgs::PoseStamped>& path_slice,  // NOLINT(runtime/references)
                                    const std::vector<geometry_msgs::PoseStamped>& local_path,
                                    const geometry_msgs::PoseStamped& robot_pose)
{
    return true;
}
}  // namespace agribot_path_slicers
