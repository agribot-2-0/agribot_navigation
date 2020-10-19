/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/local_planners/i_local_planner.h"

namespace agribot_local_planner
{
void ILocalPlanner::setFrameId(const std::string& frame)
{
  frame_id = frame;
}
}  // namespace agribot_local_planner
