/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <string>
#include <vector>

#include "agribot_navigation/common/common_functions.h"
#include "agribot_navigation/common/structures.h"

namespace VisualizationFunctions
{
void predictedPathVisualization(const geometry_msgs::Twist& velocity, const agribot_local_planner::pos& robot_center,
                                const double prediction_points_per_second, const double prediction_time,
                                const std::string& frame_id, const ros::Publisher& publisher);
void goalVisualization(const agribot_local_planner::pos& last_waypoint, const agribot_local_planner::pos& goal,
                       const std::string& frame_id, const ros::Publisher& publisher);
void pathVisualization(const std::vector<geometry_msgs::PoseStamped>& path, const std::string& frame_id,
                       const ros::Publisher& publisher);  // NOLINT(runtime/references)
void poseVisualization(const geometry_msgs::PoseStamped& pose, const ros::Publisher& publisher);
}  // namespace VisualizationFunctions
