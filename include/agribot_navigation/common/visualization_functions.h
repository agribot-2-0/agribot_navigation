/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <cmath>
#include <vector>

#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/common/common_functions.h"

namespace VisualizatioFucntions
{
void vPredictedPathVisualization(const geometry_msgs::Twist& velocity, const agribot_local_planner::pos& robot_center,
                                 const double prediction_points_per_second, const double prediction_time,
                                 const ros::Publisher& publisher);
void vNextGoalVisualization(const agribot_local_planner::pos& last_waypoint, const agribot_local_planner::pos& goal,
                            const ros::Publisher& publisher);
void vGoalRangeVisualization(const agribot_local_planner::pos& goal, const agribot_local_planner::pos& range,
                             const double goal_distance_tolerance, const ros::Publisher& publisher);
void vLocalPathVisualization(const std::vector<geometry_msgs::PoseStamped>& path,
                             const ros::Publisher& publisher);  // NOLINT(runtime/references)
}  // namespace VisualizatioFucntions
