/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wformat"
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#pragma GCC diagnostic pop
#include <agribot_common/CommonFunctions/ros_common_functions.h>
#include <agribot_navigation/common/common_functions.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

namespace agribot_global_planner
{
class AgribotGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
  AgribotGlobalPlanner();

  AgribotGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  ~AgribotGlobalPlanner();

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  bool makePlan(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);  // NOLINT(runtime/references)

private:
  costmap_2d::Costmap2DROS* costmap_ros_;        //!< costmap ros wrapper
  costmap_2d::Costmap2D* costmap_;               //!< costmap container
  base_local_planner::WorldModel* world_model_;  //!< world model
  bool initialized_;                             //!< flag indicating the planner has been initialized

  ros::Publisher plan_pub_;

  double param_step_size_;
  std::string param_frame_id_;

  nav_msgs::Path path_;  //!< container for the generated interpolated path
};

}  // namespace agribot_global_planner
