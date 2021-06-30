/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/agribot_global_planner.h"

#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>

PLUGINLIB_EXPORT_CLASS(agribot_global_planner::AgribotGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace agribot_global_planner
{
AgribotGlobalPlanner::AgribotGlobalPlanner() : costmap_ros_(NULL), initialized_(false)
{
}

AgribotGlobalPlanner::AgribotGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

AgribotGlobalPlanner::~AgribotGlobalPlanner()
{
}

void AgribotGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    // get the costmap
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~" + name);

    // load parameters
    try
    {
      param_step_size_ = ros_common_functions::getRosParam<double>(pnh.getNamespace() + "/step_size");
      param_frame_id_ = ros_common_functions::getRosParam<std::string>(pnh.getNamespace() + "/reference_frame_id");
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("[AgribotGlobalPlanner] " << ex.what());
    }

    plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);

    path_.header.frame_id = param_frame_id_;

    initialized_ = true;
    ROS_INFO("[AgribotGlobalPlanner] Planner has been initialized");
  }
  else
  {
    ROS_WARN("[AgribotGlobalPlanner] This planner has already been initialized");
  }
}

bool AgribotGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose,
                                    const geometry_msgs::PoseStamped& goal,
                                    std::vector<geometry_msgs::PoseStamped>& plan)
{
  ROS_INFO("[AgribotGlobalPlanner] Make plan start");

  if (!initialized_)
  {
    ROS_ERROR_STREAM("[AgribotGlobalPlanner] Planner not initialized, call initialize() before planner");
    return false;
  }

  try
  {
    CommonFunctions::straightPath(plan, start_pose, goal, param_frame_id_, param_step_size_);
    plan.push_back(goal);
    CommonFunctions::updateOrientationInPath(plan, 0, plan.size() - 1);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("[AgribotGlobalPlanner] " << e.what());
    return false;
  }

  path_.header.stamp = ros::Time::now();
  path_.poses = plan;
  plan_pub_.publish(path_);
  ROS_INFO("[AgribotGlobalPlanner] Make plan finish");
  return true;
}

}  // namespace agribot_global_planner
