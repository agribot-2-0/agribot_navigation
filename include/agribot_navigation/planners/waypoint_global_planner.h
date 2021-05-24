/*
 * Copyright (C) 2020 Agribot. All rights reserved.
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
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

namespace waypoint_global_planner
{
/**
 * @class CarrotPlanner
 * @brief Provides a simple global planner for producing boustrophedon paths without taking into account obstacles
 */
class WaypointGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
  /**
   * @brief Default Constructor
   */
  WaypointGlobalPlanner();

  /**
   * @brief Constructor for the planner
   * @param name The name of this planner
   * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
   */
  WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Default destructor
   */
  ~WaypointGlobalPlanner();

  /**
   * @brief Initialization function for the planner
   * @param name The name of this planner
   * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
   */
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start_pose The starting pose of the robot
   * @param goal The goal pose
   * @param plan The plan filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  bool makePlan(const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan);  // NOLINT(runtime/references)

  /**
   * @brief Creates and publishes visualization markers from an input path
   * @param path The input path to be used for the creation of visualization markers
   */
  void
  createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path);  // NOLINT(runtime/references)

  /**
   * @brief Interpolates a path (position and orientation) using a fixed number of points per meter
   * @param path The input path to be interpolated
   */
  void interpolatePath(nav_msgs::Path& path);  // NOLINT(runtime/references)

private:
  costmap_2d::Costmap2DROS* costmap_ros_;        //!< costmap ros wrapper
  costmap_2d::Costmap2D* costmap_;               //!< costmap container
  base_local_planner::WorldModel* world_model_;  //!< world model
  bool initialized_;                             //!< flag indicating the planner has been initialized

  //!< publisher of waypoint visualization markers
  ros::Publisher waypoint_marker_pub_;
  //!< publisher of goal corresponding to the final waypoint
  ros::Publisher goal_pub_;
  ros::Publisher plan_pub_;  //!< publisher of the global plan

  // !< number of waypoints per meter of generated path used for interpolation
  double waypoints_per_meter_;
  double param_goal_distance_tolerance;

  // containers
  //!< container for the manually inserted waypoints
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  nav_msgs::Path path_;  //!< container for the generated interpolated path
};

}  // namespace waypoint_global_planner
