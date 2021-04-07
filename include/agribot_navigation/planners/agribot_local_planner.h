/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <base_local_planner/goal_functions.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_converter/costmap_converter_interface.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "agribot_navigation/common/common_functions.h"
#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/common/visualization_functions.h"
#include "agribot_navigation/obstacle_detection/detection.h"
#include "agribot_navigation/obstacle_detection/obstacle.h"
#include "agribot_navigation/path_controllers/deviation_parallelness_controller.h"
#include "agribot_navigation/path_controllers/i_path_controller.h"
#include "agribot_navigation/path_controllers/pure_pursuit_controller.h"
#include "agribot_navigation/path_controllers/stanley_controller.h"
#include "agribot_navigation/path_trackers/following_tracker.h"
#include "agribot_navigation/path_trackers/i_path_tracker.h"
#include "agribot_navigation/path_trackers/ray_tracker.h"
#include "agribot_navigation/path_validators/curvature_validator.h"
#include "agribot_navigation/path_validators/distance_validator.h"
#include "agribot_navigation/path_validators/i_path_validator.h"
#include "agribot_navigation/planners/local_planners/circles_planner.h"
#include "agribot_navigation/planners/local_planners/hermite_planner.h"
#include "agribot_navigation/planners/local_planners/i_local_planner.h"

namespace agribot_local_planner
{
class AgribotPlannerROS : public nav_core::BaseLocalPlanner
{
public:
  AgribotPlannerROS();
  ~AgribotPlannerROS();
  void initialize(std::string name, tf2_ros::Buffer* tfBuffer, costmap_2d::Costmap2DROS* costmapROS);
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);  // NOLINT(runtime/references)
  bool isGoalReached();

private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* local_costmap_;
  tf2_ros::Buffer* tf_buffer_;
  tf2_ros::TransformListener* tf_listener_;
  ros::Subscriber odom_sub_;
  ros::Publisher path_pub_;
  ros::Publisher predicted_path_pub_;
  ros::Publisher path_slice_pub_;
  ros::Publisher obstacles_pub_;
  ros::Publisher obstacles_polygon_pub_;
  ros::Publisher obstacles_rectangle_pub_;
  ros::Publisher obstacles_polygon_markers_pub_;
  ros::Publisher obstacles_rectangle_markers_pub_;
  ros::Publisher rabbit_pubisher_;
  // contains the global plan
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> local_plan_;
  std::vector<geometry_msgs::PoseStamped> display_plan_;
  nav_msgs::Path local_path_;
  nav_msgs::Path path_slice_;
  geometry_msgs::Twist velocities_;  // contains the velocity
  visualization_msgs::Marker traveled_points_;
  geometry_msgs::PoseStamped robot_center_;  // present robot center frame
  std::unordered_map<std::string, std::unique_ptr<ILocalPlanner>> planners_;
  std::string reference_frame_id_;
  std::unordered_map<std::string, std::unique_ptr<agribot_path_trackers::IPathTracker>> trackers_;
  std::unordered_map<std::string, std::unique_ptr<agribot_path_validators::IPathValidator>> validators_;
  std::unordered_map<std::string, std::unique_ptr<agribot_path_controllers::IPathController>> controllers_;

  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_;
  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;
  std::string costmap_converter_name_;
  std::string costmap_converter_odom_topic_;
  std::string obstacle_frame_id;
  double costmap_converter_rate_;
  costmap_converter::ObstacleArrayConstPtr obstacles_array_;
  std::vector<ObstacleDetection::Obstacle> obstacles_;
  ObstacleDetection::Detection detection_;
  bool obstacle_ahead_;
  bool in_interrow_;

  bool goal_reached_;
  bool initialized_;
  // time measuring for statistics
  double stop_time_, start_time_, last_time_;
  double last_odom_stamp_;
  // deviation measuring for statistics
  double traveled_distance_;
  bool abort_executing_;
  // params
  int param_missed_points_tolerance_;          // n
  int param_prediction_time_;                  // seconds
  int param_prediction_points_per_second_;     // n/second
  double param_goal_distance_tolerance_;       // metres
  double param_goal_range_;                    // factor
  double param_chassis_length_;                // metres
  double param_min_radius_;                    // metres
  double param_dead_zone_;                     // degrees
  double param_max_linear_vel_;                // m/s
  double param_min_linear_vel_;                // m/s
  double param_current_deviation_weight_;      // factor
  double param_current_parallelness_weight_;   // factor
  double param_deviation_weight_straight_;     // factor
  double param_parallelness_weight_straight_;  // factor
  double param_deviation_weight_arc_;          // factor
  double param_parallelness_weight_arc_;       // factor
  double param_epsilon_;                       // unitless

  double param_design_dead_zone_;             // degrees
  double param_design_arc_radius_;            // metres
  double param_design_step_size_;             // metres
  double param_hermite_magnitude_;            // factor
  double param_odom_filter_time_;             // seconds
  double param_missed_distance_coefficient_;  // factor >1

  bool param_speed_limit_;
  bool param_debug_;
  bool param_statistics_;
  bool param_take_front_;

  void checkObstacles();
  void processObstacles(const costmap_converter::ObstacleArrayConstPtr& obstacles);
  void publishObstacles() const;
  void vOdomCallback(const nav_msgs::Odometry& odometry_msg);
  void createPlanners();
  void createTrackers();
  void createValidators();
  void createControllers();
  void createCostmapConverter(const ros::NodeHandle& nh);
  void visualizeObstaclesRectangles(const std::vector<ObstacleDetection::Obstacle>& obstacles) const;
  void visualizeObstaclesPolygons(const std::vector<ObstacleDetection::Obstacle>& obstacles) const;
  bool replan(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
              const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& end_pose,
              const std::string& planner_id);
  geometry_msgs::PoseStamped xTransformPose(geometry_msgs::PoseStamped ps,
                                            std::string& frame_id);  // NOLINT(runtime/references)
  geometry_msgs::PolygonStamped transformPolygon(const geometry_msgs::PolygonStamped& ps,
                                                 std::string& frame_id);  // NOLINT(runtime/references)
  geometry_msgs::Twist xLimitVelocities(geometry_msgs::Twist velocities);
  geometry_msgs::Twist xStopRobot();
  geometry_msgs::Twist xFinishedPathBehaviour();
  void resetControllers();
  double dCloseToZero(const double value);
  bool bReachedPathEnd(const pos& robot_coordinates, const pos& path_end_waypoint);
};
}  // namespace agribot_local_planner
