/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#pragma once

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wformat"
#include <base_local_planner/goal_functions.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>
#pragma GCC diagnostic pop
#include <agribot_common/CommonFunctions/ros_common_functions.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>
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
#include "agribot_navigation/planners/local_planners/narrow_planner.h"
#include "agribot_navigation/planners/local_planners/straight_path_planner.h"

namespace agribot_local_planner
{
class AgribotLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
  AgribotLocalPlanner();
  ~AgribotLocalPlanner();
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
  ros::Subscriber line_left_sub_;
  ros::Subscriber line_right_sub_;
  ros::Subscriber is_row_sub_;
  ros::Publisher path_pub_;
  ros::Publisher obstacles_pub_;
  ros::Publisher obstacles_left_polygon_pub_;
  ros::Publisher obstacles_right_polygon_pub_;
  ros::Publisher obstacles_polygon_pub_;
  ros::Publisher obstacles_left_line_pub_;
  ros::Publisher obstacles_right_line_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher rabbit_pub_;

  // contains the global plan
  std::vector<geometry_msgs::PoseStamped> local_plan_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  std::vector<geometry_msgs::PoseStamped> trajectory_;
  nav_msgs::Path path_slice_;
  visualization_msgs::Marker traveled_points_;
  geometry_msgs::PoseStamped robot_center_;
  geometry_msgs::PoseStamped goal_;
  geometry_msgs::PoseStamped goal_previous_;
  std::string reference_frame_id_;
  std::string obstacle_frame_id_;

  std::unordered_map<std::string, std::unique_ptr<ILocalPlanner>> planners_;
  std::unordered_map<std::string, std::unique_ptr<agribot_path_trackers::IPathTracker>> trackers_;
  std::unordered_map<std::string, std::unique_ptr<agribot_path_validators::IPathValidator>> validators_;
  std::unordered_map<std::string, std::unique_ptr<agribot_path_controllers::IPathController>> controllers_;
  std::vector<std::string> planners_fallback_strategy_;

  pluginlib::ClassLoader<costmap_converter::BaseCostmapToPolygons> costmap_converter_loader_;
  boost::shared_ptr<costmap_converter::BaseCostmapToPolygons> costmap_converter_;
  std::string costmap_converter_name_;
  std::string costmap_converter_odom_topic_;
  double costmap_converter_rate_;

  costmap_converter::ObstacleArrayConstPtr obstacles_array_;
  std::vector<ObstacleDetection::Obstacle> obstacles_;
  ObstacleDetection::Detection detection_;
  ObstacleDetection::Obstacle obstacles_left_;
  ObstacleDetection::Obstacle obstacles_right_;
  bool obstacle_ahead_;
  bool is_row_;

  bool goal_reached_;
  bool initialized_;
  bool local_plan_planned_;
  // time measuring for statistics
  double stop_time_, start_time_, last_time_;
  double last_odom_stamp_;
  // deviation measuring for statistics
  double traveled_distance_;
  bool abort_executing_;
  bool is_out_of_path_;
  int out_of_path_counter_;

  // params
  int param_max_missed_waypoints_;          // n
  double param_goal_distance_tolerance_x_;  // metres
  double param_goal_distance_tolerance_y_;  // metres
  double param_chassis_length_;             // metres
  double param_min_radius_;                 // metres
  double param_max_linear_vel_;             // m/s
  double param_min_linear_vel_;             // m/s
  double param_epsilon_;                    // unitless

  double param_design_dead_zone_;                    // degrees
  double param_design_arc_radius_;                   // metres
  double param_design_step_size_;                    // metres
  double param_hermite_magnitude_;                   // factor
  double param_odom_filter_time_;                    // seconds
  double param_missed_distance_coefficient_;         // factor >1
  double param_maximum_distance_to_path_tolerance_;  // factor >1

  double param_pure_pursuit_rabbit_distance_;  // metres
  double param_stanley_error_gain_;            // metres

  std::string param_agribot_controller_name_;

  bool param_statistics_;

  void createPlanners();
  void createTrackers();
  void createValidators();
  void createControllers();
  void createCostmapConverter(const ros::NodeHandle& nh);

  void checkObstacles();
  void processObstacles(const costmap_converter::ObstacleArrayConstPtr& obstacles);
  void publishObstacles();
  void odomCallback(const nav_msgs::Odometry& odometry_msg);
  void lineLeftCallback(const nav_msgs::Path& msg);
  void lineRightCallback(const nav_msgs::Path& msg);
  void isRowCallback(const std_msgs::Bool& msg);

  bool planPath(const std::string& planner_id, const geometry_msgs::PoseStamped& start_pose,
                const geometry_msgs::PoseStamped& end_pose);
  void calculateControls(geometry_msgs::Twist& cmd_vel, std::string controller_id);  // NOLINT(runtime/references)
  void visualizationPath(std::string frame_id);
  void visualizationObstacles(void);

  geometry_msgs::PoseStamped transformPose(const geometry_msgs::PoseStamped& pose,
                                           const std::string& frame_id);  // NOLINT(runtime/references)
  geometry_msgs::PolygonStamped transformPolygon(const geometry_msgs::PolygonStamped& pose,
                                                 const std::string& frame_id);  // NOLINT(runtime/references)
  geometry_msgs::Twist limitVelocities(geometry_msgs::Twist velocities);
  geometry_msgs::Twist stopRobot();
  geometry_msgs::Twist finishedPathBehaviour();
  void abortPlanningBehaviour();
  bool handlePathPlan();
  double closeToZero(const double value);
  bool isReachedPathEnd(const geometry_msgs::PoseStamped& robot_coordinates,
                        const geometry_msgs::PoseStamped& path_end_waypoint);
  bool isOutOfPath();
  bool isInRow();
  bool isWithinEllipse(const geometry_msgs::PoseStamped& point, const geometry_msgs::PoseStamped& ellipse_center);
  void getParameters(const ros::NodeHandle& nh);

  const std_msgs::ColorRGBA COLOR_RED = VisualizationFunctions::ColorRGBA(1, 0, 0, 1).color;
  const std_msgs::ColorRGBA COLOR_GREEN = VisualizationFunctions::ColorRGBA(0, 1, 0, 1).color;
  const std_msgs::ColorRGBA COLOR_BLUE = VisualizationFunctions::ColorRGBA(0, 0, 1, 1).color;
};
}  // namespace agribot_local_planner
