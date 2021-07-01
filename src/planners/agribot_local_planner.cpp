/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/agribot_local_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(agribot_local_planner::AgribotLocalPlanner, nav_core::BaseLocalPlanner)

namespace agribot_local_planner
{
AgribotLocalPlanner::AgribotLocalPlanner()
  : costmap_ros_(NULL)
  , costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons")
  , initialized_(false)
{
}

AgribotLocalPlanner::~AgribotLocalPlanner()
{
}

void AgribotLocalPlanner::createPlanners()
{
  auto circles_planner = std::make_unique<CirclesPlanner>(reference_frame_id_);
  circles_planner->setArcRadius(param_design_arc_radius_);
  circles_planner->setDeadZone(param_design_dead_zone_);
  circles_planner->setStepSize(param_design_step_size_);
  planners_["circles"] = std::move(circles_planner);

  auto hermite_planner = std::make_unique<HermitePlanner>(reference_frame_id_);
  hermite_planner->setStepSize(param_design_step_size_);
  hermite_planner->setMagnitudeStart(param_hermite_magnitude_);
  hermite_planner->setMagnitudeEnd(param_hermite_magnitude_);
  planners_["hermite"] = std::move(hermite_planner);

  auto narrow_planner = std::make_unique<NarrowCorridorPlanner>(obstacle_frame_id_);
  narrow_planner->setChassisLength(param_chassis_length_);
  narrow_planner->setStepSize(param_design_step_size_);
  planners_["narrow"] = std::move(narrow_planner);

  auto straight_path = std::make_unique<StraightPathPlanner>(reference_frame_id_);
  straight_path->setStepSize(param_design_step_size_);
  planners_["straight_path"] = std::move(straight_path);

  planners_fallback_strategy_.push_back("circles");
  planners_fallback_strategy_.push_back("hermite");
  planners_fallback_strategy_.push_back("straight_path");
}

void AgribotLocalPlanner::createValidators()
{
  auto curvature_validator = std::make_unique<agribot_path_validators::CurvatureValidator>();
  curvature_validator->setMaxCurvature(1.0 / param_min_radius_);
  validators_["curvature"] = std::move(curvature_validator);

  auto distance_validator = std::make_unique<agribot_path_validators::DistanceValidator>();
  distance_validator->setMinDistance(param_chassis_length_);
  validators_["distance"] = std::move(distance_validator);
}

void AgribotLocalPlanner::createTrackers()
{
  auto following_tracker = std::make_unique<agribot_path_trackers::FollowingTracker>();
  following_tracker->setGoalDistanceTolerance(param_goal_distance_tolerance_);
  following_tracker->setMissedDistanceCoefficient(param_missed_distance_coefficient_);
  trackers_["following"] = std::move(following_tracker);

  auto front_ray_tracker = std::make_unique<agribot_path_trackers::RayTracker>();
  front_ray_tracker->setRayWidth(default_tracker_front_ray_width);
  front_ray_tracker->setRayLength(default_tracker_front_ray_length);
  front_ray_tracker->setTrackingMethod(false);
  trackers_["front_ray"] = std::move(front_ray_tracker);

  auto side_ray_tracker = std::make_unique<agribot_path_trackers::RayTracker>();
  side_ray_tracker->setRayWidth(default_tracker_side_ray_width);
  side_ray_tracker->setRayLength(default_tracker_side_ray_length);
  side_ray_tracker->setTrackingMethod(true);
  trackers_["side_ray"] = std::move(side_ray_tracker);
}

void AgribotLocalPlanner::createControllers()
{
  auto pure_pursuit_controller = std::make_unique<agribot_path_controllers::PurePursuitController>();
  pure_pursuit_controller->setVelocityLinear(param_max_linear_vel_);
  pure_pursuit_controller->setRobotLength(param_chassis_length_);
  pure_pursuit_controller->setRabbitDistance(param_pure_pursuit_rabbit_distance_);
  pure_pursuit_controller->setMinMotionRadius(param_min_radius_);
  controllers_["PurePursuitController"] = std::move(pure_pursuit_controller);

  auto stanley_controller = std::make_unique<agribot_path_controllers::StanleyController>();
  stanley_controller->setVelocityLinear(param_max_linear_vel_);
  stanley_controller->setErrorGain(param_stanley_error_gain_);
  stanley_controller->setMinMotionRadius(param_min_radius_);
  stanley_controller->setRobotLength(param_chassis_length_);
  controllers_["StanleyController"] = std::move(stanley_controller);

  if (controllers_.find(param_agribot_controller_name_) == controllers_.end())
  {
    throw std::invalid_argument("Contoller not found");
  }
  else
  {
    ROS_INFO_STREAM("[AgribotLocalPlanner] Contoller " << param_agribot_controller_name_ << " loaded");
  }
}

void AgribotLocalPlanner::createCostmapConverter(const ros::NodeHandle& nh)
{
  costmap_converter_ = costmap_converter_loader_.createInstance(costmap_converter_name_);
  std::string converter_name = costmap_converter_loader_.getName(costmap_converter_name_);
  boost::replace_all(converter_name, "::", "/");
  costmap_converter_->setOdomTopic(costmap_converter_odom_topic_);
  costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
  costmap_converter_->setCostmap2D(local_costmap_);
  costmap_converter_->startWorker(ros::Rate(costmap_converter_rate_), local_costmap_, false);
}

void AgribotLocalPlanner::getParameters(const ros::NodeHandle& nh)
{
  param_goal_distance_tolerance_ =
      ros_common_functions::getRosParam<double>(nh.getNamespace() + "/goal_distance_tolerance");
  param_max_missed_waypoints_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/max_missed_waypoints");

  param_maximum_distance_to_path_tolerance_ =
      ros_common_functions::getRosParam<double>(nh.getNamespace() + "/maximum_distance_to_path_tolerance");

  param_chassis_length_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/chassis_length");
  param_min_radius_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/min_radius");

  param_max_linear_vel_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/max_linear_vel");
  param_min_linear_vel_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/min_linear_vel");
  param_epsilon_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/epsilon");

  param_design_dead_zone_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/design_dead_zone");
  param_design_arc_radius_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/design_arc_radius");
  param_hermite_magnitude_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/hermite_magnitude");

  param_odom_filter_time_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/odom_filter_time");
  param_missed_distance_coefficient_ =
      ros_common_functions::getRosParam<double>(nh.getNamespace() + "/missed_distance_coefficient");

  param_statistics_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/statistics");

  param_design_step_size_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/design_step_size");
  costmap_converter_name_ =
      ros_common_functions::getRosParam<std::string>(nh.getNamespace() + "/costmap_converter_plugin");
  costmap_converter_odom_topic_ =
      ros_common_functions::getRosParam<std::string>(nh.getNamespace() + "/costmap_converter_odom_topic");
  costmap_converter_rate_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/costmap_converter_rate");

  param_pure_pursuit_rabbit_distance_ =
      ros_common_functions::getRosParam<double>(nh.getNamespace() + "/pure_pursuit_rabbit_distance");
  param_stanley_error_gain_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/stanley_error_gain");

  param_agribot_controller_name_ =
      ros_common_functions::getRosParam<std::string>(nh.getNamespace() + "/agribot_controller_plugin");
  reference_frame_id_ = ros_common_functions::getRosParam<std::string>(nh.getNamespace() + "/reference_frame_id");
  obstacle_frame_id_ = ros_common_functions::getRosParam<std::string>(nh.getNamespace() + "/obstacle_frame_id");
}

void AgribotLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tfBuffer, costmap_2d::Costmap2DROS* costmapROS)
{
  if (!initialized_)
  {
    ros::NodeHandle gn;
    ros::NodeHandle pnh("~" + name);

    try
    {
      getParameters(pnh);
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("[AgribotLocalPlanner] " << ex.what());
      return;
    }

    costmap_ros_ = costmapROS;
    local_costmap_ = costmapROS->getCostmap();
    tf_buffer_ = tfBuffer;

    path_slice_.header.frame_id = reference_frame_id_;
    robot_center_.header.frame_id = reference_frame_id_;

    odom_sub_ = gn.subscribe("odom", 100, &AgribotLocalPlanner::odomCallback, this);

    path_pub_ = pnh.advertise<nav_msgs::Path>("local_path", 10);
    predicted_path_pub_ = pnh.advertise<nav_msgs::Path>("predicted_path", 10);
    path_slice_pub_ = pnh.advertise<nav_msgs::Path>("path_slice", 10);
    obstacles_pub_ = pnh.advertise<costmap_converter::ObstacleMsg>("obstacles", 10);
    obstacles_left_polygon_pub_ = pnh.advertise<visualization_msgs::Marker>("obstacles_left_polygon", 10);
    obstacles_right_polygon_pub_ = pnh.advertise<visualization_msgs::Marker>("obstacles_right_polygon", 10);
    obstacles_polygon_pub_ = pnh.advertise<visualization_msgs::Marker>("obstacles_polygon", 10);
    obstacles_left_line_pub_ = pnh.advertise<nav_msgs::Path>("obstacles_left_line", 10);
    obstacles_right_line_pub_ = pnh.advertise<nav_msgs::Path>("obstacles_right_line", 10);
    rabbit_pub_ = pnh.advertise<geometry_msgs::PoseStamped>("rabbit_pose", 10);
    goal_pub_ = pnh.advertise<visualization_msgs::Marker>("goal_tolerance", 10);

    // be sure that path planning arc radius is greater or equal to minimum turning radius
    if (param_design_arc_radius_ < param_min_radius_)
    {
      ROS_WARN_STREAM("[AgribotLocalPlanner] param design_arc_radius to small. Taking param_min_radius = "
                      << param_min_radius_ << " instead");
      param_design_arc_radius_ = param_min_radius_;
    }

    try
    {
      createPlanners();
      createValidators();
      createControllers();
      createCostmapConverter(pnh);
    }
    catch (const std::exception& ex)
    {
      ROS_ERROR_STREAM("[AgribotLocalPlanner] " << ex.what());
      return;
    }

    last_odom_stamp_ = 0;
    is_out_of_path_ = false;

    ros::topic::waitForMessage<nav_msgs::Odometry>("odom");

    initialized_ = true;

    if (param_statistics_)
    {
      stop_time_ = 0;
      start_time_ = 0;
      last_time_ = 0;
    }

    ROS_DEBUG_STREAM("[AgribotLocalPlanner] Plugin initialized.");
  }
  else
  {
    ROS_WARN_STREAM("[AgribotLocalPlanner] This planner has already been initialized, doing nothing.");
  }
}

bool AgribotLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  ROS_INFO_STREAM("[AgribotLocalPlanner] Plan received from Global Planner. Initialization.");

  if (!initialized_)
  {
    ROS_ERROR_STREAM("[AgribotLocalPlanner] Planner not initialized, call initialize() before planner");
    return false;
  }

  goal_ = orig_global_plan.back();
  goal_.header.stamp = ros::Time::now();
  goal_ = transformPose(goal_, reference_frame_id_);

  last_odom_stamp_ = 0;
  ros::topic::waitForMessage<nav_msgs::Odometry>("odom");

  if (is_out_of_path_)
  {
    ROS_ERROR_STREAM("[AgribotLocalPlanner] Prevent replanning. Abort planning.");
    is_out_of_path_ = false;
    return false;
  }

  // make local plan
  for (auto const& planner_name : planners_fallback_strategy_)
  {
    ROS_INFO_STREAM("[AgribotLocalPlanner] Planner " << planner_name << " taken.");
    if (planPath(planner_name, robot_center_, goal_))
    {
      ROS_INFO_STREAM("[AgribotLocalPlanner] Planner " << planner_name << " created valid path.");
      break;
    }
    else
    {
      ROS_WARN_STREAM("[AgribotLocalPlanner] Planner "
                      << planner_name << " is not able to create valid path. Taking the next planner.");
      if (planner_name == planners_fallback_strategy_.back())
      {
        ROS_ERROR_STREAM("[AgribotLocalPlanner] Unable to create valid path. Abort planning.");
        return false;
      }
    }
  }

  // set flags and reset additional variables
  abort_executing_ = false;
  goal_reached_ = false;
  traveled_points_ = visualization_msgs::Marker();
  traveled_distance_ = 0;
  out_of_path_counter_ = 0;
  planners_["narrow"]->clearObstacles();

  if (param_statistics_)
  {
    start_time_ = ros::Time::now().toSec();
    last_time_ = 0;
  }

  ROS_INFO_STREAM("[AgribotLocalPlanner] Plan received from Global Planner. Initialization finished.");

  return true;
}

bool AgribotLocalPlanner::planPath(const std::string& planner_id, const geometry_msgs::PoseStamped& start_pose,
                                   const geometry_msgs::PoseStamped& end_pose)
{
  // make local plan
  local_plan_.clear();
  try
  {
    if (!planners_[planner_id]->designPath(local_plan_, start_pose, end_pose))
    {
      ROS_WARN_STREAM("[AgribotLocalPlanner][" << planner_id << "_planner] Local planner is not able to create path ");
      return false;
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("[AgribotLocalPlanner] " << e.what());
    ROS_WARN_STREAM("[AgribotLocalPlanner][" << planner_id << "_planner] Local planner is not able to create path ");
    return false;
  }

  visualizationPath();

  // validate
  ROS_INFO_STREAM("[AgribotLocalPlanner] Validating path");
  for (auto const& [validator_id, validator] : validators_)
  {
    if (!validator->validatePath(local_plan_))
    {
      ROS_WARN_STREAM("[AgribotLocalPlanner][" << planner_id << "_planner] Local path not passed " << validator_id
                                               << " validation");
      return false;
    }
  }
  ROS_INFO_STREAM("[AgribotLocalPlanner] Validating path completed");

  ROS_INFO_STREAM("[AgribotLocalPlanner][" << planner_id
                                           << "_planner] Local plan created, path length = " << local_plan_.size());
  // reset controllers
  for (auto const& [controller_id, controller] : controllers_)
  {
    static_cast<void>(controller_id);
    controller->reset();
  }

  return true;
}

bool AgribotLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR_STREAM("[AgribotLocalPlanner] Planner not initialized, call initialize() before planner");
    return false;
  }

  if (local_plan_.size() <= 0)
  {
    ROS_ERROR("[AgribotLocalPlanner] Local plan is not created");
    return false;
  }

  checkObstacles();
  if (detection_.isInRow(planners_["narrow"]->getObstacles()))
  {
    local_plan_.clear();
    try
    {
      planners_["narrow"]->designPath(local_plan_, robot_center_, goal_);
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("[AgribotLocalPlanner]" << e.what());
      abort_executing_ = true;
    }
    publishObstacles();
  }

  // Calculate velocities
  calculateControls(cmd_vel);

  // Stop condition
  if (isReachedPathEnd(robot_center_, transformPose(local_plan_.back(), reference_frame_id_)))
  {
    cmd_vel = finishedPathBehaviour();
  }

  if (isOutOfPath())
  {
    out_of_path_counter_++;
  }

  visualizationPath();

  if (abort_executing_)
  {
    cmd_vel = stopRobot();
    local_plan_.clear();
    ROS_ERROR("[AgribotLocalPlanner] Abort plan");
    return false;
  }

  if (out_of_path_counter_ > param_max_missed_waypoints_)
  {
    out_of_path_counter_ = 0;
    is_out_of_path_ = true;
    cmd_vel = stopRobot();
    local_plan_.clear();
    ROS_ERROR_STREAM("[AgribotLocalPlanner] Out of path. Abort plan");
    return false;
  }

  return true;
}

bool AgribotLocalPlanner::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR_STREAM("[AgribotLocalPlanner] Planner not initialized, call initialize() before planner");
    return false;
  }
  return goal_reached_;
}

void AgribotLocalPlanner::checkObstacles()
{
  detection_.setRobotPose(transformPose(robot_center_, obstacle_frame_id_));
  obstacles_array_ = costmap_converter_->getObstacles();
  if (obstacles_array_->obstacles.size() > 0)
  {
    processObstacles(obstacles_array_);
  }
}

void AgribotLocalPlanner::processObstacles(const costmap_converter::ObstacleArrayConstPtr& obstacles)
{
  obstacles_left_.initializeEmpty();
  obstacles_right_.initializeEmpty();
  planners_["narrow"]->clearObstacles();
  obstacles_.clear();
  for (const auto& obstacle : obstacles->obstacles)
  {
    if (obstacle.polygon.points.size() > 2)
    {
      geometry_msgs::PolygonStamped polygon, transformed_polygon;
      polygon.header.frame_id = "odom";
      polygon.header.stamp = ros::Time::now();
      polygon.polygon = obstacle.polygon;
      transformed_polygon = transformPolygon(polygon, obstacle_frame_id_);

      auto obstacle_tmp = ObstacleDetection::Obstacle(transformed_polygon);
      planners_["narrow"]->addObstacle(obstacle_tmp);
      obstacles_.push_back(obstacle_tmp);
    }
  }
}

void AgribotLocalPlanner::publishObstacles()
{
  try
  {
    auto obstacles = planners_["narrow"]->getObstacles();
    if (obstacles.size() == 0)
    {
      return;
    }
    obstacles_left_ = obstacles[0];
    obstacles_right_ = obstacles[1];
    visualizationObstacles();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("[AgribotLocalPlanner]" << e.what());
  }
}

void AgribotLocalPlanner::truncatePath(std::vector<geometry_msgs::PoseStamped>& truncated_path)
{
  std::vector<geometry_msgs::PoseStamped> path_slice_ray;
  geometry_msgs::PoseStamped robot_center = transformPose(robot_center_, reference_frame_id_);

  if (!trackers_["side_ray"]->findPathSlice(path_slice_ray, local_plan_, robot_center))
  {
    trackers_["front_ray"]->findPathSlice(path_slice_ray, local_plan_, robot_center);
  }

  trackers_["following"]->findPathSlice(truncated_path, local_plan_, robot_center);

  if (path_slice_ray.size() > 1)
  {
    truncated_path = path_slice_ray;
  }
}

void AgribotLocalPlanner::calculateControls(geometry_msgs::Twist& cmd_vel)
{
  controllers_[param_agribot_controller_name_]->computeControl(cmd_vel, local_plan_, robot_center_);
  cmd_vel = limitVelocities(cmd_vel);
}

geometry_msgs::Twist AgribotLocalPlanner::limitVelocities(geometry_msgs::Twist velocities)
{
  if (velocities.linear.x > param_max_linear_vel_)
  {
    velocities.linear.x = param_max_linear_vel_;
  }
  if (velocities.linear.x < param_min_linear_vel_)
  {
    velocities.linear.x = param_min_linear_vel_;
  }
  if (velocities.angular.z > velocities.linear.x / param_min_radius_)
  {
    velocities.angular.z = velocities.linear.x / param_min_radius_;
  }
  if (velocities.angular.z < -velocities.linear.x / param_min_radius_)
  {
    velocities.angular.z = -velocities.linear.x / param_min_radius_;
  }

  velocities.linear.x = closeToZero(velocities.linear.x);
  velocities.linear.y = closeToZero(velocities.linear.y);
  velocities.linear.z = closeToZero(velocities.linear.z);
  velocities.angular.x = closeToZero(velocities.angular.x);
  velocities.angular.y = closeToZero(velocities.angular.y);
  velocities.angular.z = closeToZero(velocities.angular.z);
  return velocities;
}

double AgribotLocalPlanner::closeToZero(const double value)
{
  if (fabs(value) < param_epsilon_)
  {
    return 0.0;
  }
  else
  {
    return value;
  }
}

geometry_msgs::Twist AgribotLocalPlanner::finishedPathBehaviour()
{
  goal_reached_ = true;
  local_plan_.clear();
  if (param_statistics_)
  {
    stop_time_ = ros::Time::now().toSec();
    double total_time = stop_time_ - start_time_;
    ROS_INFO_STREAM("[AgribotLocalPlanner] Path finished successfully");
    ROS_INFO_STREAM("[AgribotLocalPlanner] Time      = " << total_time << " s");
    ROS_INFO_STREAM("[AgribotLocalPlanner] Distance  = " << traveled_distance_ << " m");
    ROS_INFO_STREAM("[AgribotLocalPlanner] Avg speed = " << traveled_distance_ / total_time << " m/s");
  }
  return stopRobot();
}

void AgribotLocalPlanner::odomCallback(const nav_msgs::Odometry& odometry_msg)
{
  // prevent from too frequent odom reading
  double odom_time_stamp = odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nsec / 10e9;
  if (odom_time_stamp - last_odom_stamp_ > param_odom_filter_time_)
  {
    last_odom_stamp_ = odom_time_stamp;
    geometry_msgs::PoseStamped lastRobotPosition = robot_center_;
    geometry_msgs::PoseStamped robotPose;

    robotPose.header = odometry_msg.header;
    robotPose.pose = odometry_msg.pose.pose;
    robot_center_ = transformPose(robotPose, reference_frame_id_);

    traveled_distance_ += CommonFunctions::distanceBetweenPoses(lastRobotPosition, robot_center_);
  }
}

geometry_msgs::Twist AgribotLocalPlanner::stopRobot()
{
  geometry_msgs::Twist velocity;
  velocity.linear.x = 0;
  velocity.linear.y = 0;
  velocity.angular.z = 0;
  return velocity;
}

geometry_msgs::PoseStamped AgribotLocalPlanner::transformPose(const geometry_msgs::PoseStamped& pose,
                                                              const std::string& frame_id)
{
  geometry_msgs::PoseStamped pose_transformed;
  pose_transformed.header.frame_id = frame_id;

  try
  {
    pose_transformed.header.stamp = ros::Time::now();
    tf_buffer_->transform(pose, pose_transformed, frame_id, ros::Duration(0.2));
  }
  catch (tf2::ExtrapolationException e)
  {
    ROS_ERROR_STREAM("[AgribotLocalPlanner] ExtrapolationException: " << e.what());
    abort_executing_ = true;
  }
  return pose_transformed;
}

geometry_msgs::PolygonStamped AgribotLocalPlanner::transformPolygon(const geometry_msgs::PolygonStamped& pose,
                                                                    const std::string& frame_id)
{
  geometry_msgs::PolygonStamped polygon_transformed;
  polygon_transformed.header.frame_id = frame_id;
  polygon_transformed.header.stamp = ros::Time::now();
  polygon_transformed.polygon.points.clear();

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = pose.header;
  for (const auto& point : pose.polygon.points)
  {
    pose_stamped.pose.position.x = point.x;
    pose_stamped.pose.position.y = point.y;
    geometry_msgs::PoseStamped pose_transformed = transformPose(pose_stamped, frame_id);
    geometry_msgs::Point32 point_transformed;
    point_transformed.x = pose_transformed.pose.position.x;
    point_transformed.y = pose_transformed.pose.position.y;
    polygon_transformed.polygon.points.push_back(point_transformed);
  }
  return polygon_transformed;
}

bool AgribotLocalPlanner::isTheSameGoalAsPrevious(const geometry_msgs::PoseStamped& goal)
{
  bool isTheSame = false;
  if (goal_previous_ == goal)
  {
    isTheSame = true;
  }
  goal_previous_ = goal;
  return isTheSame;
}

void AgribotLocalPlanner::visualizationPath(void)
{
  VisualizationFunctions::pathVisualization(local_plan_, reference_frame_id_, path_pub_);
  // VisualizationFunctions::pathVisualization(path_slice_.poses, reference_frame_id_, path_slice_pub_);
  // VisualizationFunctions::pathVisualization(trajectory_, reference_frame_id_, path_slice_pub_);
  // VisualizationFunctions::predictedPathVisualization(
  //     cmd_vel, CommonFunctions::pCoordinatesFromPoseStamped(robot_center_), param_prediction_points_per_second_,
  //     param_prediction_time_, reference_frame_id_, predicted_path_pub_);
  // auto pure_pursuit_pointer =
  //     dynamic_cast<agribot_path_controllers::PurePursuitController*>(controllers_["pure_pursuit"].get());
  // if (pure_pursuit_pointer != nullptr)
  // {
  //   VisualizationFunctions::poseVisualization(pure_pursuit_pointer->getRabbitPose(), rabbit_pub_);
  // }

  auto stanley_pointer =
      dynamic_cast<agribot_path_controllers::StanleyController*>(controllers_["StanleyController"].get());
  if (stanley_pointer != nullptr)
  {
    VisualizationFunctions::poseVisualization(stanley_pointer->getProjectionPose(), rabbit_pub_);
  }

  // try
  // {
  //   VisualizationFunctions::goalVisualization(local_plan_.back(), param_goal_distance_tolerance_, goal_pub_);
  // }
  // catch (const std::exception& ex)
  // {
  //   ROS_ERROR_STREAM("[AgribotLocalPlanner] " << ex.what());
  // }
}

void AgribotLocalPlanner::visualizationObstacles(void)
{
  VisualizationFunctions::pathVisualization(obstacles_left_.toPath(param_design_step_size_), obstacle_frame_id_,
                                            obstacles_left_line_pub_);
  VisualizationFunctions::pathVisualization(obstacles_right_.toPath(param_design_step_size_), obstacle_frame_id_,
                                            obstacles_right_line_pub_);

  VisualizationFunctions::obstacleVisualization(obstacles_left_, obstacles_left_polygon_pub_, COLOR_GREEN);
  VisualizationFunctions::obstacleVisualization(obstacles_right_, obstacles_right_polygon_pub_, COLOR_BLUE);
  VisualizationFunctions::obstaclesVisualization(obstacles_, obstacles_polygon_pub_);
}

bool AgribotLocalPlanner::isOutOfPath(void)
{
  return CommonFunctions::calculateCrosstrackError(local_plan_, robot_center_) >
         param_maximum_distance_to_path_tolerance_;
}

bool AgribotLocalPlanner::isReachedPathEnd(const geometry_msgs::PoseStamped& robot_coordinates,
                                           const geometry_msgs::PoseStamped& path_end_waypoint)
{
  double distanceToGoal = CommonFunctions::distanceBetweenPoses(path_end_waypoint, robot_coordinates);
  return (distanceToGoal <= param_goal_distance_tolerance_);
}

}  // namespace agribot_local_planner
