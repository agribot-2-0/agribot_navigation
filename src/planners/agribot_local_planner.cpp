/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/agribot_local_planner.h"

#include <agribot_common/CommonFunctions/ros_common_functions.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(agribot_local_planner::AgribotPlannerROS, nav_core::BaseLocalPlanner)

namespace agribot_local_planner
{
AgribotPlannerROS::AgribotPlannerROS()
  : costmap_ros_(NULL)
  , costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons")
  , initialized_(false)
{
}

AgribotPlannerROS::~AgribotPlannerROS()
{
}

void AgribotPlannerROS::createPlanners()
{
  auto circles_planner = std::make_unique<CirclesPlanner>(robot_center_.header.frame_id);
  circles_planner->setArcRadius(default_design_arc_radius);
  circles_planner->setDeadZone(default_design_dead_zone);
  circles_planner->setStepSize(param_design_step_size_);
  planners_["circles"] = std::move(circles_planner);

  auto hermite_planner = std::make_unique<HermitePlanner>(robot_center_.header.frame_id);
  hermite_planner->setStepSize(param_design_step_size_);
  hermite_planner->setMagnitudeStart(default_hermite_magnitude);
  hermite_planner->setMagnitudeEnd(default_hermite_magnitude);
  planners_["hermite"] = std::move(hermite_planner);

  auto narrow_planner = std::make_unique<NarrowCorridorPlanner>(obstacle_frame_id_);
  narrow_planner->setChassisLength(param_chassis_length_);
  narrow_planner->setStepSize(param_design_step_size_);
  planners_["narrow"] = std::move(narrow_planner);
}

void AgribotPlannerROS::createValidators()
{
  auto curvature_validator = std::make_unique<agribot_path_validators::CurvatureValidator>();
  curvature_validator->setMaxCurvature(1.0 / default_min_radius);
  validators_["curvature"] = std::move(curvature_validator);

  auto distance_validator = std::make_unique<agribot_path_validators::DistanceValidator>();
  distance_validator->setMinDistance(default_chassis_length);
  validators_["distance"] = std::move(distance_validator);
}

void AgribotPlannerROS::createTrackers()
{
  auto following_tracker = std::make_unique<agribot_path_trackers::FollowingTracker>();
  following_tracker->setGoalDistanceTolerance(default_goal_distance_tolerance);
  following_tracker->setMissedDistanceCoefficient(default_missed_distance_coefficient);
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

void AgribotPlannerROS::createControllers()
{
  auto pure_pursuit_controller = std::make_unique<agribot_path_controllers::PurePursuitController>();
  pure_pursuit_controller->setVelocityLinear(param_max_linear_vel_);
  pure_pursuit_controller->setRobotLength(default_chassis_length);
  pure_pursuit_controller->setRabbitDistance(default_pure_pursuit_rabbit_distance);
  controllers_["pure_pursuit"] = std::move(pure_pursuit_controller);

  auto stanley_controller = std::make_unique<agribot_path_controllers::StanleyController>();
  stanley_controller->setVelocityLinear(default_max_linear_vel);
  stanley_controller->setErrorGain(default_stanley_error_gain);
  stanley_controller->setThetaLimit(asin(default_chassis_length / 2 / default_min_radius));
  controllers_["stanley"] = std::move(stanley_controller);

  auto deviation_parallelness_controller =
      std::make_unique<agribot_path_controllers::DeviationParallelnessController>();
  deviation_parallelness_controller->setSpeedLimiting(true);
  deviation_parallelness_controller->setParallelnessWeight(default_parallelness_weight_straight);
  deviation_parallelness_controller->setDeviationWeight(default_deviation_weight_straight);
  deviation_parallelness_controller->setMinRadius(default_min_radius);
  deviation_parallelness_controller->setMaxLinearVelocity(default_max_linear_vel);
  deviation_parallelness_controller->setMinLinearVelocity(default_min_linear_vel);
  deviation_parallelness_controller->setEpsilon(default_epsilon);
  controllers_["deviation_parallelness"] = std::move(deviation_parallelness_controller);
}

void AgribotPlannerROS::createCostmapConverter(const ros::NodeHandle& nh)
{
  costmap_converter_ = costmap_converter_loader_.createInstance(costmap_converter_name_);
  std::string converter_name = costmap_converter_loader_.getName(costmap_converter_name_);
  boost::replace_all(converter_name, "::", "/");
  costmap_converter_->setOdomTopic(costmap_converter_odom_topic_);
  costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
  costmap_converter_->setCostmap2D(local_costmap_);
  costmap_converter_->startWorker(ros::Rate(costmap_converter_rate_), local_costmap_, false);
}

void AgribotPlannerROS::getParameters(const ros::NodeHandle& nh)
{
  param_goal_distance_tolerance_ =
      ros_common_functions::getRosParam<double>(nh.getNamespace() + "/goal_distance_tolerance");
  param_goal_range_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/goal_range");
  param_missed_points_tolerance_ =
      ros_common_functions::getRosParam<double>(nh.getNamespace() + "/missed_points_tolerance");

  param_chassis_length_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/chassis_length");
  param_min_radius_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/min_radius");

  param_prediction_time_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/prediction_time");
  param_prediction_points_per_second_ =
      ros_common_functions::getRosParam<double>(nh.getNamespace() + "/prediction_points_per_second");

  param_dead_zone_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/dead_zone");
  param_max_linear_vel_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/max_linear_vel");
  param_min_linear_vel_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/min_linear_vel");

  param_deviation_weight_straight_ =
      ros_common_functions::getRosParam<double>(nh.getNamespace() + "/deviation_weight_straight");
  param_parallelness_weight_straight_ =
      ros_common_functions::getRosParam<double>(nh.getNamespace() + "/parallelness_weight_straight");
  param_deviation_weight_arc_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/deviation_weight_arc");
  param_parallelness_weight_arc_ =
      ros_common_functions::getRosParam<double>(nh.getNamespace() + "/parallelness_weight_arc");
  param_epsilon_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/epsilon");

  param_design_dead_zone_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/design_dead_zone");
  param_design_arc_radius_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/design_arc_radius");
  param_hermite_magnitude_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/hermite_magnitude");

  param_odom_filter_time_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/odom_filter_time");
  param_missed_distance_coefficient_ =
      ros_common_functions::getRosParam<double>(nh.getNamespace() + "/missed_distance_coefficient");

  param_speed_limit_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/speed_limit");
  param_debug_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/debug");
  param_statistics_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/statistics");
  param_take_front_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/take_front");

  param_design_step_size_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/design_step_size");
  costmap_converter_name_ =
      ros_common_functions::getRosParam<std::string>(nh.getNamespace() + "/costmap_converter_plugin");
  costmap_converter_odom_topic_ =
      ros_common_functions::getRosParam<std::string>(nh.getNamespace() + "/costmap_converter_odom_topic");
  costmap_converter_rate_ = ros_common_functions::getRosParam<double>(nh.getNamespace() + "/costmap_converter_rate");

  reference_frame_id_ = ros_common_functions::getRosParam<std::string>(nh.getNamespace() + "/reference_frame_id");
  obstacle_frame_id_ = ros_common_functions::getRosParam<std::string>(nh.getNamespace() + "/obstacle_frame_id");
}

void AgribotPlannerROS::initialize(std::string name, tf2_ros::Buffer* tfBuffer, costmap_2d::Costmap2DROS* costmapROS)
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
      ROS_ERROR_STREAM("[AgribotPlanner] " << ex.what());
    }

    costmap_ros_ = costmapROS;
    local_costmap_ = costmapROS->getCostmap();
    tf_buffer_ = tfBuffer;

    local_path_.header.frame_id = reference_frame_id_;
    path_slice_.header.frame_id = reference_frame_id_;
    robot_center_.header.frame_id = reference_frame_id_;

    odom_sub_ = gn.subscribe("odom", 100, &AgribotPlannerROS::odomCallback, this);

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

    // be sure that path planning arc radius is greater or equal to minimum turning radius
    if (param_design_arc_radius_ < param_min_radius_)
    {
      ROS_WARN_STREAM("[AgribotPlanner] param design_arc_radius to small. Taking param_min_radius = "
                      << param_min_radius_ << " instead");
      param_design_arc_radius_ = param_min_radius_;
    }

    createPlanners();
    createTrackers();
    createValidators();
    createControllers();
    createCostmapConverter(pnh);

    last_odom_stamp_ = 0;
    ros::topic::waitForMessage<nav_msgs::Odometry>("odom");

    initialized_ = true;

    if (param_statistics_)
    {
      stop_time_ = 0;
      start_time_ = 0;
      last_time_ = 0;
    }

    ROS_DEBUG_STREAM("[AgribotPlanner] Plugin initialized.");
  }
  else
  {
    ROS_WARN_STREAM("[AgribotPlanner] This planner has already been initialized, doing nothing.");
  }
}

bool AgribotPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  ROS_INFO_STREAM("[AgribotPlanner] Plan received from Global Planner. Initialization.");

  if (!initialized_)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, call initialize() before planner");
    return false;
  }

  // remember global plan
  global_plan_ = orig_global_plan;

  last_odom_stamp_ = 0;
  ros::topic::waitForMessage<nav_msgs::Odometry>("odom");

  // make local plan
  if (!planPath(local_plan_, robot_center_, transformPose(global_plan_.back(), reference_frame_id_), "circles"))
  {
    if (!planPath(local_plan_, robot_center_, transformPose(global_plan_.back(), reference_frame_id_), "hermite"))
    {
      ROS_ERROR_STREAM("[AgribotPlanner] Unable to create valid path");
      return false;
    }
  }

  // set flags and reset additional variables
  abort_executing_ = false;
  goal_reached_ = false;
  traveled_points_ = visualization_msgs::Marker();
  traveled_distance_ = 0;
  planners_["narrow"]->clearObstacles();

  if (param_statistics_)
  {
    start_time_ = ros::Time::now().toSec();
    last_time_ = 0;
  }

  ROS_INFO_STREAM("[AgribotPlanner] Plan received from Global Planner. Finish Initialization.");

  return true;
}

bool AgribotPlannerROS::planPath(std::vector<geometry_msgs::PoseStamped>& local_plan,
                                 const geometry_msgs::PoseStamped& start_pose,
                                 const geometry_msgs::PoseStamped& end_pose, const std::string& planner_id)
{
  // make local plan
  local_plan.clear();
  if (!planners_[planner_id]->designPath(local_plan, start_pose, end_pose))
  {
    ROS_ERROR_STREAM("[AgribotPlanner][" << planner_id << "_planner] Local planner unable to create path ");
    return false;
  }
  else
  {
    // validate
    if (!validators_["distance"]->validatePath(local_plan))
    {
      ROS_ERROR_STREAM("[AgribotPlanner][" << planner_id << "_planner] Local path is invalid - too short");
      return false;
    }
    ROS_INFO_STREAM("[AgribotPlanner][" << planner_id
                                        << "_planner] Local planner created, path length = " << local_plan.size());
  }

  // reset trackers
  for (auto const& [tracker_id, tracker] : trackers_)
  {
    static_cast<void>(tracker_id);
    tracker->reset();
  }

  // reset controllers
  for (auto const& [controller_id, controller] : controllers_)
  {
    static_cast<void>(controller_id);
    controller->reset();
  }

  return true;
}

bool AgribotPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!initialized_)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, call initialize() before planner");
    return false;
  }

  if (local_plan_.size() <= 0)
  {
    ROS_ERROR("[AgribotPlanner] Local plan is not created");
    abort_executing_ = true;
  }

  checkObstacles();
  if (detection_.isInRow(planners_["narrow"]->getObstacles()))
  {
    local_plan_.clear();
    try
    {
      planners_["narrow"]->designPath(local_plan_, robot_center_,
                                      transformPose(global_plan_.back(), reference_frame_id_));
    }
    catch (const std::exception& e)
    {
      ROS_ERROR_STREAM("[AgribotPlanner]" << e.what());
      abort_executing_ = true;
    }
  }

  truncatePath(path_slice_.poses);

  if (path_slice_.poses.size() < 2)
  {
    cmd_vel = stopRobot();
    if (!planPath(local_plan_, robot_center_, transformPose(global_plan_.back(), reference_frame_id_), "circles"))
    {
      ROS_ERROR_STREAM("[AgribotPlanner] Unable to replan path");
      abort_executing_ = true;
    }
    return true;
  }

  // Calculate velocities
  calculateControls(cmd_vel);

  // Stop condition
  if (isReachedPathEnd(
          CommonFunctions::pCoordinatesFromPoseStamped(robot_center_),
          CommonFunctions::pCoordinatesFromPoseStamped(transformPose(local_plan_.back(), reference_frame_id_))))
  {
    cmd_vel = finishedPathBehaviour();
  }

  VisualizationFunctions::pathVisualization(local_plan_, reference_frame_id_, path_pub_);
  VisualizationFunctions::pathVisualization(path_slice_.poses, reference_frame_id_, path_slice_pub_);
  VisualizationFunctions::pathVisualization(trajectory_, reference_frame_id_, path_slice_pub_);
  VisualizationFunctions::predictedPathVisualization(
      cmd_vel, CommonFunctions::pCoordinatesFromPoseStamped(robot_center_), param_prediction_points_per_second_,
      param_prediction_time_, reference_frame_id_, predicted_path_pub_);
  auto pure_pursuit_pointer =
      dynamic_cast<agribot_path_controllers::PurePursuitController*>(controllers_["pure_pursuit"].get());
  if (pure_pursuit_pointer != nullptr)
  {
    VisualizationFunctions::poseVisualization(pure_pursuit_pointer->getRabbitPose(), rabbit_pub_);
  }
  publishObstacles();

  if (abort_executing_)
  {
    cmd_vel = stopRobot();
    local_path_.poses.clear();
    ROS_ERROR("[AgribotPlanner] Abort plan");
    return false;
  }

  return true;
}

bool AgribotPlannerROS::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, call initialize() before planner");
    return false;
  }
  return goal_reached_;
}

void AgribotPlannerROS::checkObstacles()
{
  detection_.setRobotPose(transformPose(robot_center_, obstacle_frame_id_));
  obstacles_array_ = costmap_converter_->getObstacles();
  if (obstacles_array_->obstacles.size() > 0)
  {
    processObstacles(obstacles_array_);
  }
}

void AgribotPlannerROS::processObstacles(const costmap_converter::ObstacleArrayConstPtr& obstacles)
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

void AgribotPlannerROS::publishObstacles()
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
    VisualizationFunctions::pathVisualization(obstacles_left_.toPath(param_design_step_size_), obstacle_frame_id_,
                                              obstacles_left_line_pub_);
    VisualizationFunctions::pathVisualization(obstacles_right_.toPath(param_design_step_size_), obstacle_frame_id_,
                                              obstacles_right_line_pub_);

    VisualizationFunctions::obstacleVisualization(obstacles_left_, obstacles_left_polygon_pub_, COLOR_GREEN);
    VisualizationFunctions::obstacleVisualization(obstacles_right_, obstacles_right_polygon_pub_, COLOR_BLUE);
    VisualizationFunctions::obstaclesVisualization(obstacles_, obstacles_polygon_pub_);
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("[AgribotPlanner]" << e.what());
  }
}

void AgribotPlannerROS::truncatePath(std::vector<geometry_msgs::PoseStamped>& truncated_path)
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

void AgribotPlannerROS::calculateControls(geometry_msgs::Twist& cmd_vel)
{
  controllers_["pure_pursuit"]->computeControl(cmd_vel, local_plan_, robot_center_);
  cmd_vel = limitVelocities(cmd_vel);
}

geometry_msgs::Twist AgribotPlannerROS::limitVelocities(geometry_msgs::Twist velocities)
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

double AgribotPlannerROS::closeToZero(const double value)
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

bool AgribotPlannerROS::isReachedPathEnd(const pos& robot_coordinates, const pos& path_end_waypoint)
{
  double distanceToGoal = CommonFunctions::dDistanceBetweenCoordinates(path_end_waypoint, robot_coordinates);
  return (distanceToGoal <= param_goal_distance_tolerance_);
}

geometry_msgs::Twist AgribotPlannerROS::finishedPathBehaviour()
{
  goal_reached_ = true;
  local_plan_.clear();
  if (param_statistics_)
  {
    stop_time_ = ros::Time::now().toSec();
    ROS_INFO("[AgribotPlanner] T = %f;l¸= %f", stop_time_ - start_time_, traveled_distance_);
  }
  return stopRobot();
}

void AgribotPlannerROS::odomCallback(const nav_msgs::Odometry& odometry_msg)
{
  // prevent from too frequent odom reading
  double odom_time_stamp = odometry_msg.header.stamp.sec + odometry_msg.header.stamp.nsec / 10e9;
  if (odom_time_stamp - last_odom_stamp_ > param_odom_filter_time_)
  {
    last_odom_stamp_ = odom_time_stamp;
    pos lastRobotPosition = CommonFunctions::pCoordinatesFromPoseStamped(robot_center_);
    geometry_msgs::PoseStamped robotPose;

    robotPose.header = odometry_msg.header;
    robotPose.pose = odometry_msg.pose.pose;
    robot_center_ = transformPose(robotPose, reference_frame_id_);

    traveled_distance_ += CommonFunctions::dDistanceBetweenCoordinates(
        lastRobotPosition, CommonFunctions::pCoordinatesFromPoseStamped(robot_center_));
  }
}

geometry_msgs::Twist AgribotPlannerROS::stopRobot()
{
  geometry_msgs::Twist velocity;
  velocity.linear.x = 0;
  velocity.linear.y = 0;
  velocity.angular.z = 0;
  return velocity;
}

geometry_msgs::PoseStamped AgribotPlannerROS::transformPose(const geometry_msgs::PoseStamped& pose,
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
    ROS_ERROR_STREAM("[AgribotPlanner] ExtrapolationException occur");
    abort_executing_ = true;
  }
  return pose_transformed;
}

geometry_msgs::PolygonStamped AgribotPlannerROS::transformPolygon(const geometry_msgs::PolygonStamped& pose,
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

}  // namespace agribot_local_planner
