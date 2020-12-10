/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/agribot_local_planner.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(agribot_local_planner::AgribotPlannerROS, nav_core::BaseLocalPlanner)

namespace agribot_local_planner
{
AgribotPlannerROS::AgribotPlannerROS() : costmap_ros_(NULL), initialized_(false)
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
  circles_planner->setStepSize(default_step_size);
  planners_["circles"] = std::move(circles_planner);

  auto hermite_planner = std::make_unique<HermitePlanner>(robot_center_.header.frame_id);
  hermite_planner->setStepSize(default_step_size);
  hermite_planner->setMagnitudeStart(default_hermite_magnitude);
  hermite_planner->setMagnitudeEnd(default_hermite_magnitude);
  planners_["hermite"] = std::move(hermite_planner);
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
  pure_pursuit_controller->setVelocityLinear(default_max_linear_vel);
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

void AgribotPlannerROS::initialize(std::string name, tf2_ros::Buffer* tfBuffer, costmap_2d::Costmap2DROS* costmapROS)
{
  if (!initialized_)
  {
    costmap_ros_ = costmapROS;
    tf_buffer_ = tfBuffer;
    tf_listener_ = new tf2_ros::TransformListener(*tfBuffer);

    ros::NodeHandle gn;
    ros::NodeHandle pnh("~" + name);

    odom_sub_ = gn.subscribe("odom", 100, &AgribotPlannerROS::vOdomCallback, this);
    path_pub_ = pnh.advertise<nav_msgs::Path>("local_path", 10);
    predicted_path_pub_ = pnh.advertise<nav_msgs::Path>("predicted_path", 10);
    path_slice_pub_ = pnh.advertise<nav_msgs::Path>("path_slice", 10);
    reference_frame_id_ = "map";
    local_path_.header.frame_id = "map";
    path_slice_.header.frame_id = "map";

    pnh.param("goal_distance_tolerance", param_goal_distance_tolerance_, default_goal_distance_tolerance);
    pnh.param("goal_range", param_goal_range_, default_goal_range);
    pnh.param("missed_points_tolerance", param_missed_points_tolerance_, default_missed_points_tolerance);

    pnh.param("chassis_length", param_chassis_length_, default_chassis_length);
    pnh.param("min_radius", param_min_radius_, default_min_radius);

    pnh.param("prediction_time", param_prediction_time_, default_prediction_time);
    pnh.param("prediction_points_per_second", param_prediction_points_per_second_,
              default_prediction_points_per_second);

    pnh.param("dead_zone", param_dead_zone_, default_dead_zone);
    pnh.param("max_linear_vel", param_max_linear_vel_, default_max_linear_vel);
    pnh.param("min_linear_vel", param_min_linear_vel_, default_min_linear_vel);

    pnh.param("deviation_weight_straight", param_deviation_weight_straight_, default_deviation_weight_straight);
    pnh.param("parallelness_weight_straight", param_parallelness_weight_straight_,
              default_parallelness_weight_straight);
    pnh.param("deviation_weight_arc", param_deviation_weight_arc_, default_deviation_weight_arc);
    pnh.param("parallelness_weight_arc", param_parallelness_weight_arc_, default_parallelness_weight_arc);
    pnh.param("epsilon", param_epsilon_, default_epsilon);

    pnh.param("design_dead_zone", param_design_dead_zone_, default_design_dead_zone);
    pnh.param("design_arc_radius", param_design_arc_radius_, default_design_arc_radius);
    pnh.param("design_step_size", param_design_step_size_, default_step_size);
    pnh.param("hermite_magnitude", param_hermite_magnitude_, default_hermite_magnitude);

    pnh.param("odom_filter_time", param_odom_filter_time_, default_odom_filter_time);
    pnh.param("missed_distance_coefficient", param_missed_distance_coefficient_, default_missed_distance_coefficient);

    // be sure that path planning arc radius is greater or equal to minimum turning radius
    if (param_design_arc_radius_ < param_min_radius_)
    {
      ROS_WARN_STREAM("[AgribotLocalPlanner] param design_arc_radius to small. Taking param_min_radius = "
                      << param_min_radius_ << " instead");
      param_design_arc_radius_ = param_min_radius_;
    }

    pnh.param("speed_limit", param_speed_limit_, true);
    pnh.param("debug", param_debug_, false);
    pnh.param("statistics", param_statistics_, true);
    pnh.param("take_front", param_take_front_, false);

    robot_center_.header.frame_id = "map";

    createPlanners();
    createTrackers();
    createValidators();
    createControllers();

    initialized_ = true;

    if (param_statistics_)
    {
      stop_time_ = 0;
      start_time_ = 0;
      last_time_ = 0;
    }

    ROS_DEBUG_STREAM("[AgribotPlanner] Plugin initialized.");
    gn.setParam("/agribot/agribot_local_planer/is_inited", true);
  }
  else
  {
    ROS_WARN_STREAM("[AgribotPlanner] This planner has already been "
                    << "initialized, doing nothing.");
  }
}

bool AgribotPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  if (!initialized_)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }

  // remember global plan
  global_plan_ = orig_global_plan;

  // make local plan
  if (!replan(local_plan_, robot_center_, global_plan_.back()))
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Unable to create valid path");
    return false;
  }

  // set flags and reset additional variables
  abort_executing_ = false;
  goal_reached_ = false;
  traveled_points_ = visualization_msgs::Marker();
  last_odom_stamp_ = 0;
  traveled_distance_ = 0;

  if (param_statistics_)
  {
    start_time_ = ros::Time::now().toSec();
    last_time_ = 0;
  }

  return true;
}

bool AgribotPlannerROS::replan(std::vector<geometry_msgs::PoseStamped>& local_plan,
                               const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& end_pose)
{
  // make local plan
  local_plan_.clear();
  if (!planners_["circles"]->designPath(local_plan, start_pose, end_pose))
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Local planner unable to create path ");
    return false;
  }
  else
  {
    // validate
    if (!validators_["distance"]->validatePath(local_plan))
    {
      ROS_ERROR_STREAM("[AgribotPlanner] Local path is invalid - too short");
      return false;
    }
    ROS_INFO_STREAM("[AgribotPlanner] Local planner created, path length = " << local_plan_.size());
  }

  // reset trackers
  for (auto const& [tracker_id, tracker] : trackers_)
  {
    tracker->reset();
  }

  // reset controllers
  agribot_path_controllers::PurePursuitController* pure_pursuit_pointer =
      dynamic_cast<agribot_path_controllers::PurePursuitController*>(controllers_["pure_pursuit"].get());
  if (pure_pursuit_pointer != nullptr)
  {
    pure_pursuit_pointer->resetRabbit();
  }
  return true;
}

bool AgribotPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  geometry_msgs::Twist velocity;
  VisualizatioFucntions::pathVisualization(local_plan_, "map", path_pub_);

  if (!initialized_)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }

  if (local_plan_.size() != 0)
  {
    std::vector<geometry_msgs::PoseStamped> path_slice;
    std::vector<geometry_msgs::PoseStamped> path_slice_ray;

    geometry_msgs::PoseStamped robot_center = xTransformPose(robot_center_, reference_frame_id_);
    pos path_end_waypoint =
        CommonFunctions::pCoordinatesFromPoseStamped(xTransformPose(local_plan_.back(), reference_frame_id_));

    if (!trackers_["side_ray"]->findPathSlice(path_slice_ray, local_plan_, robot_center))
    {
      trackers_["front_ray"]->findPathSlice(path_slice_ray, local_plan_, robot_center);
    }

    trackers_["following"]->findPathSlice(path_slice, local_plan_, robot_center);

    if (path_slice_ray.size() > 1)
    {
      path_slice = path_slice_ray;
    }

    VisualizatioFucntions::pathVisualization(path_slice, "map", path_slice_pub_);

    if (path_slice.size() < 2)
    {
      cmd_vel = xStopRobot();
      if (!replan(local_plan_, robot_center_, global_plan_.back()))
      {
        ROS_ERROR_STREAM("[AgribotPlanner] Unable to replan path");
        return false;
      }
      return true;
    }

    controllers_["deviation_parallelness"]->computeControl(velocity, path_slice, robot_center);

    // Calculate velocities
    velocity = xLimitVelocities(velocity);

    velocity.linear.x = dCloseToZero(velocity.linear.x);
    velocity.linear.y = dCloseToZero(velocity.linear.y);
    velocity.linear.z = dCloseToZero(velocity.linear.z);
    velocity.angular.x = dCloseToZero(velocity.angular.x);
    velocity.angular.y = dCloseToZero(velocity.angular.y);
    velocity.angular.z = dCloseToZero(velocity.angular.z);

    // Stop condition
    if (bReachedPathEnd(CommonFunctions::pCoordinatesFromPoseStamped(robot_center), path_end_waypoint))
    {
      velocity = xFinishedPathBehaviour();
    }
    VisualizatioFucntions::predictedPathVisualization(
        velocity, CommonFunctions::pCoordinatesFromPoseStamped(robot_center), param_prediction_points_per_second_,
        param_prediction_time_, "map", predicted_path_pub_);
  }
  if (abort_executing_)
  {
    cmd_vel = xStopRobot();
    ROS_ERROR("[AgribotPlanner] Abort plan");
    return false;
  }

  cmd_vel = velocity;
  return true;
}

double AgribotPlannerROS::dCloseToZero(const double value)
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

bool AgribotPlannerROS::bReachedPathEnd(const pos& robot_coordinates, const pos& path_end_waypoint)
{
  double distanceToGoal = CommonFunctions::dDistanceBetweenCoordinates(path_end_waypoint, robot_coordinates);
  return (distanceToGoal <= param_goal_distance_tolerance_);
}

geometry_msgs::Twist AgribotPlannerROS::xFinishedPathBehaviour()
{
  goal_reached_ = true;
  local_plan_.clear();
  if (param_statistics_)
  {
    stop_time_ = ros::Time::now().toSec();
    ROS_INFO("[AgribotPlanner] T = %f;l¸= %f", stop_time_ - start_time_, traveled_distance_);
  }
  return xStopRobot();
}

bool AgribotPlannerROS::isGoalReached()
{
  if (!initialized_)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }
  return goal_reached_;
}

void AgribotPlannerROS::vOdomCallback(const nav_msgs::Odometry& odometry_msg)
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
    robot_center_ = xTransformPose(robotPose, reference_frame_id_);

    traveled_distance_ += CommonFunctions::dDistanceBetweenCoordinates(
        lastRobotPosition, CommonFunctions::pCoordinatesFromPoseStamped(robot_center_));
  }
}

geometry_msgs::Twist AgribotPlannerROS::xLimitVelocities(geometry_msgs::Twist velocities)
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
  return velocities;
}

geometry_msgs::Twist AgribotPlannerROS::xStopRobot()
{
  geometry_msgs::Twist velocity;
  velocity.linear.x = 0;
  velocity.linear.y = 0;
  velocity.angular.z = 0;
  return velocity;
}

geometry_msgs::PoseStamped AgribotPlannerROS::xTransformPose(geometry_msgs::PoseStamped ps, std::string& frame_id)
{
  geometry_msgs::PoseStamped poseTransformed;
  poseTransformed.header.frame_id = frame_id;

  try
  {
    ps.header.stamp = ros::Time::now();
    tf_buffer_->transform(ps, poseTransformed, frame_id, ros::Duration(0.2));
  }
  catch (tf2::ExtrapolationException e)
  {
    ROS_ERROR_STREAM("[LocalPlanner] ExtrapolationException occur");
    abort_executing_ = true;
  }
  return poseTransformed;
}
}  // namespace agribot_local_planner
