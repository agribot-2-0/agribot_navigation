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

  auto ray_tracker = std::make_unique<agribot_path_trackers::RayTracker>();
  ray_tracker->setRayWidth(default_tracker_ray_width);
  ray_tracker->setRayLength(default_tracker_ray_length);
  trackers_["ray"] = std::move(ray_tracker);
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
    path_pub_ = pnh.advertise<visualization_msgs::Marker>("travelled_path", 10);
    predicted_path_pub_ = pnh.advertise<visualization_msgs::Marker>("predicted_path", 10);
    goal_range_pub_ = pnh.advertise<visualization_msgs::Marker>("goal_range", 10);
    next_goal_pub_ = pnh.advertise<visualization_msgs::Marker>("next_goal", 10);
    local_path_pub_ = pnh.advertise<nav_msgs::Path>("local_path", 10);
    local_path_.header.frame_id = "map";

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

    robot_center_.header.frame_id = "odom";
    last_waypoint_.header.frame_id = "odom";

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
      actual_time_ = 0;
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
  abort_executing_ = false;
  waypoint_count_ = 1;
  missed_waypoints_count_ = 0;
  plan_ = orig_global_plan;
  local_plan_.clear();

  if (!planners_["circles"]->designPath(local_plan_, robot_center_, plan_.back()))
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Local planner unable to create path ");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("[AgribotPlanner] Local planner created, path length = " << local_plan_.size());
  }

  for (auto it = trackers_.cbegin(); it != trackers_.cend(); ++it)
  {
    it->second->reset();
  }

  // controllers_["pure_pursuit"];
  agribot_path_controllers::PurePursuitController* pure_pursuit_pointer =
      dynamic_cast<agribot_path_controllers::PurePursuitController*>(controllers_["pure_pursuit"].get());
  if (pure_pursuit_pointer != nullptr)
  {
    pure_pursuit_pointer->resetRabbit();
  }

  goal_reached_ = false;
  traveled_points_ = visualization_msgs::Marker();
  last_odom_stamp_ = 0;

  last_waypoint_ = robot_center_;
  traveled_distance_ = 0;
  last_traveled_distance_ = 0;

  if (param_statistics_)
  {
    deviation_integral_ = 0;

    start_time_ = ros::Time::now().toSec();
    last_time_ = 0;
    actual_time_ = 0;
  }

  return true;
}

bool AgribotPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  geometry_msgs::Twist velocity;
  pos goal;
  std::vector<geometry_msgs::PoseStamped> path_slice;

  vLocalPathVisualization(local_plan_);

  if (!initialized_)
  {
    ROS_ERROR_STREAM("[AgribotPlanner] Planner not initialized, "
                     << "call initialize() before planner");
    return false;
  }

  if (local_plan_.size() != 0)
  {
    // Check where robot is
    pos next_waypoint = pGetWaypoint(waypoint_count_);
    pos last_waypoint = CommonFunctions::pCoordinatesFromPoseStamped(xTransformPose(last_waypoint_));
    pos robot_center = CommonFunctions::pCoordinatesFromPoseStamped(xTransformPose(robot_center_));

    goal = next_waypoint;

    // Decide whether pure_pursuit straight or arc piece of path
    vChooseDeviationParallelnessParametres(waypoint_count_);

    // Deviation error for statistics
    if (param_statistics_)
    {
      vCalculateDeviationIntegral(last_waypoint, next_waypoint, robot_center);
    }

    // Increment missed points number
    if (bWaypointMissedCondition(robot_center, last_waypoint, next_waypoint))
    {
      ROS_ERROR_STREAM("[AgribotPlanner] OUT OF PATH!");
      missed_waypoints_count_++;
    }

    // Next goal condition
    if (!bIsLastWaypointInPath(waypoint_count_) &&
        (bWaypointHitCondition(robot_center, last_waypoint, next_waypoint) ||
         bWaypointMissedCondition(robot_center, last_waypoint, next_waypoint)))
    {
      if (param_statistics_)
      {
        ROS_INFO_STREAM("[AgribotPlanner] NEXT GOAL - Traveled " << traveled_distance_);
      }
      vTakeNextGoal(goal, robot_center);
      next_waypoint = pGetWaypoint(waypoint_count_);
    }

    // Calculate velocities
    velocity = xSetVel(goal, next_waypoint, last_waypoint, robot_center);

    velocity.linear.x = dCloseToZero(velocity.linear.x);
    velocity.linear.y = dCloseToZero(velocity.linear.y);
    velocity.linear.z = dCloseToZero(velocity.linear.z);
    velocity.angular.x = dCloseToZero(velocity.angular.x);
    velocity.angular.y = dCloseToZero(velocity.angular.y);
    velocity.angular.z = dCloseToZero(velocity.angular.z);

    // Stop condition
    if (bMissedTooManyWaypointsCondition(missed_waypoints_count_) ||
        (bIsLastWaypointInPath(waypoint_count_) && bWaypointHitCondition(robot_center, last_waypoint, next_waypoint)) ||
        (bIsLastWaypointInPath(waypoint_count_) &&
         bWaypointMissedCondition(robot_center, last_waypoint, next_waypoint)))
    {
      velocity = xFinishedPathBehaviour();
    }

    // Visualization
    vVisualizations(last_waypoint, goal, next_waypoint, robot_center, velocity);
  }
  if (abort_executing_)
  {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    ROS_ERROR("[AgribotPlanner] Abort plan");
    return false;
  }
  else
  {
    cmd_vel = velocity;
    return true;
  }
}

void AgribotPlannerROS::vChooseDeviationParallelnessParametres(int waypoint_number)
{
  // check if it is not the first waypoint in path
  if (waypoint_number > 1 && waypoint_number < local_plan_.size() - 1)
  {
    // get yaws from poses
    geometry_msgs::PoseStamped currentPose = local_plan_[waypoint_number];
    geometry_msgs::PoseStamped currentPoseTransformed;
    currentPoseTransformed = xTransformPose(currentPose);

    geometry_msgs::PoseStamped lastPose = local_plan_[waypoint_number - 1];
    geometry_msgs::PoseStamped lastPoseTransformed;
    lastPoseTransformed = xTransformPose(lastPose);

    double current_waypoint_orientation =
        CommonFunctions::dGetYawFromQuaternion(currentPoseTransformed.pose.orientation);
    double last_waypoint_orientation = CommonFunctions::dGetYawFromQuaternion(lastPoseTransformed.pose.orientation);
    // check if there is difference between yaws
    if (abs(current_waypoint_orientation - last_waypoint_orientation) > param_epsilon_)
    {
      // piece of path is an arc
      param_current_deviation_weight_ = param_deviation_weight_arc_;
      param_current_parallelness_weight_ = param_parallelness_weight_arc_;
    }
    else
    {
      // piece of path is a straight line
      param_current_deviation_weight_ = param_deviation_weight_straight_;
      param_current_parallelness_weight_ = param_parallelness_weight_straight_;
    }
  }
  else
  {
    param_current_deviation_weight_ = param_deviation_weight_straight_;
    param_current_parallelness_weight_ = param_parallelness_weight_straight_;
  }
}

double AgribotPlannerROS::dCloseToZero(double value)
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

bool AgribotPlannerROS::bMissedTooManyWaypointsCondition(int missed_waypoints)
{
  return missed_waypoints > param_missed_points_tolerance_;
}

bool AgribotPlannerROS::bWaypointHitCondition(const pos& robot_center, const pos& last_waypoint,
                                              const pos& next_waypoint)
{
  if (param_take_front_)
  {
    return (bTraveledDistanceBetweenWaypoints(pCalculateRobotFrontPosition(robot_center), last_waypoint,
                                              next_waypoint) &&
            bWithinGoalRange(pCalculateRobotFrontPosition(robot_center), next_waypoint)) ||
           (bTraveledDistanceBetweenWaypoints(robot_center, last_waypoint, next_waypoint) &&
            bWithinGoalRange(robot_center, next_waypoint));
  }
  else
  {
    return (bTraveledDistanceBetweenWaypoints(robot_center, last_waypoint, next_waypoint) &&
            bWithinGoalRange(robot_center, next_waypoint));
  }
}

bool AgribotPlannerROS::bWaypointMissedCondition(const pos& robot_center, const pos& last_waypoint,
                                                 const pos& next_waypoint)
{
  return bCenterTraveledBeyondGoal(robot_center, last_waypoint, next_waypoint) &&
         bCenterOutOfGoalRange(robot_center, next_waypoint);
}

bool AgribotPlannerROS::bTraveledDistanceBetweenWaypoints(pos robot_front, const pos& last_waypoint,
                                                          const pos& next_waypoint)
{
  double distanceWaypoints = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, last_waypoint);
  double traveledDistance = CommonFunctions::dDistanceBetweenCoordinates(last_waypoint, robot_front);
  return (traveledDistance >= distanceWaypoints - param_goal_distance_tolerance_);
}

bool AgribotPlannerROS::bWithinGoalRange(pos robot_front, const pos& next_waypoint)
{
  double distanceToGoal = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, robot_front);
  return (distanceToGoal <= param_goal_distance_tolerance_);
}

bool AgribotPlannerROS::bIsLastWaypointInPath(int waypoints_count)
{
  return (waypoints_count >= (local_plan_.size() - 1));
}

bool AgribotPlannerROS::bCenterTraveledBeyondGoal(const pos& robot_center, const pos& last_waypoint,
                                                  const pos& next_waypoint)
{
  double distanceWaypoints = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, last_waypoint);
  double traveledDistance = traveled_distance_ - last_traveled_distance_;
  return (traveledDistance >= distanceWaypoints * param_missed_distance_coefficient_);
}

bool AgribotPlannerROS::bCenterOutOfGoalRange(const pos& robot_center, const pos& next_waypoint)
{
  double distanceToGoal = CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, robot_center);
  return (distanceToGoal > param_goal_distance_tolerance_);
}

void AgribotPlannerROS::vTakeNextGoal(const pos& current_goal, const pos& robot_center)
{
  waypoint_count_++;
  last_traveled_distance_ = traveled_distance_;

  pos next_waypoint = pGetFrame(waypoint_count_);
  if (bIsLastWaypointInPath(waypoint_count_ + 1))
  {
    next_waypoint.az = CommonFunctions::dAngleBetweenCoordinates(next_waypoint, pGetFrame(waypoint_count_ - 1));
    if (param_take_front_)
    {
      next_waypoint = pCalculateRobotFrontPosition(next_waypoint);
    }
  }

  if (CommonFunctions::dDistanceBetweenCoordinates(next_waypoint, current_goal) > param_chassis_length_)
  {
    last_waypoint_ = local_plan_[waypoint_count_ - 1];
  }
  else
  {
    last_waypoint_ = robot_center_;
  }
}

void AgribotPlannerROS::vCalculateDeviationIntegral(const pos& last_waypoint, const pos& next_waypoint,
                                                    const pos& robot_center)
{
  double deviation = dCalculateDeviation(last_waypoint, next_waypoint, robot_center);
  if (actual_time_ > 0)
  {
    if (!isnan(deviation))
    {
      last_time_ = actual_time_;
      actual_time_ = ros::Time::now().toSec();
      deviation_integral_ += fabs(deviation) * (actual_time_ - last_time_);
    }
  }
  else
  {
    actual_time_ = ros::Time::now().toSec();
  }
}

geometry_msgs::Twist AgribotPlannerROS::xFinishedPathBehaviour()
{
  goal_reached_ = true;
  local_plan_.clear();
  if (param_statistics_)
  {
    stop_time_ = ros::Time::now().toSec();
    ROS_INFO("[AgribotPlanner] T = %f;l¸= %f;Er = %f;n = %d;x = %d", stop_time_ - start_time_, traveled_distance_,
             deviation_integral_, waypoint_count_, missed_waypoints_count_);
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
    robot_center_ = xTransformPose(robotPose);

    traveled_distance_ += CommonFunctions::dDistanceBetweenCoordinates(
        lastRobotPosition, CommonFunctions::pCoordinatesFromPoseStamped(robot_center_));
  }
}

pos AgribotPlannerROS::pCalculateRobotFrontPosition(const pos& robot_center)
{
  pos frontPos;
  frontPos.x = robot_center.x + cos(robot_center.az) * param_chassis_length_ / 2;
  frontPos.y = robot_center.y + sin(robot_center.az) * param_chassis_length_ / 2;
  frontPos.az = robot_center.az;
  return frontPos;
}

geometry_msgs::Twist AgribotPlannerROS::xSetVel(const pos& goal, const pos& next_waypoint, const pos& last_waypoint,
                                                const pos& robot_center)
{
  geometry_msgs::Twist velocity;
  if (param_speed_limit_)
  {
    velocity.linear.x = xLinearSpeedLimitedApproach(goal, last_waypoint, robot_center).linear.x;
  }
  else
  {
    velocity.linear.x = param_max_linear_vel_;
  }

  velocity.angular.z = xDeviationApproach(next_waypoint, last_waypoint, robot_center, velocity).angular.z;

  // Velocity limiter
  return xLimitVelocities(velocity);
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

geometry_msgs::Twist AgribotPlannerROS::xDeviationApproach(const pos& goal, const pos& last_waypoint,
                                                           const pos& robot_center, geometry_msgs::Twist velocity)
{
  double deviation = dCalculateDeviation(last_waypoint, goal, robot_center);

  double factorDeviation = param_current_deviation_weight_ * deviation / (1 + deviation);
  double factorParallelness =
      param_current_parallelness_weight_ *
      (1 - fabs(cos(CommonFunctions::dAngleBetweenCoordinates(goal, last_waypoint) - robot_center.az)));
  if (CommonFunctions::dLimitAngle(CommonFunctions::dAngleBetweenCoordinates(goal, robot_center) -
                                   CommonFunctions::dAngleBetweenCoordinates(goal, last_waypoint)) < 0)
  {
    factorDeviation *= -1;
  }
  if (CommonFunctions::dLimitAngle(CommonFunctions::dAngleBetweenCoordinates(goal, last_waypoint) - robot_center.az) <
      0)
  {
    factorParallelness *= -1;
  }

  velocity.angular.z = (factorDeviation + factorParallelness) * param_max_linear_vel_ / param_min_radius_;
  return velocity;
}

geometry_msgs::Twist AgribotPlannerROS::xLinearSpeedLimitedApproach(const pos& goal, const pos& last_waypoint,
                                                                    const pos& robot_center)
{
  geometry_msgs::Twist velocity;
  double distanceWaypoints = CommonFunctions::dDistanceBetweenCoordinates(goal, last_waypoint);
  double distanceToNext = CommonFunctions::dDistanceBetweenCoordinates(goal, robot_center);
  double waypointsDistanceFactor = distanceWaypoints / (1 + distanceWaypoints);
  double deviation = dCalculateDeviation(last_waypoint, goal, robot_center);
  double parallelnessFactor =
      fabs(cos(CommonFunctions::dAngleBetweenCoordinates(goal, last_waypoint) - robot_center.az));
  double deviationFactor = 1 / (1 + deviation);
  double distanceFactor = 1 - fabs((distanceWaypoints - distanceToNext) / distanceWaypoints);
  double multiplier = (1 - deviationFactor) * (1 - distanceFactor);
  velocity.linear.x =
      param_max_linear_vel_ * (deviationFactor * (1 - multiplier)) * waypointsDistanceFactor * parallelnessFactor;
  if (isnan(velocity.linear.x))
  {
    velocity.linear.x = 0.0;
  }
  return velocity;
}

double AgribotPlannerROS::dCalculateRadius(double distance_robot_waypoint, double angle_robot_waypoint)
{
  return distance_robot_waypoint / 2 / sin((angle_robot_waypoint));
}

geometry_msgs::Twist AgribotPlannerROS::xStopRobot()
{
  geometry_msgs::Twist velocity;
  velocity.linear.x = 0;
  velocity.linear.y = 0;
  velocity.angular.z = 0;
  return velocity;
}

pos AgribotPlannerROS::pGetFrame(int count)
{
  geometry_msgs::PoseStamped nextPose = local_plan_[count];
  geometry_msgs::PoseStamped nextPoseTransformed;
  nextPoseTransformed = xTransformPose(nextPose);

  return CommonFunctions::pCoordinatesFromPoseStamped(nextPoseTransformed);
}

pos AgribotPlannerROS::pGetWaypoint(int count)
{
  pos waypoint = pGetFrame(count);
  if (bIsLastWaypointInPath(count + 1))
  {
    if (count - 1 > 0)
    {
      waypoint.az = CommonFunctions::dAngleBetweenCoordinates(waypoint, pGetFrame(count - 1));
    }

    if (param_take_front_)
    {
      waypoint = pCalculateRobotFrontPosition(waypoint);
    }
  }
  return waypoint;
}

geometry_msgs::PoseStamped AgribotPlannerROS::xTransformPose(geometry_msgs::PoseStamped ps)
{
  geometry_msgs::PoseStamped poseTransformed;
  poseTransformed.header.frame_id = "map";

  try
  {
    ps.header.stamp = ros::Time::now();
    tf_buffer_->transform(ps, poseTransformed, "map", ros::Duration(0.2));
  }
  catch (tf2::ExtrapolationException e)
  {
    ROS_ERROR_STREAM("[LocalPlanner] ExtrapolationException occur");
    abort_executing_ = true;
  }
  return poseTransformed;
}

double AgribotPlannerROS::dCalculateDeviation(const pos& last_waypoint, const pos& next_waypoint,
                                              const pos& robot_center)
{
  if (CommonFunctions::dDistanceBetweenCoordinates(last_waypoint, next_waypoint) > 0)
  {
    pos projection = CommonFunctions::pCalculateProjection(last_waypoint, next_waypoint, robot_center);
    return CommonFunctions::dDistanceBetweenCoordinates(robot_center, projection);
  }
  else
  {
    return 0.0;
  }
}

pos AgribotPlannerROS::pCalculateGoalRange(const pos& next_waypoint, const pos& robot_center, double range)
{
  pos goalRange;
  double deltaX, deltaY;
  double angleRobotNextWaypoint =
      CommonFunctions::dLimitAngle(CommonFunctions::dAngleBetweenCoordinates(next_waypoint, robot_center));

  if (!bIsLastWaypointInPath(waypoint_count_))
  {
    // Perpendicular to robot-next_frame line
    deltaX = sin(angleRobotNextWaypoint);
    deltaY = -cos(angleRobotNextWaypoint);
  }
  else
  {
    deltaX = sin(next_waypoint.az);
    deltaY = -cos(next_waypoint.az);
  }
  goalRange.x = deltaX * range;
  goalRange.y = deltaY * range;
  goalRange.az = CommonFunctions::dLimitAngle(CommonFunctions::dAngleBetweenCoordinates(next_waypoint, goalRange));

  return goalRange;
}

void AgribotPlannerROS::vPathVisualization(const pos& robot_center)
{
  traveled_points_.header.frame_id = "map";
  traveled_points_.header.stamp = ros::Time::now();
  traveled_points_.ns = "travelled_path";
  traveled_points_.action = visualization_msgs::Marker::ADD;
  traveled_points_.id = 0;
  traveled_points_.type = visualization_msgs::Marker::LINE_STRIP;
  traveled_points_.scale.x = 0.1;
  traveled_points_.color.b = 1.0;
  traveled_points_.color.a = 1.0;
  traveled_points_.pose.orientation.w = 1;
  geometry_msgs::Point p;
  p.x = robot_center.x;
  p.y = robot_center.y;
  p.z = 0.1;
  traveled_points_.points.push_back(p);
  path_pub_.publish(traveled_points_);
}

void AgribotPlannerROS::vPredictedPathVisualization(const geometry_msgs::Twist& velocity, const pos& robot_center)
{
  visualization_msgs::Marker futurePoints;
  futurePoints.header.frame_id = "map";
  futurePoints.header.stamp = ros::Time::now();
  futurePoints.ns = "predicted_path";
  futurePoints.action = visualization_msgs::Marker::ADD;
  futurePoints.id = 1;
  futurePoints.type = visualization_msgs::Marker::LINE_STRIP;
  futurePoints.scale.x = 0.1;
  futurePoints.color.r = 1.0;
  futurePoints.color.a = 1.0;
  futurePoints.pose.orientation.w = 1;

  // calculating turn radius from angular and linear velocity
  // R = v / w

  double radius;
  if (velocity.angular.z != 0)
  {
    radius = velocity.linear.x / velocity.angular.z;
  }
  else
  {
    radius = 0;
  }

  if (radius != 0)
  {
    // draw arc
    geometry_msgs::Point p, tmp;
    for (int i = 0; i < param_prediction_points_per_second_ * param_prediction_time_; i++)
    {
      double theta = i * velocity.angular.z / param_prediction_points_per_second_;
      tmp.x = radius * cos(theta - CommonFunctions::toRadians(90)) - radius * cos(CommonFunctions::toRadians(-90));
      tmp.y = radius * sin(theta - CommonFunctions::toRadians(90)) - radius * sin(CommonFunctions::toRadians(-90));

      p.x = robot_center.x + tmp.x * cos(robot_center.az) - tmp.y * sin(robot_center.az);
      p.y = robot_center.y + tmp.x * sin(robot_center.az) + tmp.y * cos(robot_center.az);

      p.z = 0.1;
      futurePoints.points.push_back(p);
    }
  }
  else
  {
    // draw straight line
    geometry_msgs::Point p;
    for (int i = 0; i < param_prediction_points_per_second_ * param_prediction_time_; i++)
    {
      p.x = robot_center.x + i * velocity.linear.x / param_prediction_points_per_second_ * cos(robot_center.az);
      p.y = robot_center.y + i * velocity.linear.x / param_prediction_points_per_second_ * sin(robot_center.az);
      p.z = 0.1;
      futurePoints.points.push_back(p);
    }
  }
  predicted_path_pub_.publish(futurePoints);
}

void AgribotPlannerROS::vNextGoalVisualization(const pos& last_waypoint, const pos& goal)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "next_goal";
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 2;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.scale.x = 0.1;
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.pose.orientation.w = 1;

  geometry_msgs::Point p;
  p.x = last_waypoint.x;
  p.y = last_waypoint.y;
  p.z = 0;
  marker.points.push_back(p);

  p.x = goal.x;
  p.y = goal.y;
  p.z = 0;
  marker.points.push_back(p);

  next_goal_pub_.publish(marker);
}

void AgribotPlannerROS::vGoalRangeVisualization(const pos& goal, const pos& range)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "goal_range";
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.color.r = 1.0;
  marker.color.a = 0.2;

  geometry_msgs::Point p;
  marker.scale.x = 2 * param_goal_distance_tolerance_;
  marker.scale.y = 2 * param_goal_distance_tolerance_;
  marker.scale.z = 0.01;
  marker.pose.position.x = goal.x;
  marker.pose.position.y = goal.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.w = 1;

  goal_range_pub_.publish(marker);
}

void AgribotPlannerROS::vLocalPathVisualization(const std::vector<geometry_msgs::PoseStamped>& path)
{
  local_path_.header.stamp = ros::Time::now();
  local_path_.poses.clear();
  local_path_.poses = path;
  local_path_pub_.publish(local_path_);
}

void AgribotPlannerROS::vVisualizations(const pos& last_waypoint, const pos& goal, const pos& next_waypoint,
                                        const pos& robot_center, const geometry_msgs::Twist& velocity)
{
  vNextGoalVisualization(last_waypoint, goal);
  vGoalRangeVisualization(next_waypoint, pCalculateGoalRange(next_waypoint, robot_center,
                                                             param_goal_range_ * param_goal_distance_tolerance_));
  //    vPathVisualization(robot_center);
  vPredictedPathVisualization(velocity, robot_center);
}
}  // namespace agribot_local_planner
