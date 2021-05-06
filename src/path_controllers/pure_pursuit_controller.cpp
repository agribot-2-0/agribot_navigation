/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_controllers/pure_pursuit_controller.h"

namespace agribot_path_controllers
{
PurePursuitController::PurePursuitController() : IPathController()
{
  lin_vel_ = default_max_linear_vel;
  robot_length_ = default_chassis_length;
  rabbit_distance_ = default_pure_pursuit_rabbit_distance;
  rabbit_count_ = 0;
  look_ahead_distance_ = 0;
}

PurePursuitController::~PurePursuitController()
{
}

void PurePursuitController::setVelocityLinear(const double velocity)
{
  lin_vel_ = velocity;
}

void PurePursuitController::setRobotLength(const double robot_length)
{
  robot_length_ = robot_length;
}

void PurePursuitController::setRabbitDistance(const double distance)
{
  rabbit_distance_ = distance;
}

void PurePursuitController::reset()
{
  rabbit_count_ = 0;
}

geometry_msgs::PoseStamped PurePursuitController::getRabbitPose()
{
  return rabbit_pose_;
}

void PurePursuitController::setRabbitPose(const geometry_msgs::PoseStamped& rabbit)
{
  rabbit_pose_ = rabbit;
}

bool PurePursuitController::computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                                           const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                                           const geometry_msgs::PoseStamped& robot_pose)
{
  do
  {
    look_ahead_distance_ = CommonFunctions::distanceBetweenPoses(rabbit_pose_, robot_pose);
    rabbit_pose_ = path_to_track[rabbit_count_];
  } while (computeRabbit(look_ahead_distance_, path_to_track.size()));

  if (look_ahead_distance_ < rabbit_distance_)
  {
    angle_between_poses_ = CommonFunctions::pCoordinatesFromPoseStamped(rabbit_pose_).az -
                           CommonFunctions::pCoordinatesFromPoseStamped(robot_pose).az;
  }
  else
  {
    angle_between_poses_ = CommonFunctions::limitAngle(CommonFunctions::angleBetweenPoses(rabbit_pose_, robot_pose));
  }

  // double chassis_length = 2.1;
  motion_radius_ = look_ahead_distance_ / (2 * sin(angle_between_poses_));
  // double steering_angle = atan((2 * chassis_length * sin(alpha)) / (look_ahead_distance_));
  // double steering_angle = atan(chassis_length / motion_radius);
  // double steering_angle1 = asin(chassis_length / motion_radius);

  // ROS_ERROR_STREAM("alpha " << CommonFunctions::toDegrees(angle_between_poses_));
  // ROS_ERROR_STREAM("motion_radius " << motion_radius_);
  // ROS_ERROR_STREAM("steering_angle " << CommonFunctions::toDegrees(steering_angle));
  // ROS_ERROR_STREAM("steering_angle1 " << CommonFunctions::toDegrees(steering_angle1));

  agribot_msgs::AgribotManualControl msg;
  msg.linear_velocity = lin_vel_;
  msg.circle_radius = motion_radius_;

  cmd_vel = teleopToVel(msg);

  return true;
}

geometry_msgs::Twist PurePursuitController::teleopToVel(const agribot_msgs::AgribotManualControl& in_msg)
{
  geometry_msgs::Twist out_msg;
  double minRadius{ 2.1 };
  // double cutoffRadius{ 110.0 };

  out_msg.linear.x = in_msg.linear_velocity;
  if (abs(in_msg.circle_radius) > minRadius)  //&& abs(in_msg.circle_radius) < cutoffRadius)
  {
    out_msg.angular.z = in_msg.linear_velocity / in_msg.circle_radius;
  }
  else if (abs(in_msg.circle_radius) < minRadius && in_msg.circle_radius > 0)
  {
    out_msg.angular.z = in_msg.linear_velocity / minRadius;
  }
  else if (abs(in_msg.circle_radius) < minRadius && in_msg.circle_radius < 0)
  {
    out_msg.angular.z = -in_msg.linear_velocity / minRadius;
  }
  return out_msg;
}

bool PurePursuitController::computeRabbit(double look_ahead_distance, const std::size_t& path_size)
{
  if (look_ahead_distance < rabbit_distance_)
  {
    rabbit_count_++;
    if (rabbit_count_ >= path_size)
    {
      rabbit_count_ = path_size - 1;
      return false;
    }
    return true;
  }
  return false;
}
}  // namespace agribot_path_controllers
