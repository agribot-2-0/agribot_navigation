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

void PurePursuitController::resetRabbit()
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
  // rabbit_pose_ = path_to_track[rabbit_count_];

  double look_ahead_distance = CommonFunctions::distanceBeetweenPoses(rabbit_pose_, robot_pose);

  while (computeRabbit(look_ahead_distance, path_to_track.size()))
  {
    rabbit_pose_ = path_to_track[rabbit_count_];
    look_ahead_distance = CommonFunctions::distanceBeetweenPoses(rabbit_pose_, robot_pose);
  }

  // double alpha = CommonFunctions::angleBeetweenPoses(rabbit_pose_, robot_pose);
  // alpha = CommonFunctions::dLimitAngle(alpha);
  double alpha = CommonFunctions::pCoordinatesFromPoseStamped(rabbit_pose_).az -
                 CommonFunctions::pCoordinatesFromPoseStamped(robot_pose).az;
  double motion_radius = look_ahead_distance / (2 * sin(alpha));

  agribot_msgs::AgribotManualControl msg;
  msg.circle_radius = motion_radius;
  msg.linear_velocity = lin_vel_;

  cmd_vel = teleopToVel(msg);

  // double theta = asin(2 * robot_length_ * sin(alpha) / look_ahead_distance);

  // if (isnan(theta))
  // {
  //   theta = 0.0;
  // }

  // cmd_vel.linear.x = lin_vel_;
  // cmd_vel.angular.z = sin(theta) * lin_vel_;
  // cmd_vel.angular.z = lin_vel_ / msg.circle_radius;

  // ROS_INFO_STREAM("pure rabbit pose " << rabbit_pose_.pose.position.x << " " << rabbit_pose_.pose.position.y);
  // ROS_INFO_STREAM("pure robot pose " << robot_pose.pose.position.x << " " << robot_pose.pose.position.y);
  // ROS_INFO_STREAM("pure ld " << look_ahead_distance);
  // ROS_INFO_STREAM("pure R " << msg.circle_radius);
  // ROS_INFO_STREAM("pure alpha " << alpha);
  // // ROS_INFO_STREAM("pure theta " << theta);
  // ROS_INFO_STREAM("pure angular " << lin_vel_ / motion_radius);
  // ROS_INFO_STREAM("pure angular " << cmd_vel.angular.z);
  return true;
}

geometry_msgs::Twist PurePursuitController::teleopToVel(const agribot_msgs::AgribotManualControl& in_msg)
{
  geometry_msgs::Twist out_msg;
  double minRadius{ 2.1 };
  double cutoffRadius{ 110.0 };

  out_msg.linear.x = in_msg.linear_velocity;
  if (abs(in_msg.circle_radius) > minRadius && abs(in_msg.circle_radius) < cutoffRadius)
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
