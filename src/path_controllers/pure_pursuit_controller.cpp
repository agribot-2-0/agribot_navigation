/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_controllers/pure_pursuit_controller.h"

namespace agribot_path_controllers
{
PurePursuitController::PurePursuitController(const std::string& frame) : IPathController()
{
  lin_vel_ = default_max_linear_vel;
  robot_length_ = default_chassis_length;
  rabbit_distance_ = default_pure_pursuit_rabbit_distance;
  rabbit_count_ = 0;
  look_ahead_distance_ = 0;
  frame_id_ = frame;
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

std::string PurePursuitController::getFrameId()
{
  return frame_id_;
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
  if (path_to_track.empty())
  {
    return false;
  }

  this->reset();
  rabbit_pose_ = path_to_track[rabbit_count_];

  do
  {
    look_ahead_distance_ = CommonFunctions::distanceBetweenPoses(rabbit_pose_, robot_pose);
    rabbit_pose_ = path_to_track[rabbit_count_];
  } while (computeRabbit(look_ahead_distance_, path_to_track.size()));

  if (look_ahead_distance_ < min_planning_distance_in_row_)
  {
    angle_between_poses_ = CommonFunctions::pCoordinatesFromPoseStamped(rabbit_pose_).az -
                           CommonFunctions::pCoordinatesFromPoseStamped(robot_pose).az;
  }
  else
  {
    angle_between_poses_ = CommonFunctions::angleBetweenPoses(rabbit_pose_, robot_pose);
  }

  if (angle_between_poses_ == 0.0)
  {
    cmd_vel.angular.x = 0.0;
    cmd_vel.linear.z = 0.0;
    return false;
  }

  motion_radius_ = look_ahead_distance_ / (tan(angle_between_poses_));

  agribot_msgs::AgribotManualControl msg;
  msg.linear_velocity = lin_vel_;
  msg.circle_radius = motion_radius_;

  cmd_vel = CommonFunctions::teleopToVel(msg, min_motion_radius_);

  if (motion_radius_ == 0.0)
  {
    cmd_vel.angular.x = 0.0;
    cmd_vel.linear.z = 0.0;
    return false;
  }

  return true;
}

bool PurePursuitController::computeRabbit(double look_ahead_distance, const std::size_t& path_size)
{
  if (rabbit_count_ >= path_size)
  {
    rabbit_count_ = path_size - 1;
    return false;
  }
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

void PurePursuitController::setMinMotionRadius(const double min_motion_radius)
{
  min_motion_radius_ = min_motion_radius;
}

void PurePursuitController::setMinPlanningDistance(const double distance)
{
  min_planning_distance_in_row_ = distance;
}

}  // namespace agribot_path_controllers
