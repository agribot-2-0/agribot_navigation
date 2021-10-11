/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#include "agribot_navigation/path_controllers/stanley_controller.h"

#include <agribot_common/CommonFunctions/common_functions.h>

#include <cmath>
#include <limits>

namespace agribot_path_controllers
{
StanleyController::StanleyController(const std::string& frame) : IPathController()
{
  frame_id_ = frame;
}

void StanleyController::setVelocityLinear(const double velocity)
{
  lin_vel_ = velocity;
}

void StanleyController::setErrorGain(const double gain)
{
  error_gain_ = gain;
}

void StanleyController::setMinMotionRadius(const double min_motion_radius)
{
  min_motion_radius_ = min_motion_radius;
}

void StanleyController::setRobotLength(const double robot_length)
{
  robot_length_ = robot_length;
}

geometry_msgs::PoseStamped StanleyController::getProjectionPose(void)
{
  return projection_pose_;
}

bool StanleyController::computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                                       const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                                       const geometry_msgs::PoseStamped& robot_pose)
{
  error_crosstrack_ = calculateCrosstrackError(path_to_track, robot_pose);
  error_heading_ = CommonFunctions::getYawFromQuaternion(
      CommonFunctions::orientationBetweenQuaternions(robot_pose.pose.orientation, projection_pose_.pose.orientation));

  error_crosstrack_factor_ = atan2(error_gain_ * error_crosstrack_, lin_vel_);

  if (CommonFunctions::distanceBetweenPoses(robot_pose, path_to_track.back()) < robot_length_)
  {
    theta_ = error_heading_;
  }
  else
  {
    theta_ = error_heading_ + error_crosstrack_factor_;
  }
  motion_radius_ = robot_length_ / 2.0 / sin(theta_);

  agribot_msgs::AgribotManualControl msg;
  msg.linear_velocity = lin_vel_;
  msg.circle_radius = motion_radius_;

  cmd_vel = CommonFunctions::teleopToVel(msg, min_motion_radius_);

  return true;
}

void StanleyController::reset()
{
}

std::string StanleyController::getFrameId()
{
  return frame_id_;
}

double StanleyController::calculateCrosstrackError(const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                                                   const geometry_msgs::PoseStamped& robot_pose)
{
  double shortest_distance = std::numeric_limits<double>::max();
  for (const auto& pose : path_to_track)
  {
    if (CommonFunctions::distanceBetweenPoses(pose, robot_pose) < shortest_distance)
    {
      shortest_distance = CommonFunctions::distanceBetweenPoses(pose, robot_pose);
      projection_pose_ = pose;
    }
  }

  double direction_of_projection =
      CommonFunctions::limitAngle(CommonFunctions::angleBetweenPoses(path_to_track.back(), robot_pose) -
                                  CommonFunctions::angleBetweenPoses(path_to_track.back(), path_to_track.front()));

  if (direction_of_projection < 0)
  {
    shortest_distance *= -1;
  }
  return shortest_distance;
}

}  // namespace agribot_path_controllers
