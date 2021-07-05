/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_msgs/AgribotManualControl.h>

#include <string>
#include <vector>

#include "agribot_navigation/common/common_functions.h"
#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/path_controllers/i_path_controller.h"

namespace agribot_path_controllers
{
class PurePursuitController : public IPathController
{
public:
  PurePursuitController();
  bool computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                      const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                      const geometry_msgs::PoseStamped& robot_pose) override;
  void reset() override;

  void setVelocityLinear(const double velocity);
  void setRobotLength(const double robot_length);
  void setRabbitDistance(const double distance);
  geometry_msgs::PoseStamped getRabbitPose();
  void setRabbitPose(const geometry_msgs::PoseStamped& rabbit);
  void setMinMotionRadius(const double min_motion_radius);

private:
  bool computeRabbit(double look_ahead_distance, const std::size_t& path_size);
  double lin_vel_;
  double robot_length_;
  double rabbit_distance_;
  double look_ahead_distance_;
  double angle_between_poses_;
  double motion_radius_;
  double min_motion_radius_;

  std::size_t rabbit_count_;
  geometry_msgs::PoseStamped rabbit_pose_;
};
}  // namespace agribot_path_controllers
