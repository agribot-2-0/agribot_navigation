/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_msgs/AgribotManualControl.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
#include <vector>

#include "agribot_navigation/common/common_functions.h"
#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/path_controllers/i_path_controller.h"

namespace agribot_path_controllers
{
class StanleyController : public IPathController
{
public:
  StanleyController() = default;
  bool computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                      const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                      const geometry_msgs::PoseStamped& robot_pose) override;
  void reset() override{};
  std::string getFrameId() override
  {
    return "base_footprint";
  };
  void setVelocityLinear(const double velocity) override;
  void setErrorGain(const double gain);
  void setMinMotionRadius(const double min_motion_radius);
  void setRobotLength(const double robot_length);

  geometry_msgs::PoseStamped getProjectionPose(void);

private:
  double lin_vel_;
  double error_gain_;
  double min_motion_radius_;
  double robot_length_;
  geometry_msgs::PoseStamped projection_pose_;

  double error_crosstrack_;
  double error_heading_;
  double error_crosstrack_factor_;
  double theta_;
  double motion_radius_;

  double calculateCrosstrackError(const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                                  const geometry_msgs::PoseStamped& robot_pose);
};
}  // namespace agribot_path_controllers
