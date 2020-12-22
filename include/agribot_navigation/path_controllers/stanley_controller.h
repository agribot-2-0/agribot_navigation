/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <string>
#include <vector>

#include "agribot_navigation/path_controllers/i_path_controller.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/common_functions.h"

namespace agribot_path_controllers
{
class StanleyController : public IPathController
{
public:
  StanleyController();
  ~StanleyController();
  bool computeControl(geometry_msgs::Twist& cmd_vel,  // NOLINT(runtime/references)
                      const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                      const geometry_msgs::PoseStamped& robot_pose) override;
  void setVelocityLinear(const double velocity);
  void setErrorGain(const double gain);
  void setThetaLimit(const double limit);

private:
  double lin_vel_;
  double error_gain_;
  double theta_limit_;
};
}  // namespace agribot_path_controllers
