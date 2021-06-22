/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <string>
#include <vector>

#include "agribot_navigation/common/default_values.h"
#include "agribot_navigation/common/structures.h"
#include "agribot_navigation/planners/local_planners/i_local_planner.h"

namespace agribot_local_planner
{
class HermitePlanner : public ILocalPlanner
{
public:
  HermitePlanner();
  explicit HermitePlanner(const std::string& frame);
  ~HermitePlanner();
  bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                  const geometry_msgs::PoseStamped& start_pose, const geometry_msgs::PoseStamped& end_pose) override;
  void addObstacle(const ObstacleDetection::Obstacle& obstacle) override;
  std::vector<ObstacleDetection::Obstacle> getObstacles(void) override;
  void clearObstacles(void) override;

  void setStepSize(const double param_step_size);
  void setMagnitudeStart(const double param_magnitude);
  void setMagnitudeEnd(const double param_magnitude);

protected:
  bool hermitePath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                   const pos& start_point, const pos& end_point, double m_start, double m_end) const;
  pos calculateDerivatives(const double angle, const double magnitude) const;
  double calculateMagnitude(const pos& start, const pos& end, const double base_magnitude,
                            const bool calculate_start) const;
  double step_size;             // metres
  double base_magnitude_start;  // factor
  double base_magnitude_end;    // factor
  std::vector<ObstacleDetection::Obstacle> obstacles_;
};
}  // namespace agribot_local_planner
