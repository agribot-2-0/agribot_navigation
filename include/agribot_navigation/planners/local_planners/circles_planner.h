/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <vector>
#include <string>

#include "agribot_navigation/planners/structures.h"
#include "agribot_navigation/planners/local_planners/i_local_planner.h"
#include "agribot_navigation/planners/default_values.h"

namespace agribot_local_planner
{
class CirclesPlanner : public ILocalPlanner
{
public:
  CirclesPlanner();
  explicit CirclesPlanner(const std::string frame);
  ~CirclesPlanner();
  virtual bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                          const geometry_msgs::PoseStamped& startPose, const geometry_msgs::PoseStamped& endPose) const;
  void setArchRadius(const double param_arch_radius);
  void setDeadZone(const double param_dead_zone);
  void setStepSize(const double param_step_size);

protected:
  pos pCircleTangentToPathAndCircle(const pos& c2_center, const pos& endPoint, const double path_angle,
                                    const double angle_difference, const double arch_radius,
                                    const double dead_zone) const;
  pos pCircleTangentToVector(const pos& tangentPoint, const double vector_orientation, const double angle_difference,
                             const double arch_radius, const double dead_zone) const;
  double calculatePerpendicular(const double angle_difference, const double vector_orientation,
                                const double dead_zone, bool reverse_direction) const;
  double calculateArchStep(const double radius, const double step_size) const;
  bool arcPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
               const pos& circle_start, const pos& circle_center, const double angle_span) const;
  bool straightPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                    const pos& start_point, const pos& end_point, const double step_size) const;
  bool assemblePath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                    const pos& startPoint, const pos& endPoint, const pos& start_tangent_point,
                    const pos& start_c1_center, const pos& start_c2_center, const pos& end_tangent_point,
                    const pos& end_c1_center, const pos& end_c2_center) const;
  pos end_c2_center;
  pos end_c1_center;
  pos end_tangent_point;
  pos start_c2_center;
  pos start_c1_center;
  pos start_tangent_point;
  double arch_radius;  // metres
  double arch_step;    // degrees
  double dead_zone;    // degrees
  double step_size;    // metres
};
}  // namespace agribot_local_planner
