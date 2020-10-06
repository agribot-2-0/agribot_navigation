/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <vector>

#include "agribot_navigation/planners/structures.h"
#include "agribot_navigation/planners/local_planners/i_local_planner.h"
#include "agribot_navigation/planners/default_values.h"

namespace agribot_local_planner
{
class PolynomialPlanner : public ILocalPlaner
{
public:
  PolynomialPlanner();
  ~PolynomialPlanner();
  virtual bool designPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                          const geometry_msgs::PoseStamped startPose, const geometry_msgs::PoseStamped endPose);
  void setStepSize(const double param_step_size);
  void setInflectionPoint(const double param_inflection_point);

protected:
  bool parametricPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                      const pos start_point, const pos end_point, const parametric_coeficients coefficients) const;
  parametric_coeficients calculateParametricCoefficients(const pos startPoint, const pos endPoint,
                                                         const double inflection) const;
  int countChangesX(const pos startPoint, const pos endPoint) const;
  int countChangesY(const pos startPoint, const pos endPoint) const;
  double step_size;         // meters
  double inflection_point;  // factor (0,1)
  int path_start_index;
};
}  // namespace agribot_local_planner
