#pragma once

#include "agribot_navigation/planners/structures.h"
#include "agribot_navigation/planners/local_planners/path_planner.h"

namespace agribot_local_planner
{

class PolynomialPlanner : public PathPlanner
{
public:
  PolynomialPlanner();
  ~PolynomialPlanner();
  bool designPath(std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::PoseStamped startPose,
                  geometry_msgs::PoseStamped endPose);

protected:
  void parametricPath(std::vector<geometry_msgs::PoseStamped>& path, pos start_point,  // NOLINT(runtime/references)
                      pos end_point, parametric_coeficients coefficients);
  parametric_coeficients calculateParametricCoefficients(pos startPoint, pos endPoint, double inflection);
  int countChangesX(pos startPoint, pos endPoint);
  int countChangesY(pos startPoint, pos endPoint);
  int x_changes;
  int y_changes;
};
}