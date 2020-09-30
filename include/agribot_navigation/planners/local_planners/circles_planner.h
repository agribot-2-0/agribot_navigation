#pragma once

#include "agribot_navigation/planners/structures.h"
#include "agribot_navigation/planners/local_planners/path_planner.h"

namespace agribot_local_planner
{

class CirclesPlanner : public PathPlanner
{
public:
  CirclesPlanner();
  ~CirclesPlanner();
  bool designPath(std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::PoseStamped startPose,
                  geometry_msgs::PoseStamped endPose);
  void setArchRadius(double param_arch_radius);
  void setArchStep(double param_arch_step);
  void setDeadZone(double param_dead_zone);

protected:
  pos pCalculateC1Center(pos c2_center, pos end_point, double path_angle, double angle_difference, double arch_radius,
                         double dead_zone);
  pos pCalculateC2Center(pos endPoint, double end_orientation, double angle_difference, double arch_radius,
                         double dead_zone);
  pos pCalculateTangentPoint(pos c1_center, double path_angle, double angle_difference, double arch_radius,
                             double dead_zone);
  // NOLINTNEXTLINE(runtime/references)
  void arcPath(std::vector<geometry_msgs::PoseStamped>& path, pos circle_start, pos circle_center, double angle_span);
  bool assemblePath(std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::PoseStamped startPose,
                    geometry_msgs::PoseStamped endPose, pos start_tangent_point, pos start_c1_center,
                    pos start_c2_center, pos end_tangent_point, pos end_c1_center, pos end_c2_center);
  pos end_c2_center;
  pos end_c1_center;
  pos end_tangent_point;
  pos start_c2_center;
  pos start_c1_center;
  pos start_tangent_point;
  double arch_radius;
  double arch_step;
  double dead_zone;
};
}  // namespace agribot_local_planner
