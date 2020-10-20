/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/local_planners/circles_planner.h"

namespace agribot_local_planner
{
CirclesPlanner::CirclesPlanner() : ILocalPlanner()
{
  dead_zone = default_design_dead_zone;
  arc_radius = default_design_arc_radius;
  step_size = default_step_size;
  arc_step = calculateArcStep(arc_radius, step_size);
}

CirclesPlanner::CirclesPlanner(const std::string& frame) : ILocalPlanner()
{
  CirclesPlanner();
  setFrameId(frame);
}

CirclesPlanner::~CirclesPlanner()
{
}

bool CirclesPlanner::designPath(std::vector<geometry_msgs::PoseStamped>& path,
                                const geometry_msgs::PoseStamped& start_pose,
                                const geometry_msgs::PoseStamped& end_pose) const
{
  pos start_point = CommonFunctions::pCoordinatesFromPoseStamped(start_pose);
  pos end_point = CommonFunctions::pCoordinatesFromPoseStamped(end_pose);
  int path_start_index = path.size();

  double path_angle = CommonFunctions::dAngleBetweenCoordinates(end_point, start_point);
  double reverse_path_angle = CommonFunctions::dAngleBetweenCoordinates(start_point, end_point);
  double end_orientation = end_point.az;
  double reverse_robot_orientation = start_point.az - M_PI;
  double end_angle_difference = CommonFunctions::dLimitAngle((end_orientation - path_angle));
  double start_angle_difference = CommonFunctions::dLimitAngle((reverse_robot_orientation - reverse_path_angle));

  pos end_c2_center = pCircleTangentToVector(end_point, end_orientation, end_angle_difference, arc_radius, dead_zone);
  pos end_c1_center =
      pCircleTangentToPathAndCircle(end_c2_center, end_point, path_angle, end_angle_difference, arc_radius, dead_zone);
  pos end_tangent_point =
      pCircleTangentToVector(end_c1_center, path_angle, end_angle_difference, arc_radius, dead_zone);

  pos start_c2_center =
      pCircleTangentToVector(start_point, reverse_robot_orientation, start_angle_difference, arc_radius, dead_zone);
  pos start_c1_center = pCircleTangentToPathAndCircle(start_c2_center, start_point, reverse_path_angle,
                                                      start_angle_difference, arc_radius, dead_zone);
  pos start_tangent_point =
      pCircleTangentToVector(start_c1_center, reverse_path_angle, start_angle_difference, arc_radius, dead_zone);

  if (!assemblePath(path, start_point, end_point, start_tangent_point, start_c1_center, start_c2_center,
                    end_tangent_point, end_c1_center, end_c2_center))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Failed to create plan");
    return false;
  }

  path.push_back(end_pose);
  CommonFunctions::updateOrientationInPath(path, path_start_index, path.size() - 1);
  return true;
}

void CirclesPlanner::setArcRadius(const double param_arc_radius)
{
  arc_radius = param_arc_radius;
  arc_step = calculateArcStep(arc_radius, step_size);
}

void CirclesPlanner::setDeadZone(const double param_dead_zone)
{
  dead_zone = param_dead_zone;
}

void CirclesPlanner::setStepSize(const double param_step_size)
{
  step_size = param_step_size;
  arc_step = calculateArcStep(arc_radius, step_size);
}

/**
 * determine center of circle tangent to original path and second circle
 * as an interception of circle with radius twice as big as given radius
 * and line parallel to original path
 * distance from line and path equals given radius */
pos CirclesPlanner::pCircleTangentToPathAndCircle(const pos& circle_center, const pos& end_point,
                                                  const double path_angle, const double angle_difference,
                                                  const double arc_radius, const double dead_zone) const
{
  pos c1_center;
  pos point_on_parallel;
  double dx, dy, a, b, c, discriminant, t1, t2, diameter;
  diameter = arc_radius * 2.0;

  // angle of vector perpendicular to path
  double perpendicular = calculatePerpendicular(angle_difference, path_angle, dead_zone, true);

  if (isnan(perpendicular))
  {
    c1_center.x = std::numeric_limits<double>::quiet_NaN();
    c1_center.y = std::numeric_limits<double>::quiet_NaN();
    return c1_center;
  }

  // path x and y gradient
  dx = cos(path_angle);
  dy = sin(path_angle);

  // point laying on the parallel to the path
  point_on_parallel.x = end_point.x + arc_radius * cos(perpendicular);
  point_on_parallel.y = end_point.y + arc_radius * sin(perpendicular);

  // determine roots of square function - one of the roots is searched circle center
  a = dx * dx + dy * dy;
  b = 2 * (dx * (point_on_parallel.x - circle_center.x) + dy * (point_on_parallel.y - circle_center.y));
  c = (point_on_parallel.x - circle_center.x) * (point_on_parallel.x - circle_center.x) +
      (point_on_parallel.y - circle_center.y) * (point_on_parallel.y - circle_center.y) - diameter * diameter;
  discriminant = b * b - 4 * a * c;
  if (discriminant >= 0)
  {
    t2 = (-b - sqrt(discriminant)) / (2 * a);
    c1_center.x = dx * t2 + point_on_parallel.x;
    c1_center.y = dy * t2 + point_on_parallel.y;
  }
  else
  {
    c1_center.x = std::numeric_limits<double>::quiet_NaN();
    c1_center.y = std::numeric_limits<double>::quiet_NaN();
  }
  return c1_center;
}

/**
 * determine center of circle tangent to orientation vector
 * as a vector with length equal to given radius
 * perpendicular to the orientation vector
 */
pos CirclesPlanner::pCircleTangentToVector(const pos& tangentPoint, const double vector_orientation,
                                           const double angle_difference, const double arc_radius,
                                           const double dead_zone) const
{
  pos circle_center;
  double perpendicular = calculatePerpendicular(angle_difference, vector_orientation, dead_zone, false);
  if (isnan(perpendicular))
  {
    circle_center.x = std::numeric_limits<double>::quiet_NaN();
    circle_center.y = std::numeric_limits<double>::quiet_NaN();
    return circle_center;
  }

  circle_center.x = tangentPoint.x + arc_radius * cos(perpendicular);
  circle_center.y = tangentPoint.y + arc_radius * sin(perpendicular);
  return circle_center;
}

/**
 *  determine perpendicular direction depending on angle difference between path and orientation vector
 */
double CirclesPlanner::calculatePerpendicular(const double angle_difference, const double vector_orientation,
                                              const double dead_zone, bool reverse_direction) const
{
  double perpendicular;
  double direction;
  if (reverse_direction)
  {
    direction = -1.0;
  }
  else
  {
    direction = 1.0;
  }

  if (angle_difference < CommonFunctions::toRadians(-dead_zone))
  {
    perpendicular = vector_orientation - direction * CommonFunctions::toRadians(90);
  }
  else if (angle_difference > CommonFunctions::toRadians(dead_zone))
  {
    perpendicular = vector_orientation + direction * CommonFunctions::toRadians(90);
  }
  else
  {
    perpendicular = std::numeric_limits<double>::quiet_NaN();
  }
  return perpendicular;
}

double CirclesPlanner::calculateArcStep(const double radius, const double step_size) const
{
  // calculated from: step = 2 * pi * r * angle / 360
  return (180 * step_size) / (M_PI * radius);
}

bool CirclesPlanner::arcPath(std::vector<geometry_msgs::PoseStamped>& path, const pos& circle_start,
                             const pos& circle_center, const double angle_span) const
{
  double angle_start_center = CommonFunctions::dAngleBetweenCoordinates(circle_start, circle_center);

  int steps = abs(CommonFunctions::toDegrees(angle_span)) / arc_step;
  double step_size = angle_span / steps;

  if (steps <= 0)
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble arc path");
    return false;
  }

  for (int i = 0; i < steps; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.seq = path.size() + 1;
    pose.pose.position.x = circle_center.x + arc_radius * cos(angle_start_center + static_cast<double>(i) * step_size);
    pose.pose.position.y = circle_center.y + arc_radius * sin(angle_start_center + static_cast<double>(i) * step_size);
    path.push_back(pose);
  }
  return true;
}

bool CirclesPlanner::straightPath(std::vector<geometry_msgs::PoseStamped>& path, const pos& start_point,
                                  const pos& end_point, const double step_size) const
{
  int steps = CommonFunctions::dDistanceBetweenCoordinates(start_point, end_point) / step_size;

  if (steps <= 0)
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble straight path");
    return false;
  }

  for (int i = 0; i < steps; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.seq = path.size() + 1;
    pose.pose.position.x = start_point.x + static_cast<double>(i) / steps * (end_point.x - start_point.x);
    pose.pose.position.y = start_point.y + static_cast<double>(i) / steps * (end_point.y - start_point.y);
    path.push_back(pose);
  }
  return true;
}

double CirclesPlanner::calculateSpanC1(const pos& tangent_point, const pos& c1_center, const pos& c2_center,
                                       const bool reverse_direction) const
{
  double direction = 1.0;
  if (reverse_direction)
  {
    direction = -1.0;
  }
  double a_p1_ps = CommonFunctions::dAngleBetweenCoordinates(tangent_point, c1_center);
  double a_p1_p2 = CommonFunctions::dAngleBetweenCoordinates(c2_center, c1_center);
  return CommonFunctions::dLimitAngle(direction * (a_p1_p2 - a_p1_ps));
}

double CirclesPlanner::calculateSpanC2(const pos& end_point, const pos& c1_center, const pos& c2_center,
                                       const double angle_difference, const bool reverse_direction) const
{
  double direction = 1.0;
  if (reverse_direction)
  {
    direction = -1.0;
  }
  double a_p2_p1 = CommonFunctions::dAngleBetweenCoordinates(c1_center, c2_center);
  double a_p2_pk = CommonFunctions::dAngleBetweenCoordinates(end_point, c2_center);
  double c2_span = direction * (a_p2_pk - a_p2_p1);

  /* if the final orientation relative to path is positive
   (robot turns left), circle c2 is followed counterclockwise
   c2 is followed clockwise in right turn case*/
  if (direction * angle_difference < 0)
  {
    if (c2_span > 0)
    {
      c2_span = fmod(c2_span - CommonFunctions::toRadians(360), CommonFunctions::toRadians(360));
    }
  }
  else
  {
    if (c2_span < 0)
    {
      c2_span = fmod(c2_span + CommonFunctions::toRadians(360), CommonFunctions::toRadians(360));
    }
  }
  return c2_span;
}

bool CirclesPlanner::assemblePath(std::vector<geometry_msgs::PoseStamped>& path, const pos& start_point,
                                  const pos& end_point, const pos& start_tangent_point, const pos& start_c1_center,
                                  const pos& start_c2_center, const pos& end_tangent_point, const pos& end_c1_center,
                                  const pos& end_c2_center) const
{
  // end_angle_difference = end_orientation - path_angle;
  double end_angle_difference =
      CommonFunctions::dLimitAngle((end_point.az - CommonFunctions::dAngleBetweenCoordinates(end_point, start_point)));
  // start_angle_difference = reverse_robot_orientation - reverse_path_angle;
  double start_angle_difference = CommonFunctions::dLimitAngle(
      ((start_point.az - M_PI) - CommonFunctions::dAngleBetweenCoordinates(start_point, end_point)));

  // spans for end arcs
  double end_c1_span = calculateSpanC1(end_tangent_point, end_c1_center, end_c2_center, false);
  double end_c2_span = calculateSpanC2(end_point, end_c1_center, end_c2_center, end_angle_difference, false);

  // spans for start arcs
  double start_c1_span = calculateSpanC1(start_tangent_point, start_c1_center, start_c2_center, true);
  double start_c2_span = calculateSpanC2(start_point, start_c1_center, start_c2_center, start_angle_difference, true);

  if ((CommonFunctions::dDistanceBetweenCoordinates(start_point, end_point) <
       CommonFunctions::dDistanceBetweenCoordinates(end_tangent_point, end_point) +
           CommonFunctions::dDistanceBetweenCoordinates(start_tangent_point, start_point)) ||
      isnan(end_c2_center.x) || isnan(end_c1_center.x) || isnan(end_tangent_point.x))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Unable to create proper path");
    return false;
  }

  // appednd arc from circle start_c2
  if (!arcPath(path, start_point, start_c2_center, start_c2_span))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble first starting arc");
    return false;
  }
  // appednd arc from circle start_c1
  if (!arcPath(path, start_c2_center, start_c1_center, start_c1_span))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble second starting arc");
    return false;
  }
  // appednd straight from circle start to circle end
  if (!straightPath(path, start_tangent_point, end_tangent_point, step_size))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble straight path between arcs");
    return false;
  }
  // appednd arc from circle end_c1
  if (!arcPath(path, end_tangent_point, end_c1_center, end_c1_span))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble first ending arc");
    return false;
  }
  // appednd arc from circle end_c2
  if (!arcPath(path, end_c1_center, end_c2_center, end_c2_span))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble second ending arc");
    return false;
  }

  return true;
}
}  // namespace agribot_local_planner
