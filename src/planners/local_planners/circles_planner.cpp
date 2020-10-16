/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include "agribot_navigation/planners/local_planners/circles_planner.h"

namespace agribot_local_planner
{
CirclesPlanner::CirclesPlanner() : ILocalPlanner()
{
  dead_zone = default_design_dead_zone;
  arch_radius = default_design_arch_radius;
  step_size = default_step_size;
  arch_step = calculateArchStep(arch_radius, step_size);
}

CirclesPlanner::CirclesPlanner(const std::string frame) : ILocalPlanner()
{
  CirclesPlanner();
  setHeader(frame);
}

CirclesPlanner::~CirclesPlanner()
{
}

bool CirclesPlanner::designPath(std::vector<geometry_msgs::PoseStamped>& path,
                                const geometry_msgs::PoseStamped& startPose,
                                const geometry_msgs::PoseStamped& endPose) const
{
  pos startPoint = functions.pFrameFromPoseStamped(startPose);
  pos endPoint = functions.pFrameFromPoseStamped(endPose);
  int path_start_index = path.size();

  double path_angle = functions.dAngleBetweenFrames(endPoint, startPoint);
  double reverse_path_angle = functions.dAngleBetweenFrames(startPoint, endPoint);
  double end_orientation = endPoint.az;
  double reverse_robot_orientation = startPoint.az - M_PI;
  double end_angle_difference = functions.dLimitAngle((end_orientation - path_angle));
  double start_angle_difference = functions.dLimitAngle((reverse_robot_orientation - reverse_path_angle));

  pos end_c2_center = pCircleTangentToVector(endPoint, end_orientation, end_angle_difference, arch_radius, dead_zone);
  pos end_c1_center =
      pCircleTangentToPathAndCircle(end_c2_center, endPoint, path_angle, end_angle_difference, arch_radius, dead_zone);
  pos end_tangent_point =
      pCircleTangentToVector(end_c1_center, path_angle, end_angle_difference, arch_radius, dead_zone);

  pos start_c2_center =
      pCircleTangentToVector(startPoint, reverse_robot_orientation, start_angle_difference, arch_radius, dead_zone);
  pos start_c1_center = pCircleTangentToPathAndCircle(start_c2_center, startPoint, reverse_path_angle,
                                                      start_angle_difference, arch_radius, dead_zone);
  pos start_tangent_point =
      pCircleTangentToVector(start_c1_center, reverse_path_angle, start_angle_difference, arch_radius, dead_zone);

  if (assemblePath(path, startPoint, endPoint, start_tangent_point, start_c1_center, start_c2_center, end_tangent_point,
                   end_c1_center, end_c2_center))
  {
    path.push_back(endPose);
    updateOrientation(path, path_start_index, path.size() - 1);
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Failed to create plan");
    return false;
  }
}

void CirclesPlanner::setArchRadius(const double param_arch_radius)
{
  arch_radius = param_arch_radius;
  arch_step = calculateArchStep(arch_radius, step_size);
}

void CirclesPlanner::setDeadZone(const double param_dead_zone)
{
  dead_zone = param_dead_zone;
}

void CirclesPlanner::setStepSize(const double param_step_size)
{
  step_size = param_step_size;
  arch_step = calculateArchStep(arch_radius, step_size);
}

/**
 * determine center of circle tangent to original path and second circle
 * as an interception of circle with radius twice as big as given radius
 * and line parallel to original path
 * distance from line and path equals given radius */
pos CirclesPlanner::pCircleTangentToPathAndCircle(const pos& circle_center, const pos& endPoint,
                                                  const double path_angle, const double angle_difference,
                                                  const double arch_radius, const double dead_zone) const
{
  pos c1_center;
  pos point_on_parallel;
  double dx, dy, a, b, c, discriminant, t1, t2, diameter;
  diameter = arch_radius * 2.0;

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
  point_on_parallel.x = endPoint.x + arch_radius * cos(perpendicular);
  point_on_parallel.y = endPoint.y + arch_radius * sin(perpendicular);

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
                                           const double angle_difference, const double arch_radius,
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

  circle_center.x = tangentPoint.x + arch_radius * cos(perpendicular);
  circle_center.y = tangentPoint.y + arch_radius * sin(perpendicular);
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

  if (angle_difference < -dead_zone * deg_to_rad)
  {
    perpendicular = vector_orientation - direction * 90 * deg_to_rad;
  }
  else if (angle_difference > dead_zone * deg_to_rad)
  {
    perpendicular = vector_orientation + direction * 90 * deg_to_rad;
  }
  else
  {
    perpendicular = std::numeric_limits<double>::quiet_NaN();
  }
  return perpendicular;
}

double CirclesPlanner::calculateArchStep(const double radius, const double step_size) const
{
  // calculated from: step = 2 * pi * r * angle / 360
  return (180 * step_size) / (M_PI * radius);
}

bool CirclesPlanner::arcPath(std::vector<geometry_msgs::PoseStamped>& path, const pos& circle_start,
                             const pos& circle_center, const double angle_span) const
{
  double angle_start_center = functions.dAngleBetweenFrames(circle_start, circle_center);

  int steps = abs(angle_span * rad_to_deg) / arch_step;
  double step_size = angle_span / steps;

  if (steps > 0)
  {
    for (int i = 0; i < steps; i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = frame_id;
      pose.header.seq = path.size() + 1;
      pose.pose.position.x =
          circle_center.x + arch_radius * cos(angle_start_center + static_cast<double>(i) * step_size);
      pose.pose.position.y =
          circle_center.y + arch_radius * sin(angle_start_center + static_cast<double>(i) * step_size);
      path.push_back(pose);
    }
    return true;
  }
  else
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble arc path");
    return false;
  }
}

bool CirclesPlanner::straightPath(std::vector<geometry_msgs::PoseStamped>& path, const pos& start_point,
                                  const pos& end_point, const double step_size) const
{
  int steps = functions.dDistanceBetweenFrames(start_point, end_point) / step_size;

  if (steps > 0)
  {
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
  else
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble straight path");
    return false;
  }
}

bool CirclesPlanner::assemblePath(std::vector<geometry_msgs::PoseStamped>& path, const pos& startPoint,
                                  const pos& endPoint, const pos& start_tangent_point, const pos& start_c1_center,
                                  const pos& start_c2_center, const pos& end_tangent_point, const pos& end_c1_center,
                                  const pos& end_c2_center) const
{
  // end_angle_difference = end_orientation - path_angle;
  double end_angle_difference =
      functions.dLimitAngle((endPoint.az - functions.dAngleBetweenFrames(endPoint, startPoint)));
  // start_angle_difference = reverse_robot_orientation - reverse_path_angle;
  double start_angle_difference =
      functions.dLimitAngle(((startPoint.az - M_PI) - functions.dAngleBetweenFrames(startPoint, endPoint)));

  // calculate angles and spans for end archs
  double a_end_p1_ps = functions.dAngleBetweenFrames(end_tangent_point, end_c1_center);
  double a_end_p1_p2 = functions.dAngleBetweenFrames(end_c2_center, end_c1_center);
  double end_c1_span = functions.dLimitAngle(a_end_p1_p2 - a_end_p1_ps);
  double a_end_p2_p1 = functions.dAngleBetweenFrames(end_c1_center, end_c2_center);
  double a_end_p2_pk = functions.dAngleBetweenFrames(endPoint, end_c2_center);
  double end_c2_span = a_end_p2_pk - a_end_p2_p1;

  /* if the final orientation relative to path is positive
   (robot turns left), circle end_c2 is followed counterclockwise
   end_c2 is followed clockwise in right turn case*/
  if (end_angle_difference < 0)
  {
    if (end_c2_span > 0)
    {
      end_c2_span = fmod(end_c2_span - 360 * deg_to_rad, 360 * deg_to_rad);
    }
  }
  else
  {
    if (end_c2_span < 0)
    {
      end_c2_span = fmod(end_c2_span + 360 * deg_to_rad, 360 * deg_to_rad);
    }
  }

  // calculate angles and spans for start archs
  double a_start_p1_ps = functions.dAngleBetweenFrames(start_tangent_point, start_c1_center);
  double a_start_p1_p2 = functions.dAngleBetweenFrames(start_c2_center, start_c1_center);
  double start_c1_span = functions.dLimitAngle(a_start_p1_ps - a_start_p1_p2);
  double a_start_p2_p1 = functions.dAngleBetweenFrames(start_c1_center, start_c2_center);
  double a_start_p2_pk = functions.dAngleBetweenFrames(startPoint, start_c2_center);
  double start_c2_span = a_start_p2_p1 - a_start_p2_pk;

  /* if the final orientation relative to path is positive
   (robot turns left), circle end_c2 is followed counterclockwise
   end_c2 is followed clockwise in right turn case*/
  if (start_angle_difference > 0)
  {
    if (start_c2_span > 0)
    {
      start_c2_span = fmod(start_c2_span - 360 * deg_to_rad, 360 * deg_to_rad);
    }
  }
  else
  {
    if (start_c2_span < 0)
    {
      start_c2_span = fmod(start_c2_span + 360 * deg_to_rad, 360 * deg_to_rad);
    }
  }

  if ((functions.dDistanceBetweenFrames(startPoint, endPoint) <
       functions.dDistanceBetweenFrames(end_tangent_point, endPoint) +
           functions.dDistanceBetweenFrames(start_tangent_point, startPoint)) ||
      isnan(end_c2_center.x) || isnan(end_c1_center.x) || isnan(end_tangent_point.x))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Unable to create proper path");
    return false;
  }

  // appednd arch from circle start_c2
  if (!arcPath(path, startPoint, start_c2_center, start_c2_span))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble first starting arch");
    return false;
  }
  // appednd arch from circle start_c1
  if (!arcPath(path, start_c2_center, start_c1_center, start_c1_span))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble second starting arch");
    return false;
  }
  // appednd straight from circle start to circle end
  if (!straightPath(path, start_tangent_point, end_tangent_point, step_size))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble straight path between archs");
    return false;
  }
  // appednd arch from circle end_c1
  if (!arcPath(path, end_tangent_point, end_c1_center, end_c1_span))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble first ending arch");
    return false;
  }
  // appednd arch from circle end_c2
  if (!arcPath(path, end_c1_center, end_c2_center, end_c2_span))
  {
    ROS_ERROR_STREAM("[CirclesPlanner] Could not assemble second ending arch");
    return false;
  }

  return true;
}
}  // namespace agribot_local_planner
