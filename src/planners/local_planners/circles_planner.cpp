#include "agribot_navigation/planners/local_planners/circles_planner.h"

namespace agribot_local_planner
{
CirclesPlanner::CirclesPlanner() : PathPlanner()
{
  dead_zone = 0.0;
  arch_radius = 2.0;
  arch_step = 5.0;
}

CirclesPlanner::~CirclesPlanner()
{
}

void CirclesPlanner::setArchRadius(double param_arch_radius)
{
  arch_radius = param_arch_radius;
}

void CirclesPlanner::setArchStep(double param_arch_step)
{
  arch_step = param_arch_step;
}

void CirclesPlanner::setDeadZone(double param_dead_zone)
{
  dead_zone = param_dead_zone;
}

bool CirclesPlanner::designPath(std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::PoseStamped startPose,
                                geometry_msgs::PoseStamped endPose)
{
  ROS_INFO_STREAM("[CirclesPlanner] XD");
  pos startPoint = pFrameFromPoseStamped(startPose);
  pos endPoint = pFrameFromPoseStamped(endPose);
  header = endPose.header;

  double path_angle = dAngleBetweenFrames(endPoint, startPoint);
  double reverse_path_angle = dAngleBetweenFrames(startPoint, endPoint);
  double end_orientation = endPoint.az;
  double reverse_robot_orientation = startPoint.az - M_PI;
  double end_angle_difference = dLimitAngle((end_orientation - path_angle));
  double start_angle_difference = dLimitAngle((reverse_robot_orientation - reverse_path_angle));

  pos end_c2_center = pCalculateC2Center(endPoint, end_orientation, end_angle_difference, arch_radius, dead_zone);
  pos end_c1_center =
      pCalculateC1Center(end_c2_center, endPoint, path_angle, end_angle_difference, arch_radius, dead_zone);
  pos end_tangent_point =
      pCalculateTangentPoint(end_c1_center, path_angle, end_angle_difference, arch_radius, dead_zone);

  pos start_c2_center =
      pCalculateC2Center(startPoint, reverse_robot_orientation, start_angle_difference, arch_radius, dead_zone);
  pos start_c1_center = pCalculateC1Center(start_c2_center, startPoint, reverse_path_angle, start_angle_difference,
                                           arch_radius, dead_zone);
  pos start_tangent_point =
      pCalculateTangentPoint(start_c1_center, reverse_path_angle, start_angle_difference, arch_radius, dead_zone);

  return assemblePath(path, startPose, endPose, start_tangent_point, start_c1_center, start_c2_center,
                      end_tangent_point, end_c1_center, end_c2_center);
}

pos CirclesPlanner::pCalculateC2Center(pos endPoint, double end_orientation, double angle_difference,
                                       double arch_radius, double dead_zone)
{
  // determine center of circle tangent to orientation vector
  // as a vector of length equals to given radius
  // perpendicular to the orientation vector

  pos c2_center;
  if (angle_difference < -dead_zone * DEG_TO_RAD)
  {
    double perpendicular = end_orientation - M_PI / 2;
    c2_center.x = endPoint.x + arch_radius * cos(perpendicular);
    c2_center.y = endPoint.y + arch_radius * sin(perpendicular);
  }
  else if (angle_difference > dead_zone * DEG_TO_RAD)
  {
    double perpendicular = end_orientation + M_PI / 2;
    c2_center.x = endPoint.x + arch_radius * cos(perpendicular);
    c2_center.y = endPoint.y + arch_radius * sin(perpendicular);
  }
  else
  {
    c2_center.x = std::numeric_limits<double>::quiet_NaN();
    c2_center.y = std::numeric_limits<double>::quiet_NaN();
  }
  return c2_center;
}

pos CirclesPlanner::pCalculateC1Center(pos c2_center, pos endPoint, double path_angle, double angle_difference,
                                       double arch_radius, double dead_zone)
{
  // determine center of circle tangent to original path
  // as a interception of circle with radius twice as big as given radius
  // and line parallel to original path
  // distance from line and path equals given radius

  pos c1_center;
  pos point_on_parallel;
  double dx, dy, a, b, c, discriminant, t1, t2, diameter;
  diameter = arch_radius * 2.0;

  dx = cos(path_angle);
  dy = sin(path_angle);

  if (angle_difference < -dead_zone * DEG_TO_RAD)
  {
    double perpendicular = path_angle + 90 * DEG_TO_RAD;
    point_on_parallel.x = endPoint.x + arch_radius * cos(perpendicular);
    point_on_parallel.y = endPoint.y + arch_radius * sin(perpendicular);
    a = dx * dx + dy * dy;
    b = 2 * (dx * (point_on_parallel.x - c2_center.x) + dy * (point_on_parallel.y - c2_center.y));
    c = (point_on_parallel.x - c2_center.x) * (point_on_parallel.x - c2_center.x) +
        (point_on_parallel.y - c2_center.y) * (point_on_parallel.y - c2_center.y) - diameter * diameter;
    discriminant = b * b - 4 * a * c;
    if (discriminant >= 0)
    {
      t1 = (-b + sqrt(discriminant)) / (2 * a);
      t2 = (-b - sqrt(discriminant)) / (2 * a);
      c1_center.x = dx * t2 + point_on_parallel.x;
      c1_center.y = dy * t2 + point_on_parallel.y;
    }
    else
    {
      c1_center.x = std::numeric_limits<double>::quiet_NaN();
      c1_center.y = std::numeric_limits<double>::quiet_NaN();
    }
  }
  else if (angle_difference > dead_zone * DEG_TO_RAD)
  {
    double perpendicular = path_angle - 90 * DEG_TO_RAD;
    point_on_parallel.x = endPoint.x + arch_radius * cos(perpendicular);
    point_on_parallel.y = endPoint.y + arch_radius * sin(perpendicular);
    a = dx * dx + dy * dy;
    b = 2 * (dx * (point_on_parallel.x - c2_center.x) + dy * (point_on_parallel.y - c2_center.y));
    c = (point_on_parallel.x - c2_center.x) * (point_on_parallel.x - c2_center.x) +
        (point_on_parallel.y - c2_center.y) * (point_on_parallel.y - c2_center.y) - diameter * diameter;
    discriminant = b * b - 4 * a * c;
    if (discriminant >= 0)
    {
      t1 = (-b + sqrt(discriminant)) / (2 * a);
      t2 = (-b - sqrt(discriminant)) / (2 * a);
      c1_center.x = dx * t2 + point_on_parallel.x;
      c1_center.y = dy * t2 + point_on_parallel.y;
    }
    else
    {
      c1_center.x = std::numeric_limits<double>::quiet_NaN();
      c1_center.y = std::numeric_limits<double>::quiet_NaN();
    }
  }
  else
  {
    c1_center.x = std::numeric_limits<double>::quiet_NaN();
    c1_center.y = std::numeric_limits<double>::quiet_NaN();
  }
  return c1_center;
}

pos CirclesPlanner::pCalculateTangentPoint(pos c1_center, double path_angle, double angle_difference,
                                           double arch_radius, double dead_zone)
{
  // determine the point of the intersection of
  // the circle C2 and the original path

  pos tangent_point;
  if (angle_difference < -dead_zone * DEG_TO_RAD)
  {
    double perpendicular = path_angle - M_PI / 2;
    tangent_point.x = c1_center.x + arch_radius * cos(perpendicular);
    tangent_point.y = c1_center.y + arch_radius * sin(perpendicular);
  }
  else if (angle_difference > dead_zone * DEG_TO_RAD)
  {
    double perpendicular = path_angle + M_PI / 2;
    tangent_point.x = c1_center.x + arch_radius * cos(perpendicular);
    tangent_point.y = c1_center.y + arch_radius * sin(perpendicular);
  }
  else
  {
    tangent_point.x = std::numeric_limits<double>::quiet_NaN();
    tangent_point.y = std::numeric_limits<double>::quiet_NaN();
  }
  return tangent_point;
}

void CirclesPlanner::arcPath(std::vector<geometry_msgs::PoseStamped>& path, pos circle_start, pos circle_center,
                             double angle_span)
{
  double angle_start_center = dAngleBetweenFrames(circle_start, circle_center);

  int steps = abs(angle_span * RAD_TO_DEG) / arch_step;
  double step_size = angle_span / steps;

  if (steps > 0)
  {
    for (int i = 0; i < steps; i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = header.frame_id;
      pose.header.seq = path.size() + 1;
      pose.pose.position.x =
          circle_center.x + arch_radius * cos(angle_start_center + static_cast<double>(i) * step_size);
      pose.pose.position.y =
          circle_center.y + arch_radius * sin(angle_start_center + static_cast<double>(i) * step_size);
      path.push_back(pose);
    }
  }
}

bool CirclesPlanner::assemblePath(std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::PoseStamped startPose,
                                  geometry_msgs::PoseStamped endPose, pos start_tangent_point, pos start_c1_center,
                                  pos start_c2_center, pos end_tangent_point, pos end_c1_center, pos end_c2_center)
{
  pos startPoint = pFrameFromPoseStamped(startPose);
  pos endPoint = pFrameFromPoseStamped(endPose);

  double path_angle = dAngleBetweenFrames(endPoint, startPoint);
  double reverse_path_angle = dAngleBetweenFrames(startPoint, endPoint);
  double end_orientation = endPoint.az;
  double reverse_robot_orientation = startPoint.az - M_PI;
  double end_angle_difference = dLimitAngle((end_orientation - path_angle));
  double start_angle_difference = dLimitAngle((reverse_robot_orientation - reverse_path_angle));

  // calculate angles and spans for end archs
  double a_end_p1_ps = dAngleBetweenFrames(end_tangent_point, end_c1_center);
  double a_end_p1_p2 = dAngleBetweenFrames(end_c2_center, end_c1_center);
  double end_c1_span = dLimitAngle(a_end_p1_p2 - a_end_p1_ps);
  double a_end_p2_p1 = dAngleBetweenFrames(end_c1_center, end_c2_center);
  double a_end_p2_pk = dAngleBetweenFrames(endPoint, end_c2_center);
  double end_c2_span = a_end_p2_pk - a_end_p2_p1;

  // if the final orientation relative to path is positive
  // (robot turns left), circle end_c2 is followed counterclockwise
  // end_c2 is followed clockwise in right turn case
  if (end_angle_difference < 0)
  {
    if (end_c2_span > 0)
    {
      end_c2_span = fmod(end_c2_span - 360 * DEG_TO_RAD, 360 * DEG_TO_RAD);
    }
  }
  else
  {
    if (end_c2_span < 0)
    {
      end_c2_span = fmod(end_c2_span + 360 * DEG_TO_RAD, 360 * DEG_TO_RAD);
    }
  }

  // calculate angles and spans for start archs
  double a_start_p1_ps = dAngleBetweenFrames(start_tangent_point, start_c1_center);
  double a_start_p1_p2 = dAngleBetweenFrames(start_c2_center, start_c1_center);
  double start_c1_span = dLimitAngle(a_start_p1_ps - a_start_p1_p2);
  double a_start_p2_p1 = dAngleBetweenFrames(start_c1_center, start_c2_center);
  double a_start_p2_pk = dAngleBetweenFrames(startPoint, start_c2_center);
  double start_c2_span = a_start_p2_p1 - a_start_p2_pk;

  // if the final orientation relative to path is positive
  // (robot turns left), circle end_c2 is followed counterclockwise
  // end_c2 is followed clockwise in right turn case
  if (start_angle_difference > 0)
  {
    if (start_c2_span > 0)
    {
      start_c2_span = fmod(start_c2_span - 360 * DEG_TO_RAD, 360 * DEG_TO_RAD);
    }
  }
  else
  {
    if (start_c2_span < 0)
    {
      start_c2_span = fmod(start_c2_span + 360 * DEG_TO_RAD, 360 * DEG_TO_RAD);
    }
  }

  if ((dDistanceBetweenFrames(startPoint, endPoint) <
       dDistanceBetweenFrames(end_tangent_point, endPoint) + dDistanceBetweenFrames(start_tangent_point, startPoint)) ||
      isnan(end_c2_center.x) || isnan(end_c1_center.x) || isnan(end_tangent_point.x))
  {
    return false;
  }
  else
  {
    // appednd arch from circle end_c2
    arcPath(path, startPoint, start_c2_center, start_c2_span);
    ROS_INFO_STREAM("[CirclesPlanner] 1 = " << path.size());

    // appednd arch from circle end_c1
    arcPath(path, start_c2_center, start_c1_center, start_c1_span);
    ROS_INFO_STREAM("[CirclesPlanner] 2 = " << path.size());

    straightPath(path, start_tangent_point, end_tangent_point, step_size);
    ROS_INFO_STREAM("[CirclesPlanner] 3 = " << path.size());

    // appednd arch from circle end_c1
    arcPath(path, end_tangent_point, end_c1_center, end_c1_span);
    ROS_INFO_STREAM("[CirclesPlanner] 4 = " << path.size());

    // appednd arch from circle end_c2
    arcPath(path, end_c1_center, end_c2_center, end_c2_span);
    ROS_INFO_STREAM("[CirclesPlanner] 5 = " << path.size());

    // append final pose
    path.push_back(endPose);
    ROS_INFO_STREAM("[CirclesPlanner] 6 = " << path.size());

    return true;
  }
}
}