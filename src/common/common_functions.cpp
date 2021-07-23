/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#include "agribot_navigation/common/common_functions.h"

namespace CommonFunctions
{
double getYawFromQuaternion(const geometry_msgs::Quaternion& quat)
{
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::Quaternion getQuaternionFromYaw(const double yaw)
{
  tf2::Quaternion quaterion_tf;
  quaterion_tf.setRPY(0, 0, yaw);
  return tf2::toMsg(quaterion_tf);
}
/**
 * Limit angles to range <-180, 180> degrees
 */
double limitAngle(double angle)
{
  while (angle > common_functions::radians(180))
  {
    angle -= common_functions::radians(360);
  }
  while (angle < common_functions::radians(-180))
  {
    angle += common_functions::radians(360);
  }
  return angle;
}

double limitAngle(double angle, double range)
{
  while (angle > range)
  {
    angle -= 2 * range;
  }
  while (angle < -range)
  {
    angle += 2 * range;
  }
  return angle;
}

double dAngleBetweenCoordinates(const agribot_local_planner::pos& targetFrame,
                                const agribot_local_planner::pos& currentFrame)
{
  return std::atan2(targetFrame.y - currentFrame.y, targetFrame.x - currentFrame.x);
}

double dDistanceBetweenCoordinates(const agribot_local_planner::pos& targetFrame,
                                   const agribot_local_planner::pos& currentFrame)
{
  return std::sqrt(std::pow(targetFrame.y - currentFrame.y, 2) + std::pow(targetFrame.x - currentFrame.x, 2));
}

double angleBetweenPoses(const geometry_msgs::PoseStamped& target_frame,
                         const geometry_msgs::PoseStamped& current_frame)
{
  return std::atan2(target_frame.pose.position.y - current_frame.pose.position.y,
                    target_frame.pose.position.x - current_frame.pose.position.x);
}
double distanceBetweenPoses(const geometry_msgs::PoseStamped& target_frame,
                            const geometry_msgs::PoseStamped& current_frame)
{
  return std::sqrt(std::pow(target_frame.pose.position.y - current_frame.pose.position.y, 2) +
                   std::pow(target_frame.pose.position.x - current_frame.pose.position.x, 2));
}

agribot_local_planner::pos pCoordinatesFromPoseStamped(const geometry_msgs::PoseStamped& ps)
{
  agribot_local_planner::pos coordinates;
  coordinates.x = ps.pose.position.x;
  coordinates.y = ps.pose.position.y;
  coordinates.az = getYawFromQuaternion(ps.pose.orientation);
  return coordinates;
}

// TODO(lchojnacki): Use iterators instead of indexes
void updateOrientationInPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                             const std::size_t start_index, const std::size_t end_index)
{
  // add proper orientation to each point
  for (std::size_t i = start_index; i < end_index; i++)
  {
    double angle = std::atan2(path[i + 1].pose.position.y - path[i].pose.position.y,
                              path[i + 1].pose.position.x - path[i].pose.position.x);
    path[i].pose.orientation.x = 0;
    path[i].pose.orientation.y = 0;
    path[i].pose.orientation.z = sin(angle / 2.0);
    path[i].pose.orientation.w = cos(angle / 2.0);
  }
}

agribot_local_planner::pos pCalculateProjection(const agribot_local_planner::pos& lastWaypoint,
                                                const agribot_local_planner::pos& nextWaypoint,
                                                const agribot_local_planner::pos& robotCenter)
{
  agribot_local_planner::pos projection;
  double a = dDistanceBetweenCoordinates(nextWaypoint, lastWaypoint);
  double b = dDistanceBetweenCoordinates(lastWaypoint, robotCenter);
  double c = dDistanceBetweenCoordinates(nextWaypoint, robotCenter);
  double dx = nextWaypoint.x - lastWaypoint.x;
  double dy = nextWaypoint.y - lastWaypoint.y;
  // taken from cosine law
  // cosine = (a*a + b*b - c*c) / (2*a*b)
  // a1 = b * cosine
  // ratio = a1 / a
  double ratio = (a * a + b * b - c * c) / (2 * a * a);
  projection.x = lastWaypoint.x + ratio * dx;
  projection.y = lastWaypoint.y + ratio * dy;
  projection.az = dAngleBetweenCoordinates(nextWaypoint, lastWaypoint);
  return projection;
}

geometry_msgs::PoseStamped calculateProjection(const geometry_msgs::PoseStamped& lastWaypoint,
                                               const geometry_msgs::PoseStamped& nextWaypoint,
                                               const geometry_msgs::PoseStamped& robotCenter)
{
  geometry_msgs::PoseStamped projection;
  projection.header = robotCenter.header;
  double a = distanceBetweenPoses(nextWaypoint, lastWaypoint);
  double b = distanceBetweenPoses(lastWaypoint, robotCenter);
  double c = distanceBetweenPoses(nextWaypoint, robotCenter);
  double dx = nextWaypoint.pose.position.x - lastWaypoint.pose.position.x;
  double dy = nextWaypoint.pose.position.y - lastWaypoint.pose.position.y;
  // taken from cosine law
  // cosine = (a*a + b*b - c*c) / (2*a*b)
  // a1 = b * cosine
  // ratio = a1 / a
  double ratio = (a * a + b * b - c * c) / (2 * a * a);
  projection.pose.position.x = lastWaypoint.pose.position.x + ratio * dx;
  projection.pose.position.y = lastWaypoint.pose.position.y + ratio * dy;
  projection.pose.orientation = getQuaternionFromYaw(angleBetweenPoses(nextWaypoint, lastWaypoint));
  return projection;
}

double calculateRadiusFromVelocity(const geometry_msgs::Twist& velocity)
{
  if (velocity.angular.z != 0)
  {
    return velocity.linear.x / velocity.angular.z;
  }
  return 0.0;
}

void straightPath(std::vector<geometry_msgs::PoseStamped>& path,  // NOLINT(runtime/references)
                  const geometry_msgs::PoseStamped& start_point, const geometry_msgs::PoseStamped& end_point,
                  const std::string& frame_id, const double step_size)
{
  int steps = CommonFunctions::distanceBetweenPoses(start_point, end_point) / step_size;

  if (steps <= 0)
  {
    throw std::invalid_argument("Could not assemble straight path");
  }

  for (int i = 0; i < steps; i++)
  {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.seq = path.size() + 1;
    pose.pose.position.x = start_point.pose.position.x +
                           static_cast<double>(i) / steps * (end_point.pose.position.x - start_point.pose.position.x);
    pose.pose.position.y = start_point.pose.position.y +
                           static_cast<double>(i) / steps * (end_point.pose.position.y - start_point.pose.position.y);
    path.push_back(pose);
  }
}

double calculateCrosstrackError(const std::vector<geometry_msgs::PoseStamped>& path_to_track,
                                const geometry_msgs::PoseStamped& robot_pose)
{
  double shortest_distance = std::numeric_limits<double>::max();
  for (const auto& pose : path_to_track)
  {
    if (CommonFunctions::distanceBetweenPoses(pose, robot_pose) < shortest_distance)
    {
      shortest_distance = CommonFunctions::distanceBetweenPoses(pose, robot_pose);
    }
  }
  return shortest_distance;
}

geometry_msgs::Quaternion orientationBetweenQuaternions(const geometry_msgs::Quaternion& quaternion1,
                                                        const geometry_msgs::Quaternion& quaternion2)
{
  tf2::Quaternion q_1, q_rot, q_2;
  tf2::convert(quaternion1, q_1);
  tf2::convert(quaternion2, q_2);

  q_rot = q_2 * q_1.inverse();
  return tf2::toMsg(q_rot);
}

geometry_msgs::Twist teleopToVel(const agribot_msgs::AgribotManualControl& in_msg, const double min_motion_radius,
                                 const double cutoff_radius)
{
  geometry_msgs::Twist out_msg;

  out_msg.linear.x = in_msg.linear_velocity;
  if (abs(in_msg.circle_radius) > min_motion_radius && abs(in_msg.circle_radius) < cutoff_radius)
  {
    out_msg.angular.z = in_msg.linear_velocity / in_msg.circle_radius;
  }
  else if (abs(in_msg.circle_radius) < min_motion_radius && in_msg.circle_radius > 0)
  {
    out_msg.angular.z = in_msg.linear_velocity / min_motion_radius;
  }
  else if (abs(in_msg.circle_radius) < min_motion_radius && in_msg.circle_radius < 0)
  {
    out_msg.angular.z = -in_msg.linear_velocity / min_motion_radius;
  }
  return out_msg;
}

geometry_msgs::Twist teleopToVel(const agribot_msgs::AgribotManualControl& in_msg, const double min_motion_radius)
{
  return teleopToVel(in_msg, min_motion_radius, std::numeric_limits<double>::max());
}

}  // namespace CommonFunctions
