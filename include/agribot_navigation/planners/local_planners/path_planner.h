#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cmath>
#include <vector>

#include "agribot_navigation/planners/structures.h"

#define DEG_TO_RAD (M_PI / 180)
#define RAD_TO_DEG (180 / M_PI)

namespace agribot_local_planner
{

class PathPlanner
{
public:
  PathPlanner();
  ~PathPlanner();
  virtual bool designPath(std::vector<geometry_msgs::PoseStamped>& path, geometry_msgs::PoseStamped startPose,
                          geometry_msgs::PoseStamped endPose);  // NOLINT(runtime/references)
  void setStepSize(double param_step_size);
  void updateOrientation(std::vector<geometry_msgs::PoseStamped>& path);

protected:
  double step_size;
  std_msgs::Header header;
  bool straightPath(std::vector<geometry_msgs::PoseStamped>& path, pos start_point, pos end_point, double step_size);
  double dGetYawFromQuaternion(geometry_msgs::Quaternion quat);
  double dLimitAngle(double angle);
  double dAngleBetweenFrames(pos targetFrame, pos currentFrame);
  double dDistanceBetweenFrames(pos targetFrame, pos currentFrame);
  pos pFrameFromPoseStamped(geometry_msgs::PoseStamped ps);
};
}