/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */
#include "agribot_navigation/path_validators/distance_validator.h"

namespace agribot_path_validators
{
DistanceValidator::DistanceValidator()
{
  min_distance_ = default_chassis_length;
}
DistanceValidator::~DistanceValidator()
{
}
bool DistanceValidator::validatePath(const std::vector<geometry_msgs::PoseStamped>& path)
{
  double path_distance = 0.0;
  for (std::size_t i = 1; i < path.size(); i++)
  {
    path_distance +=
        CommonFunctions::dDistanceBetweenCoordinates(CommonFunctions::pCoordinatesFromPoseStamped(path[i]),
                                                     CommonFunctions::pCoordinatesFromPoseStamped(path[i - 1]));
  }

  if (path_distance < min_distance_)
  {
    ROS_ERROR_STREAM("[DistanceValidator] Path is not valid. Path distance equals "
                     << path_distance << " Min distance is " << min_distance_);
    return false;
  }
  return true;
}
void DistanceValidator::setMinDistance(const double distance)
{
  min_distance_ = distance;
}
}  // namespace agribot_path_validators
