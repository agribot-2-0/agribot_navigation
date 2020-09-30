#include "agribot_navigation/planners/local_planners/polynomial_planner.h"

namespace agribot_local_planner
{
PolynomialPlanner::PolynomialPlanner() : PathPlanner()
{
}

PolynomialPlanner::~PolynomialPlanner()
{
}

void PolynomialPlanner::parametricPath(std::vector<geometry_msgs::PoseStamped>& path, pos start_point, pos end_point,
                                       parametric_coeficients coefficients)
{
  pos point;
  pos last_point = start_point;
  double t_delta = 0.0001;

  geometry_msgs::PoseStamped pose;
  // pose.header.frame_id = header.frame_id;
  // pose.header.seq = path.size() + 1;
  pose.pose.position.x = start_point.x;
  pose.pose.position.y = start_point.y;
  path.push_back(pose);

  for (double t = 0.0; t <= 1.0; t += t_delta)
  {
    point.x = coefficients.x.p3 * pow(t, 3) + coefficients.x.p2 * pow(t, 2) + coefficients.x.p1 * t + coefficients.x.p0;
    point.y = coefficients.y.p3 * pow(t, 3) + coefficients.y.p2 * pow(t, 2) + coefficients.y.p1 * t + coefficients.y.p0;
    if (dDistanceBetweenFrames(last_point, point) >= step_size)
    {
      last_point = point;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      path.push_back(pose);
    }
  }
}

parametric_coeficients PolynomialPlanner::calculateParametricCoefficients(pos startPoint, pos endPoint,
                                                                          double inflection)
{
  parametric_coeficients coefficients;
  x_changes = countChangesX(startPoint, endPoint);
  y_changes = countChangesY(startPoint, endPoint);
  if (x_changes == 2 && y_changes == 2)
  {
    coefficients.x.p3 = ((12 * inflection - 6) * endPoint.y + (6 - 12 * inflection) * startPoint.y +
                         (endPoint.az * (4 - 6 * inflection) + startPoint.az * (2 - 6 * inflection)) * endPoint.x +
                         (startPoint.az * (6 * inflection - 2) + endPoint.az * (6 * inflection - 4)) * startPoint.x) /
                        (startPoint.az * (9 * pow(inflection, 2) - 9 * inflection + 2) +
                         endPoint.az * (-9 * pow(inflection, 2) + 9 * inflection - 2));
    coefficients.x.p2 = -((18 * pow(inflection, 2) - 6) * endPoint.y + (6 - 18 * pow(inflection, 2)) * startPoint.y +
                          (startPoint.az * (3 * inflection - 9 * pow(inflection, 2)) +
                           endPoint.az * (-9 * pow(inflection, 2) - 3 * inflection + 6)) *
                              endPoint.x +
                          (endPoint.az * (9 * pow(inflection, 2) + 3 * inflection - 6) +
                           startPoint.az * (9 * pow(inflection, 2) - 3 * inflection)) *
                              startPoint.x) /
                        (startPoint.az * (9 * pow(inflection, 2) - 9 * inflection + 2) +
                         endPoint.az * (-9 * pow(inflection, 2) + 9 * inflection - 2));
    coefficients.x.p1 = (6 * inflection * endPoint.y - 6 * inflection * startPoint.y -
                         6 * endPoint.az * inflection * endPoint.x + 6 * endPoint.az * inflection * startPoint.x) /
                        (startPoint.az * (3 * inflection - 1) + endPoint.az * (1 - 3 * inflection));
    coefficients.x.p0 = startPoint.x;
    coefficients.y.p3 = ((endPoint.az * (6 * inflection - 2) + startPoint.az * (6 * inflection - 4)) * endPoint.y +
                         (startPoint.az * (4 - 6 * inflection) + endPoint.az * (2 - 6 * inflection)) * startPoint.y +
                         startPoint.az * endPoint.az * (6 - 12 * inflection) * endPoint.x +
                         startPoint.az * endPoint.az * (12 * inflection - 6) * startPoint.x) /
                        (startPoint.az * (9 * pow(inflection, 2) - 9 * inflection + 2) +
                         endPoint.az * (-9 * pow(inflection, 2) + 9 * inflection - 2));
    coefficients.y.p2 = -((startPoint.az * (9 * pow(inflection, 2) + 3 * inflection - 6) +
                           endPoint.az * (9 * pow(inflection, 2) - 3 * inflection)) *
                              endPoint.y +
                          (endPoint.az * (3 * inflection - 9 * pow(inflection, 2)) +
                           startPoint.az * (-9 * pow(inflection, 2) - 3 * inflection + 6)) *
                              startPoint.y +
                          startPoint.az * endPoint.az * (6 - 18 * pow(inflection, 2)) * endPoint.x +
                          startPoint.az * endPoint.az * (18 * pow(inflection, 2) - 6) * startPoint.x) /
                        (startPoint.az * (9 * pow(inflection, 2) - 9 * inflection + 2) +
                         endPoint.az * (-9 * pow(inflection, 2) + 9 * inflection - 2));
    coefficients.y.p1 = (6 * startPoint.az * inflection * endPoint.y - 6 * startPoint.az * inflection * startPoint.y -
                         6 * startPoint.az * endPoint.az * inflection * endPoint.x +
                         6 * startPoint.az * endPoint.az * inflection * startPoint.x) /
                        (startPoint.az * (3 * inflection - 1) + endPoint.az * (1 - 3 * inflection));
    coefficients.y.p0 = startPoint.y;
  }
  else if (x_changes == 2 && y_changes == 1)
  {
    coefficients.x.p3 = ((4 * inflection - 2) * endPoint.y + (2 - 4 * inflection) * startPoint.y +
                         (endPoint.az * (2 - 2 * inflection) - 2 * startPoint.az * inflection) * endPoint.x +
                         (endPoint.az * (2 * inflection - 2) + 2 * startPoint.az * inflection) * startPoint.x) /
                        (startPoint.az * (3 * pow(inflection, 2) - 2 * inflection) +
                         endPoint.az * (-3 * pow(inflection, 2) + 4 * inflection - 1));
    coefficients.x.p2 =
        -((6 * pow(inflection, 2) - 2) * endPoint.y + (2 - 6 * pow(inflection, 2)) * startPoint.y +
          (endPoint.az * (3 - 3 * pow(inflection, 2)) - 3 * startPoint.az * pow(inflection, 2)) * endPoint.x +
          (endPoint.az * (3 * pow(inflection, 2) - 3) + 3 * startPoint.az * pow(inflection, 2)) * startPoint.x) /
        (startPoint.az * (3 * pow(inflection, 2) - 2 * inflection) +
         endPoint.az * (-3 * pow(inflection, 2) + 4 * inflection - 1));
    coefficients.x.p1 = ((6 * pow(inflection, 2) - 4 * inflection) * endPoint.y +
                         (4 * inflection - 6 * pow(inflection, 2)) * startPoint.y +
                         endPoint.az * (6 * inflection - 6 * pow(inflection, 2)) * endPoint.x +
                         endPoint.az * (6 * pow(inflection, 2) - 6 * inflection) * startPoint.x) /
                        (startPoint.az * (3 * pow(inflection, 2) - 2 * inflection) +
                         endPoint.az * (-3 * pow(inflection, 2) + 4 * inflection - 1));
    coefficients.x.p0 = startPoint.x;
    coefficients.y.p3 = 0.0;
    coefficients.y.p2 = -((startPoint.az * (3 * pow(inflection, 2) - 2 * inflection) +
                           endPoint.az * (3 * pow(inflection, 2) - 4 * inflection + 1)) *
                              endPoint.y +
                          (endPoint.az * (-3 * pow(inflection, 2) + 4 * inflection - 1) +
                           startPoint.az * (2 * inflection - 3 * pow(inflection, 2))) *
                              startPoint.y +
                          startPoint.az * endPoint.az * (6 * inflection - 6 * pow(inflection, 2)) * endPoint.x +
                          startPoint.az * endPoint.az * (6 * pow(inflection, 2) - 6 * inflection) * startPoint.x) /
                        (startPoint.az * (3 * pow(inflection, 2) - 2 * inflection) +
                         endPoint.az * (-3 * pow(inflection, 2) + 4 * inflection - 1));
    coefficients.y.p1 = (startPoint.az * (6 * pow(inflection, 2) - 4 * inflection) * endPoint.y +
                         startPoint.az * (4 * inflection - 6 * pow(inflection, 2)) * startPoint.y +
                         startPoint.az * endPoint.az * (6 * inflection - 6 * pow(inflection, 2)) * endPoint.x +
                         startPoint.az * endPoint.az * (6 * pow(inflection, 2) - 6 * inflection) * startPoint.x) /
                        (startPoint.az * (3 * pow(inflection, 2) - 2 * inflection) +
                         endPoint.az * (-3 * pow(inflection, 2) + 4 * inflection - 1));
    coefficients.y.p0 = startPoint.y;
  }
  else if (x_changes == 2 && y_changes == 0)
  {
    coefficients.x.p3 =
        ((endPoint.az + startPoint.az) * endPoint.y + (-endPoint.az - startPoint.az) * startPoint.y -
         2 * startPoint.az * endPoint.az * endPoint.x + 2 * startPoint.az * endPoint.az * startPoint.x) /
        (startPoint.az * endPoint.az);
    coefficients.x.p2 =
        -((2 * endPoint.az + startPoint.az) * endPoint.y + (-2 * endPoint.az - startPoint.az) * startPoint.y -
          3 * startPoint.az * endPoint.az * endPoint.x + 3 * startPoint.az * endPoint.az * startPoint.x) /
        (startPoint.az * endPoint.az);
    coefficients.x.p1 = -(startPoint.y - endPoint.y) / startPoint.az;
    coefficients.x.p0 = startPoint.x;
    coefficients.y.p3 = 0.0;
    coefficients.y.p2 = 0.0;
    coefficients.y.p1 = endPoint.y - startPoint.y;
    coefficients.y.p0 = startPoint.y;
  }
  else if (x_changes == 1 && y_changes == 2)
  {
    coefficients.x.p3 = 0.0;
    coefficients.x.p2 = -((6 * pow(inflection, 2) - 6 * inflection) * endPoint.y +
                          (6 * inflection - 6 * pow(inflection, 2)) * startPoint.y +
                          (startPoint.az * (-3 * pow(inflection, 2) + 4 * inflection - 1) +
                           endPoint.az * (2 * inflection - 3 * pow(inflection, 2))) *
                              endPoint.x +
                          (endPoint.az * (3 * pow(inflection, 2) - 2 * inflection) +
                           startPoint.az * (3 * pow(inflection, 2) - 4 * inflection + 1)) *
                              startPoint.x) /
                        (startPoint.az * (3 * pow(inflection, 2) - 4 * inflection + 1) +
                         endPoint.az * (2 * inflection - 3 * pow(inflection, 2)));
    coefficients.x.p1 = ((6 * pow(inflection, 2) - 6 * inflection) * endPoint.y +
                         (6 * inflection - 6 * pow(inflection, 2)) * startPoint.y +
                         endPoint.az * (4 * inflection - 6 * pow(inflection, 2)) * endPoint.x +
                         endPoint.az * (6 * pow(inflection, 2) - 4 * inflection) * startPoint.x) /
                        (startPoint.az * (3 * pow(inflection, 2) - 4 * inflection + 1) +
                         endPoint.az * (2 * inflection - 3 * pow(inflection, 2)));
    coefficients.x.p0 = startPoint.x;
    coefficients.y.p3 = ((startPoint.az * (2 * inflection - 2) + 2 * endPoint.az * inflection) * endPoint.y +
                         (startPoint.az * (2 - 2 * inflection) - 2 * endPoint.az * inflection) * startPoint.y +
                         startPoint.az * endPoint.az * (2 - 4 * inflection) * endPoint.x +
                         startPoint.az * endPoint.az * (4 * inflection - 2) * startPoint.x) /
                        (startPoint.az * (3 * pow(inflection, 2) - 4 * inflection + 1) +
                         endPoint.az * (2 * inflection - 3 * pow(inflection, 2)));
    coefficients.y.p2 =
        -((startPoint.az * (3 * pow(inflection, 2) - 3) + 3 * endPoint.az * pow(inflection, 2)) * endPoint.y +
          (startPoint.az * (3 - 3 * pow(inflection, 2)) - 3 * endPoint.az * pow(inflection, 2)) * startPoint.y +
          startPoint.az * endPoint.az * (2 - 6 * pow(inflection, 2)) * endPoint.x +
          startPoint.az * endPoint.az * (6 * pow(inflection, 2) - 2) * startPoint.x) /
        (startPoint.az * (3 * pow(inflection, 2) - 4 * inflection + 1) +
         endPoint.az * (2 * inflection - 3 * pow(inflection, 2)));
    coefficients.y.p1 = (startPoint.az * (6 * pow(inflection, 2) - 6 * inflection) * endPoint.y +
                         startPoint.az * (6 * inflection - 6 * pow(inflection, 2)) * startPoint.y +
                         startPoint.az * endPoint.az * (4 * inflection - 6 * pow(inflection, 2)) * endPoint.x +
                         startPoint.az * endPoint.az * (6 * pow(inflection, 2) - 4 * inflection) * startPoint.x) /
                        (startPoint.az * (3 * pow(inflection, 2) - 4 * inflection + 1) +
                         endPoint.az * (2 * inflection - 3 * pow(inflection, 2)));
    coefficients.y.p0 = startPoint.y;
  }
  else if (x_changes == 0 && y_changes == 2)
  {
    coefficients.x.p3 = 0.0;
    coefficients.x.p2 = 0.0;
    coefficients.x.p1 = endPoint.x - startPoint.x;
    coefficients.x.p0 = startPoint.x;
    coefficients.y.p3 = -2 * endPoint.y + 2 * startPoint.y + (endPoint.az + startPoint.az) * endPoint.x +
                        (-endPoint.az - startPoint.az) * startPoint.x;
    coefficients.y.p2 = 3 * endPoint.y - 3 * startPoint.y + (-endPoint.az - 2 * startPoint.az) * endPoint.x +
                        (endPoint.az + 2 * startPoint.az) * startPoint.x;
    coefficients.y.p1 = startPoint.az * endPoint.x - startPoint.az * startPoint.x;
    coefficients.y.p0 = startPoint.y;
  }
  else
  {
    coefficients.x.p3 = 0.0;
    coefficients.x.p2 = -(2 * endPoint.y - 2 * startPoint.y + (-endPoint.az - startPoint.az) * endPoint.x +
                          (endPoint.az + startPoint.az) * startPoint.x) /
                        (startPoint.az - endPoint.az);
    coefficients.x.p1 =
        (2 * endPoint.y - 2 * startPoint.y - 2 * endPoint.az * endPoint.x + 2 * endPoint.az * startPoint.x) /
        (startPoint.az - endPoint.az);
    coefficients.x.p0 = startPoint.x;
    coefficients.y.p3 = 0.0;
    coefficients.y.p2 =
        -((endPoint.az + startPoint.az) * endPoint.y + (-endPoint.az - startPoint.az) * startPoint.y -
          2 * startPoint.az * endPoint.az * endPoint.x + 2 * startPoint.az * endPoint.az * startPoint.x) /
        (startPoint.az - endPoint.az);
    coefficients.y.p1 =
        (2 * startPoint.az * endPoint.y - 2 * startPoint.az * startPoint.y -
         2 * startPoint.az * endPoint.az * endPoint.x + 2 * startPoint.az * endPoint.az * startPoint.x) /
        (startPoint.az - endPoint.az);
    coefficients.y.p0 = startPoint.y;
  }
  return coefficients;
}

int PolynomialPlanner::countChangesX(pos startPoint, pos endPoint)
{
  int changes = 0;
  bool path_growing, start_growing, end_growing;
  path_growing = start_growing = end_growing = false;
  if (endPoint.x > startPoint.x)
    path_growing = true;
  if (abs(endPoint.az) < M_PI_2)
    end_growing = true;
  if (abs(startPoint.az) < M_PI_2)
    start_growing = true;
  if (path_growing)
  {
    if (!end_growing)
      changes++;
    if (!start_growing)
      changes++;
  }
  else
  {
    if (end_growing)
      changes++;
    if (start_growing)
      changes++;
  }
  return changes;
}

int PolynomialPlanner::countChangesY(pos startPoint, pos endPoint)
{
  int changes = 0;
  bool path_growing, start_growing, end_growing;
  path_growing = start_growing = end_growing = false;
  if (endPoint.y > startPoint.y)
    path_growing = true;
  if (endPoint.az > 0)
    end_growing = true;
  if (startPoint.az > 0)
    start_growing = true;
  if (path_growing)
  {
    if (!end_growing)
      changes++;
    if (!start_growing)
      changes++;
  }
  else
  {
    if (end_growing)
      changes++;
    if (start_growing)
      changes++;
  }
  return changes;
}
}