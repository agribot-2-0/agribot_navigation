/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

namespace agribot_local_planner
{
struct pos
{
  double x, y, az;
};

struct polynomial_3
{
  // y = a*x^3+b*x^2+c*x+d
  double p0, p1, p2, p3;
};

struct parametric_coeficients
{
  polynomial_3 x;
  polynomial_3 y;
};
}  // namespace agribot_local_planner
