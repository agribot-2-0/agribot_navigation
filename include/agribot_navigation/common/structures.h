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
  // y = p3*x^3+p2*x^2+p1*x+p0
  double p0, p1, p2, p3;
};

struct parametric_coefficients
{
  polynomial_3 x;
  polynomial_3 y;
};
}  // namespace agribot_local_planner
