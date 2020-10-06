/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

constexpr double DEFAULT_GOAL_DISTANCE_TOLERANCE = 0.75;       // meters
constexpr double DEFAULT_GOAL_RANGE = 0.66;                    // factor
constexpr int DEFAULT_MISSED_POINTS_TOLLERANCE = 10000;        // n
constexpr double DEFAULT_CHASSIS_LENGTH = 2.1;                 // meters
constexpr double DEFAULT_MIN_RADIUS = 2.0;                     // meters
constexpr int DEFAULT_PREDICTION_TIME = 60;                    // seconds
constexpr int DEFAULT_PREDICTION_POINTS_PER_SECOND = 2;        // n/second
constexpr double DEFAULT_DEAD_ZONE = 3.0;                      // degrees
constexpr double DEFAULT_MAX_LINEAR_VEL = 0.7;                 // m/s
constexpr double DEFAULT_MIN_LINEAR_VEL = 0.1;                 // m/s
constexpr double DEFAULT_DEVIATION_WEIGHT_STRAIGHT = 0.1;      // factor
constexpr double DEFAULT_PARALLELNESS_WEIGHT_STRAIGHT = 10.0;  // factor
constexpr double DEFAULT_DEVIATION_WEIGHT_ARCH = 0.1;          // factor
constexpr double DEFAULT_PARALLELNESS_WEIGHT_ARCH = 10.0;      // factor
constexpr double DEFAULT_EPSILON = 1e-3;                       // unitless
constexpr double DEFAULT_DESIGN_DEAD_ZONE = 0.0;               // degrees
constexpr double DEFAULT_DESIGN_ARCH_RADIUS = 2.0;             // meters
constexpr double DEFAULT_DESIGN_STEP_SIZE = 0.1;               // meters
constexpr double DEFAULT_ODOM_FILTER_TIME = 0.1;               // seconds
constexpr double DEFAULT_MISSED_DISTANCE_COEFFICIENT = 1.1;    // factor
constexpr double DEFAULT_INFLECTION_POINT = 0.5;               // factor (0,1)
