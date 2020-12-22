/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

constexpr double default_goal_distance_tolerance = 0.75;       // meters
constexpr double default_goal_range = 0.66;                    // factor
constexpr int default_missed_points_tolerance = 10;            // n
constexpr double default_chassis_length = 2.1;                 // meters
constexpr double default_min_radius = 2.0;                     // meters
constexpr int default_prediction_time = 60;                    // seconds
constexpr int default_prediction_points_per_second = 2;        // n/second
constexpr double default_dead_zone = 3.0;                      // degrees
constexpr double default_max_linear_vel = 1.5;                 // m/s
constexpr double default_min_linear_vel = 0.1;                 // m/s
constexpr double default_deviation_weight_straight = 5.0;      // factor
constexpr double default_parallelness_weight_straight = 90.0;  // factor
constexpr double default_deviation_weight_arc = 0.1;           // factor
constexpr double default_parallelness_weight_arc = 10.0;       // factor
constexpr double default_epsilon = 1e-3;                       // unitless
constexpr double default_design_dead_zone = 0.0;               // degrees
constexpr double default_design_arc_radius = 2.0;              // meters
constexpr double default_step_size = 0.1;                      // meters
constexpr double default_odom_filter_time = 0.1;               // seconds
constexpr double default_missed_distance_coefficient = 1.1;    // factor
constexpr double default_hermite_magnitude = 20.0;             // factor
constexpr double default_pure_pursuit_curvature = 10.0;        // factor
constexpr double default_pure_pursuit_rabbit_distance = 5.0;   // meters
constexpr double default_stanley_error_gain = 0.1;             // factor
constexpr double default_tracker_side_ray_length = 1.0;        // meters
constexpr double default_tracker_side_ray_width = 90.0;        // degrees
constexpr double default_tracker_front_ray_length = 25.0;      // meters
constexpr double default_tracker_front_ray_width = 10.0;       // degrees
