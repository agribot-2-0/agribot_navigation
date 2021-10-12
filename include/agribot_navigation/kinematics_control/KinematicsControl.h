/*
 * Copyright (C) 2020-2021 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_common/AgribotComponent.h>
#include <agribot_msgs/AgribotCaterpillarWheelsRotation.h>
#include <agribot_navigation/kinematics_control/PID.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <memory>
#include <string>

enum
{
  X,
  Y,
  Z,
};

enum
{
  FRONT_LINEAR,
  FRONT_ANGULAR,
  BACK_LINEAR,
  BACK_ANGULAR
};

enum
{
  FRONT_LEFT,
  FRONT_RIGHT,
  BACK_LEFT,
  BACK_RIGHT
};

enum
{
  FRONT_CART,
  BACK_CART
};

enum class SteeringMode
{
  four_wheel,
  front_wheel,
  crab
};

enum class DrivingDirection
{
  forward,
  backward
};

struct PidTermsCoefficients
{
  double kp_;
  double ki_;
  double kd_;
};

struct Carts_Setpoints
{
  double front_cart_sp_;
  double back_cart_sp_;
};

class KinematicsControl : public AgribotComponent
{
private:
  ros::Publisher front_set_point_pub_;
  ros::Publisher front_control_value_pub_;
  ros::Publisher front_process_value_pub_;
  ros::Publisher front_error_pub_;
  ros::Publisher front_integral_pub_;
  ros::Publisher front_derivative_pub_;
  ros::Publisher front_des_set_point_pub_;

  ros::Publisher back_set_point_pub_;
  ros::Publisher back_control_value_pub_;
  ros::Publisher back_process_value_pub_;
  ros::Publisher back_error_pub_;
  ros::Publisher back_integral_pub_;
  ros::Publisher back_derivative_pub_;
  ros::Publisher back_des_set_point_pub_;

  ros::Publisher front_vel_pub_;
  ros::Publisher back_vel_pub_;
  ros::Publisher multiplier_pub_;
  ros::Publisher set_point_pub_;
  ros::Publisher front_dif_pub_;
  ros::Publisher back_dif_pub_;

  ros::Publisher track_encoder_sum_publisher_;

  geometry_msgs::Twist input_velocity_;
  double input_linear_[3];
  double input_angular_[3];
  double caterpillar_velocities_[4];
  double caterpillar_independent_velocities_[4];
  double control_[3];
  double cart_angle_[2];
  double chassis_angle_;
  double wheel_separation_distance_;
  double wheel_radius_;
  double robot_axis_to_axis_length;
  double last_time_;
  double linear_multiplier_;
  double angular_multiplier_;

  PidTermsCoefficients front_cart_coefficients_;
  PidTermsCoefficients back_cart_coefficients_;
  double front_limit_;
  double back_limit_;
  std::unique_ptr<PID> front_cart_pid_;
  std::unique_ptr<PID> back_cart_pid_;
  SteeringMode steering_mode_;

  void setPidsParameters();
  double calculateLinearControls(double linear_vel, double angular_vel);
  double calculateFrontAngularControl(double linear_vel, double angular_vel);
  double calculateBackAngularControl(double linear_vel, double angular_vel);
  double calculateAngularMultiplier(double linear_multiplier);
  void updatePids(double t);
  void setPidsSetpoints();
  void calculateKinematicsControl();
  double frontCartAngleDifference();
  double backCartAngleDifference();
  double maximumCartAngleDifference(double diff_front, double diff_back);
  double velocitiesToRadius(double linear, double angular);
  double radiusToAngle(const double Radius) const;
  double multiplierGaussian(double x);
  double multiplierSigmoid(double x);
  double multiplierRoot2(double x);
  double multiplierRoot3(double x);
  double multiplierRoot4(double x);
  double multiplierLinear(double x);
  double multiplierExponential(double x);
  Carts_Setpoints xCalculatePIDSetPoints(double linear_vel, double angular_vel);
  Carts_Setpoints getSetPoints(const DrivingDirection& direction, const double radius) const;
  PidTermsCoefficients getFrontPIDCoefficients();
  PidTermsCoefficients getBackPIDCoefficients();

  double robot_axis_to_axis_length_ = 2.1;

public:
  KinematicsControl();
  virtual int configure();
  virtual bool startup();
  virtual bool update();
  virtual bool stop();
  virtual bool cleanup();
  void createPubs(ros::NodeHandle* n);
  void setInputLinear(double x, double y, double z);
  void setInputAngular(double x, double y, double z);
  void setInputVelocity(const geometry_msgs::Twist& velocity);
  void setChassisAngle(double theta);
  void setFrontCartAngle(double theta);
  void setBackCartAngle(double theta);
  double* getCaterpillarVelocities();
  void setFrontKp(double Kp);
  void setBackKp(double Kp);
  void setFrontKi(double Ki);
  void setBackKi(double Ki);
  void setFrontKd(double Kd);
  void setBackKd(double Kd);
  void setFrontLimit(double limit);
  void setBackLimit(double limit);
  void setSteeringMode(const std::string& mode);
};
