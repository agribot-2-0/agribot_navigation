/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_navigation/common/common_functions.h>
#include <agribot_navigation/kinematics_control/KinematicsControl.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <cmath>

#define DEBUG
#define HISTERESIS 0.05
#define RAD_TO_DEG (180 / M_PI)
#define DEG_TO_RAD (M_PI / 180)
#define ANGLE_RANGE (M_PI / 2)
#define LINEARITY_RANGE 5.0
#define SP_RAMP_SUBSTEP 0.225
#define SP_RAMP_SUBSTEPS_NUMBER 20
#define ANGULAR_CHANGE_THRESHOLD 0.5
#define ANGULAR_MULTIPLIER 2.0

KinematicsControl::KinematicsControl()
{
  this->cart_angle_[FRONT_CART] = 0;
  this->cart_angle_[BACK_CART] = 0;
  this->chassis_angle_ = 0;
  this->wheel_radius_ = 0.16;
  this->wheel_separation_ = 1.05 + 0.025;
  this->chassis_length_ = 2.42 - 2 * wheel_radius_;
  this->linear_multiplier_ = 1;

  this->front_cart_coefficients_.kp_ = 0.0;
  this->back_cart_coefficients_.kp_ = 0.0;
  this->front_cart_coefficients_.ki_ = 0.0;
  this->back_cart_coefficients_.ki_ = 0.0;
  this->front_cart_coefficients_.kd_ = 0.0;
  this->back_cart_coefficients_.kd_ = 0.0;

  this->front_limit_ = 0.0;
  this->back_limit_ = 0.0;

  this->precision_range_ = 0.0;

  this->front_cart_pid_ = new PID();
  this->back_cart_pid_ = new PID();

  setSteeringMode("four_wheel");
}

void KinematicsControl::createPubs(ros::NodeHandle* n)
{
#ifdef DEBUG
  front_set_point_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/SetPoint", 10);
  front_control_value_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/ControlValue", 10);
  front_process_value_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/ProcessValue", 10);
  front_error_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/Error", 10);
  front_integral_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/IntegrationError", 10);
  front_derivative_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/DerivativeError", 10);
  front_des_set_point_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/DesiredSetPoint", 10);

  back_set_point_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/backPID/SetPoint", 10);
  back_control_value_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/backPID/ControlValue", 10);
  back_process_value_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/backPID/ProcessValue", 10);
  back_error_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/backPID/Error", 10);
  back_integral_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/backPID/IntegrationError", 10);
  back_derivative_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/backPID/DerivativeError", 10);
  back_des_set_point_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/backPID/DesiredSetPoint", 10);

  front_vel_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/limiter/FrontVelocity", 10);
  back_vel_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/limiter/BackVelocity", 10);
  multiplier_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/limiter/Multiplier", 10);
  set_point_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/limiter/SetPoint", 10);
  front_dif_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/limiter/FrontDifference", 10);
  back_dif_pub_ = n->advertise<std_msgs::Float64>("agribot/kin_controler/limiter/BackDifference", 10);
#endif
}

int KinematicsControl::configure()
{
  return 0;
}

bool KinematicsControl::startup()
{
  return false;
}

bool KinematicsControl::update()
{
  this->calculateKinematicsControl();
  return true;
}

bool KinematicsControl::stop()
{
  return false;
}

bool KinematicsControl::cleanup()
{
  return false;
}

PidTermsCoefficients KinematicsControl::getFrontPIDCoefficients()
{
  PidTermsCoefficients coefficients;

  coefficients = { this->front_cart_coefficients_.kp_, this->front_cart_coefficients_.ki_,
                   this->front_cart_coefficients_.kd_ };

  return coefficients;
}

PidTermsCoefficients KinematicsControl::getBackPIDCoefficients()
{
  PidTermsCoefficients coefficients;

  coefficients = { this->back_cart_coefficients_.kp_, this->back_cart_coefficients_.ki_,
                   this->back_cart_coefficients_.kd_ };

  return coefficients;
}

void KinematicsControl::setPidsParameters()
{
  PidTermsCoefficients coefficients;

  coefficients = getFrontPIDCoefficients();
  this->front_cart_pid_->setKp(coefficients.kp_);
  this->front_cart_pid_->setKi(coefficients.ki_);
  this->front_cart_pid_->setKd(coefficients.kd_);

  coefficients = getBackPIDCoefficients();
  this->back_cart_pid_->setKp(coefficients.kp_);
  this->back_cart_pid_->setKi(coefficients.ki_);
  this->back_cart_pid_->setKd(coefficients.kd_);

  this->front_cart_pid_->setSpAcceleration(SP_RAMP_SUBSTEP * DEG_TO_RAD, SP_RAMP_SUBSTEPS_NUMBER);
  this->back_cart_pid_->setSpAcceleration(SP_RAMP_SUBSTEP * DEG_TO_RAD, SP_RAMP_SUBSTEPS_NUMBER);
}

Carts_Setpoints KinematicsControl::xCalculatePIDSetPoints(double linear_vel, double angular_vel)
{
  Carts_Setpoints SP;
  // provide PIDs with Set Points, basing on received v and w
  if (angular_vel != 0)
  {
    // Angular - yes
    if (linear_vel != 0)
    {
      // Angular - yes / Linear -yes
      if (angular_vel * chassis_length_ / 2 / linear_vel > 1)
      {
        // Desired radius too small
        SP = { M_PI / 2, -M_PI / 2 };
        ROS_WARN_NAMED("agribot_navigation", "Radius = %f, lower than half of chasis length = %f",
                       linear_vel / angular_vel, chassis_length_ / 2);
      }
      else
      {
        if (angular_vel * chassis_length_ / 2 / linear_vel < -1)
        {
          // Desired radius too small
          SP = { -M_PI / 2, M_PI / 2 };
          ROS_WARN_NAMED("agribot_navigation", "Radius = %f, lower than half of chasis length = %f",
                         linear_vel / angular_vel, chassis_length_ / 2);
        }
        else
        {
          // Desired radius within proper range
          if (linear_vel > 0)
          {
            // Moving forward - back cart is pulled
            SP = getSetPoints(DrivingDirection::forward, velocitiesToRadius(linear_vel, angular_vel));
          }
          else
          {
            SP = getSetPoints(DrivingDirection::backward, velocitiesToRadius(linear_vel, angular_vel));
          }
        }
      }
    }
    else
    {
      // Angular - yes / Linear -no
      SP = { M_PI / 2, -M_PI / 2 };
    }
  }
  else
  {
    // Angular - no
    if (linear_vel != 0)
    {
      // Angular - no / Linear -yes
      SP = { 0, 0 };
    }
    else
    {
      // Angular - no / Linear -no
      SP = { 0, 0 };
    }
  }
  return SP;
}

Carts_Setpoints KinematicsControl::getSetPoints(const DrivingDirection& direction, const double radius) const
{
  if (direction == DrivingDirection::forward)
  {
    if (steering_mode_ == SteeringMode::four_wheel)
    {
      return { radiusToAngle(radius), -cart_angle_[FRONT_CART] };
    }
    else if (steering_mode_ == SteeringMode::front_wheel)
    {
      return { radiusToAngle(radius), 0 };
    }
    else if (steering_mode_ == SteeringMode::crab)
    {
      return { radiusToAngle(radius), radiusToAngle(radius) };
    }
    else
    {
      return { 0, 0 };
    }
  }
  else
  {
    if (steering_mode_ == SteeringMode::four_wheel)
    {
      return { -cart_angle_[BACK_CART], -radiusToAngle(radius) };
    }
    else if (steering_mode_ == SteeringMode::front_wheel)
    {
      return { 0, -radiusToAngle(radius) };
    }
    else if (steering_mode_ == SteeringMode::crab)
    {
      return { -radiusToAngle(radius), -radiusToAngle(radius) };
    }
    else
    {
      return { 0, 0 };
    }
  }
}

double KinematicsControl::calculateLinearControls(double linear_vel, double angular_vel)
{
  if (angular_vel != 0)
  {
    // Angular - yes
    if (linear_vel != 0)
    {
      // Angular - yes / Linear -yes
      control_[0] = linear_vel;
    }
    else
    {
      // Angular - yes / Linear -no
      control_[0] = angular_vel * chassis_length_ / 2;
    }
  }
  else
  {
    if (linear_vel != 0)
    {
      // Angular - no / Linear -yes
      control_[0] = linear_vel;
    }
    else
    {
      // Angular - no / Linear -no
      control_[0] = 0.0;
    }
  }
  return control_[0];
}

double KinematicsControl::calculateFrontAngularControl(double linear_vel, double angular_vel)
{
  updatePids(ros::Time::now().toSec());
  if (linear_vel == 0 && angular_vel == 0)
  {
    // Stopped
    return 0;
  }
  else
  {
    // Moving
    return this->front_cart_pid_->getControlValue();
  }
}

double KinematicsControl::calculateBackAngularControl(double linear_vel, double angular_vel)
{
  updatePids(ros::Time::now().toSec());
  if (linear_vel == 0 && angular_vel == 0)
  {
    // Stopped
    return 0;
  }
  else
  {
    // Moving
    return this->back_cart_pid_->getControlValue();
  }
}

void KinematicsControl::setPidsSetpoints()
{
  Carts_Setpoints CartsSetPoints;
  CartsSetPoints = this->xCalculatePIDSetPoints(input_linear_[X], input_angular_[Z]);
  this->front_cart_pid_->setSp(CartsSetPoints.front_cart_sp_);
  this->back_cart_pid_->setSp(CartsSetPoints.back_cart_sp_);

  if (input_linear_[X] == 0 && input_angular_[Z] == 0)
  {
    this->front_cart_pid_->resetError();
    this->back_cart_pid_->resetError();
    this->front_cart_pid_->resetSetPoints(cart_angle_[FRONT_CART]);
    this->back_cart_pid_->resetSetPoints(cart_angle_[BACK_CART]);
  }
}

double KinematicsControl::calculateAngularMultiplier(double linear_multiplier)
{
  if (linear_multiplier < ANGULAR_CHANGE_THRESHOLD)
  {
    return ANGULAR_MULTIPLIER;
  }
  else
  {
    return 1.0;
  }
}

double KinematicsControl::velocitiesToRadius(double linear, double angular)
{
  return linear / angular;
}

double KinematicsControl::radiusToAngle(const double Radius) const
{
  double angle{ 0 };
  if (steering_mode_ == SteeringMode::front_wheel)
  {
    angle = atan(chassis_length_ / Radius);
  }
  else
  {
    angle = asin(chassis_length_ / 2.0 / Radius);
  }
  return angle;
}

double KinematicsControl::multiplierLinear(double x)
{
  return 1 - x;
}

double KinematicsControl::multiplierGaussian(double x)
{
  return 0.04379209 + (0.97703891) / (1 + pow((x / 0.1482077), 2.181746));
}

double KinematicsControl::multiplierSigmoid(double x)
{
  return 1.092919 - (5.091961 / 4.758651) * (1 - exp(-4.758651 * x));
}

double KinematicsControl::multiplierExponential(double x)
{
  return 1.0875 * exp(-4.52068 * x);
}

double KinematicsControl::multiplierRoot2(double x)
{
  return 1 - (pow(x, (1 / 2.0)));
}

double KinematicsControl::multiplierRoot3(double x)
{
  return 1 - (pow(x, (1 / 3.0)));
}

double KinematicsControl::multiplierRoot4(double x)
{
  return 1 - (pow(x, (1 / 4.0)));
}

double KinematicsControl::frontCartAngleDifference()
{
  return abs(cart_angle_[FRONT_CART] - front_cart_pid_->getDesiredSetPoint());
}

double KinematicsControl::backCartAngleDifference()
{
  return abs(cart_angle_[BACK_CART] - back_cart_pid_->getDesiredSetPoint());
}

double KinematicsControl::maximumCartAngleDifference(double diff_front, double diff_back)
{
  return std::max(diff_front, diff_back);
}

void KinematicsControl::calculateKinematicsControl()
{
  this->setPidsParameters();
  this->setPidsSetpoints();

  control_[0] = this->calculateLinearControls(input_linear_[X], input_angular_[Z]);
  control_[1] = this->calculateFrontAngularControl(input_linear_[X], input_angular_[Z]);
  control_[2] = this->calculateBackAngularControl(input_linear_[X], input_angular_[Z]);

  this->linear_multiplier_ = this->multiplierGaussian(
      this->maximumCartAngleDifference(this->frontCartAngleDifference(), this->backCartAngleDifference()) /
      ANGLE_RANGE);
  this->angular_multiplier_ = this->calculateAngularMultiplier(this->linear_multiplier_);

  caterpillar_velocities_[FRONT_LINEAR] = this->linear_multiplier_ * control_[0];
  caterpillar_velocities_[FRONT_ANGULAR] = this->angular_multiplier_ * control_[1];
  caterpillar_velocities_[BACK_LINEAR] =
      this->linear_multiplier_ * cos(cart_angle_[FRONT_CART]) / cos(cart_angle_[BACK_CART]) * control_[0];
  caterpillar_velocities_[BACK_ANGULAR] = this->angular_multiplier_ * control_[2];

#ifdef DEBUG
  std_msgs::Float64 message_front_SP;
  std_msgs::Float64 message_front_CV;
  std_msgs::Float64 message_front_PV;
  std_msgs::Float64 message_front_ER;
  std_msgs::Float64 message_front_IE;
  std_msgs::Float64 message_front_DE;
  std_msgs::Float64 message_front_DSP;
  message_front_SP.data = front_cart_pid_->getSetPoint() * RAD_TO_DEG;
  message_front_CV.data = front_cart_pid_->getControlValue() * RAD_TO_DEG;
  message_front_PV.data = front_cart_pid_->getProcessValue() * RAD_TO_DEG;
  message_front_ER.data = front_cart_pid_->getProportionalPart() * RAD_TO_DEG;
  message_front_IE.data = front_cart_pid_->getIntegralPart() * RAD_TO_DEG;
  message_front_DE.data = front_cart_pid_->getDerivativePart() * RAD_TO_DEG;
  message_front_DSP.data = front_cart_pid_->getDesiredSetPoint() * RAD_TO_DEG;

  std_msgs::Float64 message_back_SP;
  std_msgs::Float64 message_back_CV;
  std_msgs::Float64 message_back_PV;
  std_msgs::Float64 message_back_ER;
  std_msgs::Float64 message_back_IE;
  std_msgs::Float64 message_back_DE;
  std_msgs::Float64 message_back_DSP;
  message_back_SP.data = back_cart_pid_->getSetPoint() * RAD_TO_DEG;
  message_back_CV.data = back_cart_pid_->getControlValue() * RAD_TO_DEG;
  message_back_PV.data = back_cart_pid_->getProcessValue() * RAD_TO_DEG;
  message_back_ER.data = back_cart_pid_->getProportionalPart() * RAD_TO_DEG;
  message_back_IE.data = back_cart_pid_->getIntegralPart() * RAD_TO_DEG;
  message_back_DE.data = back_cart_pid_->getDerivativePart() * RAD_TO_DEG;
  message_back_DSP.data = back_cart_pid_->getDesiredSetPoint() * RAD_TO_DEG;

  std_msgs::Float64 message_linear_FV;
  std_msgs::Float64 message_linear_BV;
  std_msgs::Float64 message_linear_MP;
  std_msgs::Float64 message_linear_SP;
  std_msgs::Float64 message_linear_FD;
  std_msgs::Float64 message_linear_BD;
  message_linear_FV.data = caterpillar_velocities_[FRONT_LINEAR];
  message_linear_BV.data = caterpillar_velocities_[BACK_LINEAR];
  message_linear_MP.data = this->linear_multiplier_;
  message_linear_SP.data = control_[0];
  message_linear_FD.data = this->frontCartAngleDifference() * RAD_TO_DEG;
  message_linear_BD.data = this->backCartAngleDifference() * RAD_TO_DEG;

  front_set_point_pub_.publish(message_front_SP);
  front_control_value_pub_.publish(message_front_CV);
  front_process_value_pub_.publish(message_front_PV);
  front_error_pub_.publish(message_front_ER);
  front_integral_pub_.publish(message_front_IE);
  front_derivative_pub_.publish(message_front_DE);
  front_des_set_point_pub_.publish(message_front_DSP);

  back_set_point_pub_.publish(message_back_SP);
  back_control_value_pub_.publish(message_back_CV);
  back_process_value_pub_.publish(message_back_PV);
  back_error_pub_.publish(message_back_ER);
  back_integral_pub_.publish(message_back_IE);
  back_derivative_pub_.publish(message_back_DE);
  back_des_set_point_pub_.publish(message_back_DSP);

  front_vel_pub_.publish(message_linear_FV);
  back_vel_pub_.publish(message_linear_BV);
  multiplier_pub_.publish(message_linear_MP);
  set_point_pub_.publish(message_linear_SP);
  front_dif_pub_.publish(message_linear_FD);
  back_dif_pub_.publish(message_linear_BD);
#endif
}

double* KinematicsControl::getCaterpillarVelocities()
{
  return this->caterpillar_velocities_;
}

void KinematicsControl::setInputLinear(double x, double y, double z)
{
  this->input_linear_[X] = x;
  this->input_linear_[Y] = y;
  this->input_linear_[Z] = z;
}

void KinematicsControl::setInputAngular(double x, double y, double z)
{
  this->input_angular_[X] = x;
  this->input_angular_[Y] = y;
  this->input_angular_[Z] = z;
}

void KinematicsControl::setChassisAngle(double theta)
{
  this->chassis_angle_ = theta;
}

void KinematicsControl::setFrontCartAngle(double theta)
{
  this->cart_angle_[FRONT_CART] = theta;
}

void KinematicsControl::setBackCartAngle(double theta)
{
  this->cart_angle_[BACK_CART] = theta;
}

void KinematicsControl::updatePids(double t)
{
  double dt = t - this->last_time_;
  this->front_cart_pid_->Update(cart_angle_[FRONT_CART], dt);
  this->back_cart_pid_->Update(cart_angle_[BACK_CART], dt);
  this->last_time_ = t;
}

void KinematicsControl::setFrontKp(double Kp)
{
  this->front_cart_coefficients_.kp_ = Kp;
}

void KinematicsControl::setBackKp(double Kp)
{
  this->back_cart_coefficients_.kp_ = Kp;
}

void KinematicsControl::setFrontKi(double Ki)
{
  this->front_cart_coefficients_.ki_ = Ki;
}

void KinematicsControl::setBackKi(double Ki)
{
  this->back_cart_coefficients_.ki_ = Ki;
}

void KinematicsControl::setFrontKd(double Kd)
{
  this->front_cart_coefficients_.kd_ = Kd;
}

void KinematicsControl::setBackKd(double Kd)
{
  this->back_cart_coefficients_.kd_ = Kd;
}

void KinematicsControl::setFrontLimit(double limit)
{
  this->front_limit_ = limit * DEG_TO_RAD;
  this->front_cart_pid_->setLowLimit(-this->front_limit_);
  this->front_cart_pid_->setHighLimit(this->front_limit_);
}

void KinematicsControl::setBackLimit(double limit)
{
  this->back_limit_ = limit * DEG_TO_RAD;
  this->back_cart_pid_->setLowLimit(-this->back_limit_);
  this->back_cart_pid_->setHighLimit(this->back_limit_);
}

void KinematicsControl::setPrecisionRange(double range)
{
  this->precision_range_ = range;
}

void KinematicsControl::setSteeringMode(const std::string& mode)
{
  if (mode == "four_wheel")
  {
    steering_mode_ = SteeringMode::four_wheel;
  }
  else if (mode == "front_wheel")
  {
    steering_mode_ = SteeringMode::front_wheel;
  }
  else if (mode == "crab")
  {
    steering_mode_ = SteeringMode::crab;
  }
}
