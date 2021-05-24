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
  this->_pdCartAngle[FRONT_CART] = 0;
  this->_pdCartAngle[BACK_CART] = 0;
  this->_dChassisAngle = 0;
  this->_dWheelRadius = 0.16;
  this->_dWheelSeparation = 1.05 + 0.025;
  this->_dChassisLength = 2.42 - 2 * _dWheelRadius;
  this->_dLinearMultiplier = 1;

  this->_dCoarseKp = 0.0;
  this->_dCoarseKi = 0.0;
  this->_dCoarseKd = 0.0;
  this->_dPreciseKp = 0.0;
  this->_dPreciseKd = 0.0;
  this->_dPreciseKi = 0.0;

  this->_dFrontLimit = 0.0;
  this->_dBackLimit = 0.0;

  this->_dPrecisionRange = 0.0;

  this->_pFrontCartPID = new PID();
  this->_pBackCartPID = new PID();

  setSteeringMode(SteeringMode::four_wheel);
}

void KinematicsControl::vCreatePubs(ros::NodeHandle n)
{
#ifdef DEBUG
  _rFrontSetPointPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/SetPoint", 10);
  _rFrontControlValuePub = n.advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/ControlValue", 10);
  _rFrontProcessValuePub = n.advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/ProcessValue", 10);
  _rFrontErrorPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/Error", 10);
  _rFrontIntegralPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/IntegrationError", 10);
  _rFrontDerivativePub = n.advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/DerivativeError", 10);
  _rFrontDesSetPointPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/frontPID/DesiredSetPoint", 10);

  _rBackSetPointPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/backPID/SetPoint", 10);
  _rBackControlValuePub = n.advertise<std_msgs::Float64>("agribot/kin_controler/backPID/ControlValue", 10);
  _rBackProcessValuePub = n.advertise<std_msgs::Float64>("agribot/kin_controler/backPID/ProcessValue", 10);
  _rBackErrorPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/backPID/Error", 10);
  _rBackIntegralPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/backPID/IntegrationError", 10);
  _rBackDerivativePub = n.advertise<std_msgs::Float64>("agribot/kin_controler/backPID/DerivativeError", 10);
  _rBackDesSetPointPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/backPID/DesiredSetPoint", 10);

  _rFrontVelPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/limiter/FrontVelocity", 10);
  _rBackVelPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/limiter/BackVelocity", 10);
  _rMultipiterPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/limiter/Multiplier", 10);
  _rSetPointPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/limiter/SetPoint", 10);
  _rFrontDifPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/limiter/FrontDifference", 10);
  _rBackDifPub = n.advertise<std_msgs::Float64>("agribot/kin_controler/limiter/BackDifference", 10);
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
  this->vCalculateKinematicsControl();
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

PID_terms_coefficients KinematicsControl::xSelectPIDCoefficients(double Error)
{
  PID_terms_coefficients coefficients;
  if (abs(Error * RAD_TO_DEG) > this->_dPrecisionRange)
  {
    coefficients = { this->_dCoarseKp, this->_dCoarseKi, this->_dCoarseKd };
  }
  else
  {
    coefficients = { this->_dPreciseKp, this->_dPreciseKi, this->_dPreciseKd };
  }
  return coefficients;
}

void KinematicsControl::vSetPIDsParameters()
{
  PID_terms_coefficients coefficients;

  coefficients = xSelectPIDCoefficients(_pFrontCartPID->dGetError());
  this->_pFrontCartPID->vSetKp(coefficients.dKp);
  this->_pFrontCartPID->vSetKi(coefficients.dKi);
  this->_pFrontCartPID->vSetKd(coefficients.dKd);

  coefficients = xSelectPIDCoefficients(_pFrontCartPID->dGetError());
  this->_pBackCartPID->vSetKp(coefficients.dKp);
  this->_pBackCartPID->vSetKi(coefficients.dKi);
  this->_pBackCartPID->vSetKd(coefficients.dKd);

  this->_pFrontCartPID->vSetSPAcceleration(SP_RAMP_SUBSTEP * DEG_TO_RAD, SP_RAMP_SUBSTEPS_NUMBER);
  this->_pBackCartPID->vSetSPAcceleration(SP_RAMP_SUBSTEP * DEG_TO_RAD, SP_RAMP_SUBSTEPS_NUMBER);
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
      if (angular_vel * _dChassisLength / 2 / linear_vel > 1)
      {
        // Desired radius too small
        SP = { M_PI / 2, -M_PI / 2 };
        ROS_WARN_NAMED("agribot_navigation", "Radius = %f, lower than half of chasis length = %f",
                       linear_vel / angular_vel, _dChassisLength / 2);
      }
      else
      {
        if (angular_vel * _dChassisLength / 2 / linear_vel < -1)
        {
          // Desired radius too small
          SP = { -M_PI / 2, M_PI / 2 };
          ROS_WARN_NAMED("agribot_navigation", "Radius = %f, lower than half of chasis length = %f",
                         linear_vel / angular_vel, _dChassisLength / 2);
        }
        else
        {
          // Desired radius within proper range
          if (linear_vel > 0)
          {
            // Moving forward - back cart is pulled
            SP = getSetPoints(DrivingDirection::forward, dVelocitiesToRadius(linear_vel, angular_vel));
          }
          else
          {
            SP = getSetPoints(DrivingDirection::backward, dVelocitiesToRadius(linear_vel, angular_vel));
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
      return { dRadiusToAngle(radius), -_pdCartAngle[FRONT_CART] };
    }
    else if (steering_mode_ == SteeringMode::front_wheel)
    {
      return { dRadiusToAngle(radius), 0 };
    }
    else if (steering_mode_ == SteeringMode::crab)
    {
      return { dRadiusToAngle(radius), dRadiusToAngle(radius) };
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
      return { -_pdCartAngle[BACK_CART], -dRadiusToAngle(radius) };
    }
    else if (steering_mode_ == SteeringMode::front_wheel)
    {
      return { 0, -dRadiusToAngle(radius) };
    }
    else if (steering_mode_ == SteeringMode::crab)
    {
      return { -dRadiusToAngle(radius), -dRadiusToAngle(radius) };
    }
    else
    {
      return { 0, 0 };
    }
  }
}

double KinematicsControl::dCalculateLinearControls(double linear_vel, double angular_vel)
{
  if (angular_vel != 0)
  {
    // Angular - yes
    if (linear_vel != 0)
    {
      // Angular - yes / Linear -yes
      _pdControl[0] = linear_vel;
    }
    else
    {
      // Angular - yes / Linear -no
      _pdControl[0] = angular_vel * _dChassisLength / 2;
    }
  }
  else
  {
    if (linear_vel != 0)
    {
      // Angular - no / Linear -yes
      _pdControl[0] = linear_vel;
    }
    else
    {
      // Angular - no / Linear -no
      _pdControl[0] = 0.0;
    }
  }
  return _pdControl[0];
}

double KinematicsControl::dCalculateFrontAngularControl(double linear_vel, double angular_vel)
{
  vUpdatePIDs(ros::Time::now().toSec());
  if (linear_vel == 0 && angular_vel == 0)
  {
    // Stopped
    return 0;
  }
  else
  {
    // Moving
    return this->_pFrontCartPID->dGetControlValue();
  }
}

double KinematicsControl::dCalculateBackAngularControl(double linear_vel, double angular_vel)
{
  vUpdatePIDs(ros::Time::now().toSec());
  if (linear_vel == 0 && angular_vel == 0)
  {
    // Stopped
    return 0;
  }
  else
  {
    // Moving
    return this->_pBackCartPID->dGetControlValue();
  }
}

void KinematicsControl::vSetPIDsSetpoints()
{
  Carts_Setpoints CartsSetPoints;
  CartsSetPoints = this->xCalculatePIDSetPoints(_pdInputLinear[X], _pdInputAngular[Z]);
  this->_pFrontCartPID->vSetSP(CartsSetPoints.dFrontCartSP);
  this->_pBackCartPID->vSetSP(CartsSetPoints.dBackCartSP);

  if (_pdInputLinear[X] == 0 && _pdInputAngular[Z] == 0)
  {
    this->_pFrontCartPID->vResetError();
    this->_pBackCartPID->vResetError();
    this->_pFrontCartPID->vResetSetPoints(_pdCartAngle[FRONT_CART]);
    this->_pBackCartPID->vResetSetPoints(_pdCartAngle[BACK_CART]);
  }
}

double KinematicsControl::dCalculateAngularMultiplier(double linear_multiplier)
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

double KinematicsControl::dVelocitiesToRadius(double linear, double angular)
{
  return linear / angular;
}

double KinematicsControl::dRadiusToAngle(const double Radius) const
{
  double angle{ 0 };
  if (steering_mode_ == SteeringMode::front_wheel)
  {
    angle = atan(_dChassisLength / Radius);
  }
  else
  {
    angle = asin(_dChassisLength / 2.0 / Radius);
  }
  return angle;
}

double KinematicsControl::dMultiplierLinear(double x)
{
  return 1 - x;
}

double KinematicsControl::dMultiplierGaussian(double x)
{
  return 0.04379209 + (0.97703891) / (1 + pow((x / 0.1482077), 2.181746));
}

double KinematicsControl::dMultiplierSigmoid(double x)
{
  return 1.092919 - (5.091961 / 4.758651) * (1 - exp(-4.758651 * x));
}

double KinematicsControl::dMultiplierExponential(double x)
{
  return 1.0875 * exp(-4.52068 * x);
}

double KinematicsControl::dMultiplierRoot2(double x)
{
  return 1 - (pow(x, (1 / 2.0)));
}

double KinematicsControl::dMultiplierRoot3(double x)
{
  return 1 - (pow(x, (1 / 3.0)));
}

double KinematicsControl::dMultiplierRoot4(double x)
{
  return 1 - (pow(x, (1 / 4.0)));
}

double KinematicsControl::dFrontCartAngleDifference()
{
  return abs(_pdCartAngle[FRONT_CART] - _pFrontCartPID->dGetDesiredSetPoint());
}

double KinematicsControl::dBackCartAngleDifference()
{
  return abs(_pdCartAngle[BACK_CART] - _pBackCartPID->dGetDesiredSetPoint());
}

double KinematicsControl::dMaximumCartAngleDifference(double diff_front, double diff_back)
{
  return std::max(diff_front, diff_back);
}

void KinematicsControl::vCalculateKinematicsControl()
{
  this->vSetPIDsParameters();
  this->vSetPIDsSetpoints();

  _pdControl[0] = this->dCalculateLinearControls(_pdInputLinear[X], _pdInputAngular[Z]);
  _pdControl[1] = this->dCalculateFrontAngularControl(_pdInputLinear[X], _pdInputAngular[Z]);
  _pdControl[2] = this->dCalculateBackAngularControl(_pdInputLinear[X], _pdInputAngular[Z]);

  this->_dLinearMultiplier = this->dMultiplierGaussian(
      this->dMaximumCartAngleDifference(this->dFrontCartAngleDifference(), this->dBackCartAngleDifference()) /
      ANGLE_RANGE);
  this->_dAngularMultiplier = this->dCalculateAngularMultiplier(this->_dLinearMultiplier);

  _pdCaterpillarVelocities[FRONT_LINEAR] = this->_dLinearMultiplier * _pdControl[0];
  _pdCaterpillarVelocities[FRONT_ANGULAR] = this->_dAngularMultiplier * _pdControl[1];
  _pdCaterpillarVelocities[BACK_LINEAR] =
      this->_dLinearMultiplier * cos(_pdCartAngle[FRONT_CART]) / cos(_pdCartAngle[BACK_CART]) * _pdControl[0];
  _pdCaterpillarVelocities[BACK_ANGULAR] = this->_dAngularMultiplier * _pdControl[2];

#ifdef DEBUG
  std_msgs::Float64 message_front_SP;
  std_msgs::Float64 message_front_CV;
  std_msgs::Float64 message_front_PV;
  std_msgs::Float64 message_front_ER;
  std_msgs::Float64 message_front_IE;
  std_msgs::Float64 message_front_DE;
  std_msgs::Float64 message_front_DSP;
  message_front_SP.data = _pFrontCartPID->dGetSetPoint() * RAD_TO_DEG;
  message_front_CV.data = _pFrontCartPID->dGetControlValue() * RAD_TO_DEG;
  message_front_PV.data = _pFrontCartPID->dGetProcessValue() * RAD_TO_DEG;
  message_front_ER.data = _pFrontCartPID->dGetProportionalPart() * RAD_TO_DEG;
  message_front_IE.data = _pFrontCartPID->dGetIntegralPart() * RAD_TO_DEG;
  message_front_DE.data = _pFrontCartPID->dGetDerivativePart() * RAD_TO_DEG;
  message_front_DSP.data = _pFrontCartPID->dGetDesiredSetPoint() * RAD_TO_DEG;

  std_msgs::Float64 message_back_SP;
  std_msgs::Float64 message_back_CV;
  std_msgs::Float64 message_back_PV;
  std_msgs::Float64 message_back_ER;
  std_msgs::Float64 message_back_IE;
  std_msgs::Float64 message_back_DE;
  std_msgs::Float64 message_back_DSP;
  message_back_SP.data = _pBackCartPID->dGetSetPoint() * RAD_TO_DEG;
  message_back_CV.data = _pBackCartPID->dGetControlValue() * RAD_TO_DEG;
  message_back_PV.data = _pBackCartPID->dGetProcessValue() * RAD_TO_DEG;
  message_back_ER.data = _pBackCartPID->dGetProportionalPart() * RAD_TO_DEG;
  message_back_IE.data = _pBackCartPID->dGetIntegralPart() * RAD_TO_DEG;
  message_back_DE.data = _pBackCartPID->dGetDerivativePart() * RAD_TO_DEG;
  message_back_DSP.data = _pBackCartPID->dGetDesiredSetPoint() * RAD_TO_DEG;

  std_msgs::Float64 message_linear_FV;
  std_msgs::Float64 message_linear_BV;
  std_msgs::Float64 message_linear_MP;
  std_msgs::Float64 message_linear_SP;
  std_msgs::Float64 message_linear_FD;
  std_msgs::Float64 message_linear_BD;
  message_linear_FV.data = _pdCaterpillarVelocities[FRONT_LINEAR];
  message_linear_BV.data = _pdCaterpillarVelocities[BACK_LINEAR];
  message_linear_MP.data = this->_dLinearMultiplier;
  message_linear_SP.data = _pdControl[0];
  message_linear_FD.data = this->dFrontCartAngleDifference() * RAD_TO_DEG;
  message_linear_BD.data = this->dBackCartAngleDifference() * RAD_TO_DEG;

  _rFrontSetPointPub.publish(message_front_SP);
  _rFrontControlValuePub.publish(message_front_CV);
  _rFrontProcessValuePub.publish(message_front_PV);
  _rFrontErrorPub.publish(message_front_ER);
  _rFrontIntegralPub.publish(message_front_IE);
  _rFrontDerivativePub.publish(message_front_DE);
  _rFrontDesSetPointPub.publish(message_front_DSP);

  _rBackSetPointPub.publish(message_back_SP);
  _rBackControlValuePub.publish(message_back_CV);
  _rBackProcessValuePub.publish(message_back_PV);
  _rBackErrorPub.publish(message_back_ER);
  _rBackIntegralPub.publish(message_back_IE);
  _rBackDerivativePub.publish(message_back_DE);
  _rBackDesSetPointPub.publish(message_back_DSP);

  _rFrontVelPub.publish(message_linear_FV);
  _rBackVelPub.publish(message_linear_BV);
  _rMultipiterPub.publish(message_linear_MP);
  _rSetPointPub.publish(message_linear_SP);
  _rFrontDifPub.publish(message_linear_FD);
  _rBackDifPub.publish(message_linear_BD);
#endif
}

double* KinematicsControl::pdGetCaterpillarVelocities()
{
  return this->_pdCaterpillarVelocities;
}

void KinematicsControl::vSetInputLinear(double x, double y, double z)
{
  this->_pdInputLinear[X] = x;
  this->_pdInputLinear[Y] = y;
  this->_pdInputLinear[Z] = z;
}

void KinematicsControl::vSetInputAngular(double x, double y, double z)
{
  this->_pdInputAngular[X] = x;
  this->_pdInputAngular[Y] = y;
  this->_pdInputAngular[Z] = z;
}

void KinematicsControl::vSetChassisAngle(double theta)
{
  this->_dChassisAngle = theta;
}

void KinematicsControl::vSetFrontCartAngle(double theta)
{
  this->_pdCartAngle[FRONT_CART] = theta;
}

void KinematicsControl::vSetBackCartAngle(double theta)
{
  this->_pdCartAngle[BACK_CART] = theta;
}

void KinematicsControl::vUpdatePIDs(double t)
{
  double dt = t - this->_dLastTime;
  this->_pFrontCartPID->Update(_pdCartAngle[FRONT_CART], dt);
  this->_pBackCartPID->Update(_pdCartAngle[BACK_CART], dt);
  this->_dLastTime = t;
}

void KinematicsControl::vSetPreciseKp(double Kp)
{
  this->_dPreciseKp = Kp;
}

void KinematicsControl::vSetPreciseKi(double Ki)
{
  this->_dPreciseKi = Ki;
}

void KinematicsControl::vSetPreciseKd(double Kd)
{
  this->_dPreciseKd = Kd;
}

void KinematicsControl::vSetCoarseKp(double Kp)
{
  this->_dCoarseKp = Kp;
}

void KinematicsControl::vSetCoarseKi(double Ki)
{
  this->_dCoarseKi = Ki;
}

void KinematicsControl::vSetCoarseKd(double Kd)
{
  this->_dCoarseKd = Kd;
}

void KinematicsControl::vSetFrontLimit(double limit)
{
  this->_dFrontLimit = limit * DEG_TO_RAD;
  this->_pFrontCartPID->vSetLowLimit(-this->_dFrontLimit);
  this->_pFrontCartPID->vSetHighLimit(this->_dFrontLimit);
}

void KinematicsControl::vSetBackLimit(double limit)
{
  this->_dBackLimit = limit * DEG_TO_RAD;
  this->_pBackCartPID->vSetLowLimit(-this->_dBackLimit);
  this->_pBackCartPID->vSetHighLimit(this->_dBackLimit);
}

void KinematicsControl::vSetPrecisionRange(double range)
{
  this->_dPrecisionRange = range;
}

void KinematicsControl::setSteeringMode(const SteeringMode& mode)
{
  steering_mode_ = mode;
}
