/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <agribot_common/AgribotComponent.h>
#include <agribot_navigation/kinematics_control/PID.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

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
  FRONT_CART,
  BACK_CART
};

struct PID_terms_coefficients
{
  double dKp;
  double dKi;
  double dKd;
};

struct Carts_Setpoints
{
  double dFrontCartSP;
  double dBackCartSP;
};

class KinematicsControl : public AgribotComponent
{
private:
  ros::Publisher _rFrontSetPointPub;
  ros::Publisher _rFrontControlValuePub;
  ros::Publisher _rFrontProcessValuePub;
  ros::Publisher _rFrontErrorPub;
  ros::Publisher _rFrontIntegralPub;
  ros::Publisher _rFrontDerivativePub;
  ros::Publisher _rFrontDesSetPointPub;

  ros::Publisher _rBackSetPointPub;
  ros::Publisher _rBackControlValuePub;
  ros::Publisher _rBackProcessValuePub;
  ros::Publisher _rBackErrorPub;
  ros::Publisher _rBackIntegralPub;
  ros::Publisher _rBackDerivativePub;
  ros::Publisher _rBackDesSetPointPub;

  ros::Publisher _rFrontVelPub;
  ros::Publisher _rBackVelPub;
  ros::Publisher _rMultipiterPub;
  ros::Publisher _rSetPointPub;
  ros::Publisher _rFrontDifPub;
  ros::Publisher _rBackDifPub;

  double _pdInputLinear[3];
  double _pdInputAngular[3];
  double _pdCaterpillarVelocities[4];
  double _pdControl[3];
  double _pdCartAngle[2];
  double _dChassisAngle;
  double _dWheelSeparation;
  double _dWheelRadius;
  double _dChassisLength;
  double _dLastTime;
  double _dLinearMultiplier;
  double _dAngularMultiplier;
  double _dCoarseKp;
  double _dCoarseKi;
  double _dCoarseKd;
  double _dPreciseKp;
  double _dPreciseKd;
  double _dPreciseKi;
  double _dPrecisionRange;
  double _dFrontLimit;
  double _dBackLimit;
  PID* _pFrontCartPID;
  PID* _pBackCartPID;
  void vSetPIDsParameters();
  double dCalculateLinearControls(double linear_vel, double angular_vel);
  double dCalculateFrontAngularControl(double linear_vel, double angular_vel);
  double dCalculateBackAngularControl(double linear_vel, double angular_vel);
  double dCalculateAngularMultiplier(double linear_multiplier);
  void vUpdatePIDs(double t);
  void vSetPIDsSetpoints();
  void vCalculateKinematicsControl();
  double dFrontCartAngleDifference();
  double dBackCartAngleDifference();
  double dMaximumCartAngleDifference(double diff_front, double diff_back);
  double dVelocitiesToRadius(double linear, double angular);
  double dRadiusToAngle(double Radius);
  double dMultiplierGaussian(double x);
  double dMultiplierSigmoid(double x);
  double dMultiplierRoot2(double x);
  double dMultiplierRoot3(double x);
  double dMultiplierRoot4(double x);
  double dMultiplierLinear(double x);
  double dMultiplierExponential(double x);
  Carts_Setpoints xCalculatePIDSetPoints(double linear_vel, double angular_vel);
  PID_terms_coefficients xSelectPIDCoefficients(double Error);

public:
  KinematicsControl();
  virtual int configure();
  virtual bool startup();
  virtual bool update();
  virtual bool stop();
  virtual bool cleanup();
  void vCreatePubs(ros::NodeHandle n);
  void vSetInputLinear(double x, double y, double z);
  void vSetInputAngular(double x, double y, double z);
  void vSetChassisAngle(double theta);
  void vSetFrontCartAngle(double theta);
  void vSetBackCartAngle(double theta);
  double* pdGetCaterpillarVelocities();
  void vSetPreciseKp(double Kp);
  void vSetPreciseKi(double Ki);
  void vSetPreciseKd(double Kd);
  void vSetCoarseKp(double Kp);
  void vSetCoarseKi(double Ki);
  void vSetCoarseKd(double Kd);
  void vSetFrontLimit(double limit);
  void vSetBackLimit(double limit);
  void vSetPrecisionRange(double range);
};
