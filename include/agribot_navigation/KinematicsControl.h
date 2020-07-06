/*
* Copyright (C) 2020 Agribot. All rights reserved.
*/

#pragma once

#include <agribot_common/AgribotComponent.h>
#include <agribot_navigation/PID.h>

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

enum {
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

class KinematicsControl: public AgribotComponent
{
 private:
    double _pdInputLinear [3];
    double _pdInputAngular [3];
    double _pdCaterpillarVelocities  [4];
    double _pdControl[3];
    double _pdCartAngle [2];
    double _dChassisAngle;
    double _dWheelSeparation;
    double _dWheelRadius;
    double _dChassisLength;
    double _dLastTime;
    double _dLinearMultiplier;
    double _dAngularMultiplier;
    PID *_pFrontCartPID;
    PID *_pBackCartPID;
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
    Carts_Setpoints xCalculatePIDSetPoints(double linear_vel,
                                           double angular_vel);
    PID_terms_coefficients xSelectPIDCoefficients(double Error);
 public:
    KinematicsControl();
    virtual int configure();
    virtual bool startup();
    virtual bool update();
    virtual bool stop();
    virtual bool cleanup();
    void vSetInputLinear(double x, double y, double z);
    void vSetInputAngular(double x, double y, double z);
    void vSetChassisAngle(double theta);
    void vSetFrontCartAngle(double theta);
    void vSetBackCartAngle(double theta);
    double* pdGetCaterpillarVelocities();
};
