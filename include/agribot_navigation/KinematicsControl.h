# pragma once
#include <agribot_common/AgribotComponent.h>
#include <agribot_navigation/PID.h>

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
    PID *_pFrontCartPID;
    PID *_pBackCartPID;
public: 
    KinematicsControl();
    virtual int configure();
    virtual bool startup();
    virtual bool update();
    virtual bool stop();
    virtual bool cleanup();
    void vCalculateKinematicsControl();
    double* pdGetCaterpillarVelocities();
    void vSetInputLinear(double x, double y, double z);
    void vSetInputAngular(double x, double y, double z);   
    void vSetChassisAngle(double theta);
    void vSetFrontCartAngle(double theta);
    void vSetBackCartAngle(double theta);
    void vRegulatePIDs();
    void vSetPIDSetPoints();
    void vCalculateAngularVelocities();
    void vUpdatePIDs(double t);
    double angle_diff_front();
    double angle_diff_back();
    double diff(double diff_front, double diff_back);
    double dVelocitiesToRadius(double linear, double angular);
    double dRadiusToAngle(double Radius);
    double dMultiplierGaussian(double x);
    double dMultiplierSigmoid(double x);
    double dMultiplierRoot2(double x);
    double dMultiplierRoot3(double x);
    double dMultiplierRoot4(double x);
    double dMultiplierLinear(double x);
    double dMultiplierExponential(double x);
};
