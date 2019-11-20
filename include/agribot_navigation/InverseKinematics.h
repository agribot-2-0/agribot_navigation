# pragma once
#include <agribot_common/AgribotComponent.h>
#include <agribot_navigation/PID.h>

class InverseKinematics: public AgribotComponent
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
    InverseKinematics();
    virtual int configure();
    virtual bool startup();
    virtual bool update();
    virtual bool stop();
    virtual bool cleanup();
    void vCalculateInverseKinematics();
    double* pdGetCaterpillarVelocities();
    void vSetInputLinear(double x, double y, double z);
    void vSetInputAngular(double x, double y, double z);   
    void vSetChassisAngle(double theta);
    void vSetFrontCartAngle(double theta);
    void vSetBackCartAngle(double theta);
    void vUpdatePIDs(double t);
};
