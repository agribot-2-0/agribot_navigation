# pragma once

#include <agribot_navigation/AgribotComponent.h>
#include <agribot_navigation/PID.h>

class InverseKinematics: public AgribotComponent
{
private:
    double _pfInputLinear [3];
    double _pfInputAngular [3];
    double _pfCaterpillarVelocities  [4];
    double _pfControl[3];
    double _pfCartAngle [2];
    double _fChassisAngle;
    double wheel_separation_;
    double wheel_radius_;
    double chasis_length_;
    double _lastTime;
    PID *_Front_Cart_PID;
    PID *_Back_Cart_PID;
public: 
    InverseKinematics();

    virtual int configure();
    virtual bool startup();
    virtual bool update();
    virtual bool stop();
    virtual bool cleanup();

    void vCalculateInverseKinematics();
    double* vGetCaterpilareVelocities();
    void vSetInputLinear(double x, double y, double z);
    void vSetInputAngular(double x, double y, double z);   
    void vSetChassisAngle(double theta);
    void vSetFrontCartAngle(double theta);
    void vSetBackCartAngle(double theta);
    void vUpdatePIDs(double t);
};