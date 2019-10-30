# pragma once

#include <agribot_navigation/AgribotComponent.h>

class InverseKinematics: public AgribotComponent
{
private:
    double _pfInputLinear [3];
    double _pfInputAngular [3];
    double _pfCaterpillarVelocities  [4];
    double _pfControl[3];
    double _pfCartAngle [2];
    double wheel_separation_;
    double wheel_radius_;
    double chasis_length_;
public: 
    InverseKinematics();

    virtual int configure();
    virtual bool startup();
    virtual bool update();
    virtual bool stop();
    virtual bool cleanup();

    void vCalculateInverseKinematics();
    void vGetCaterpilareVelocities(double output[]);
    void vSetInputLinear(double x, double y, double z);
    void vSetInputAngular(double x, double y, double z);   
    void vSetFrontCartAngle(double theta);
    void vSetBackCartAngle(double theta);
};