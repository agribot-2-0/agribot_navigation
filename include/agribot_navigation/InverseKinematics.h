# pragma once

#include <agribot_navigation/AgribotComponent.h>

class InverseKinematics: public AgribotComponent
{
private:
    float _pfInputLinear [3];
    float _pfInputAngular [3];
    float _pfCaterpillarVelocities  [4];
    float _pfControl[3];
    float _pfCartAngle [2];
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
    void vGetCaterpilareVelocities(float output[]);
    void vSetInputLinear(float x, float y, float z);
    void vSetInputAngular(float x, float y, float z);   
};