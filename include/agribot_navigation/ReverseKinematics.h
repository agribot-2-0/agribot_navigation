# pragma once

#include <agribot_navigation/AgribotComponent.h>

class ReverseKinematics: public AgribotComponent
{
private:
    float[3] _pfInputLinear;
    float[3] _pfInputAngular;
    float[4] _pfCaterpillarVelocities;
public:
    ReverseKinematics();
    
    virtual int configure();
    virtual bool startup();
    virtual bool update();
    virtual bool stop();
    virtual bool cleanup();

    void vCalculateReverseKinematics();
    void vGetCaterpilareVelocities(float* output);
    void vSetInputLinear(float x, float y, float z);
    void vSetInputAngular(float x, float y, float z);   
};