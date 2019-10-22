# pragma once

#include <agribot_navigation/AgribotComponent.h>

class InverseKinematics: public AgribotComponent
{
private:
    float _pfInputLinear [3];
    float _pfInputAngular [3];
    float _pfCaterpillarVelocities  [4];
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