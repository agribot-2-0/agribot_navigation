#include <agribot_navigation/InverseKinematics.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>

#define HISTERESIS 0.05
#define PI 3.14159265359

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

InverseKinematics::InverseKinematics()
{
    this->_pfCartAngle[FRONT_CART] = 0;
    this->_pfCartAngle[BACK_CART] = 0;
    this->_fChassisAngle = 0;
    this->wheel_radius_ = 0.16;
    this->wheel_separation_ = 1.05 + 0.025;
    this->chasis_length_ = 2.42 - 2 * wheel_radius_;
    //  TO DO
    //  SET UP PID GAINS
    this->_Front_Cart_PID = new PID(0.05, 0.00001, 0.8, -PI / 4, PI / 4);
    this->_Back_Cart_PID = new PID(0.05, 0.00001, 0.8, -PI / 4, PI / 4);
}
int InverseKinematics::configure() 
{
    return 0;
}
bool InverseKinematics::startup() 
{
    return false;
}
bool InverseKinematics::update()
{
    this->vCalculateInverseKinematics();
    return true;
}
bool InverseKinematics::stop() 
{
    return false;
}
bool InverseKinematics::cleanup() 
{
    return false;
}
void InverseKinematics::vCalculateInverseKinematics()
{
    _pfControl[0] = _pfInputLinear[X];
    if (_pfInputAngular[Z] != 0)
    {
        if (_pfInputLinear[X] != 0)
        {
            if (_pfInputAngular[Z] * chasis_length_ / 2 / _pfInputLinear[X] > 1)
            {
                this->_Front_Cart_PID->vSetSP(PI / 2);
                ROS_ERROR_NAMED("agribot_navigation","Radius = %f, lower than half of chasis length = %f", _pfInputLinear[X]/_pfInputAngular[Z], chasis_length_ / 2);
            }
            else 
            {
                if (_pfInputAngular[Z] * chasis_length_ / 2 / _pfInputLinear[X] < -1)
                {
                    this->_Front_Cart_PID->vSetSP(-PI / 2);
                    ROS_ERROR_NAMED("agribot_navigation","Radius = %f, lower than half of chasis length = %f", _pfInputLinear[X]/_pfInputAngular[Z], chasis_length_ / 2);
                }
                else
                {
                    this->_Front_Cart_PID->vSetSP(asin(_pfInputAngular[Z] * chasis_length_ / 2 / _pfInputLinear[X]));
                }
            }
            this->_Back_Cart_PID->vSetSP(-_pfCartAngle[FRONT_CART]);
        }
        else
        {
            _pfControl[0] = _pfInputAngular[Z] * chasis_length_ / 2;
            this->_Front_Cart_PID->vSetSP(PI / 2);
            this->_Back_Cart_PID->vSetSP(-PI / 2);
        }
    }    
    else
    {    
        if (_pfInputLinear[X] != 0)
        {
            this->_Front_Cart_PID->vSetSP(0);
            this->_Back_Cart_PID->vSetSP(0);
        }
        else
        {
            this->_Front_Cart_PID->vResetError();
            this->_Back_Cart_PID->vResetError();
            this->_Front_Cart_PID->vSetSP(_pfCartAngle[FRONT_CART]);
            this->_Back_Cart_PID->vSetSP(_pfCartAngle[BACK_CART]);
        }
    }
    vUpdatePIDs(ros::Time::now().toSec());
    if (_pfInputLinear[X] == 0 && _pfInputAngular[Z] == 0)
    {
        _pfControl[1] = 0;
        _pfControl[2] = 0;
    }
    else
    {
        _pfControl[1] = this->_Front_Cart_PID->dGetCV();
        _pfControl[2] = this->_Back_Cart_PID->dGetCV();
    }
    _pfCaterpillarVelocities[FRONT_LINEAR] = _pfControl[0];
    _pfCaterpillarVelocities[FRONT_ANGULAR] = _pfControl[1];
    _pfCaterpillarVelocities[BACK_LINEAR] = cos(_pfCartAngle[FRONT_CART]) / cos(_pfCartAngle[BACK_CART]) * _pfControl[0];
    _pfCaterpillarVelocities[BACK_ANGULAR] = _pfControl[2];
}
double *InverseKinematics::vGetCaterpilareVelocities()
{
    return this->_pfCaterpillarVelocities;
}
void InverseKinematics::vSetInputLinear(double x, double y, double z)
{
    this->_pfInputLinear[X] = x;
    this->_pfInputLinear[Y] = y;
    this->_pfInputLinear[Z] = z;
}
void InverseKinematics::vSetInputAngular(double x, double y, double z)
{
    this->_pfInputAngular[X] = x;
    this->_pfInputAngular[Y] = y;
    this->_pfInputAngular[Z] = z;
}
void InverseKinematics::vSetChassisAngle(double theta)
{
    this->_fChassisAngle = theta;
}
void InverseKinematics::vSetFrontCartAngle(double theta)
{
    this->_pfCartAngle[FRONT_CART] = theta;
}
void InverseKinematics::vSetBackCartAngle(double theta)
{
    this->_pfCartAngle[BACK_CART] = theta;
}
void InverseKinematics::vUpdatePIDs(double t)
{
    double dt = t - this->_lastTime;
    this->_Front_Cart_PID->Update(_pfCartAngle[FRONT_CART], dt, true);
    this->_Back_Cart_PID->Update(_pfCartAngle[BACK_CART], dt, false);
    this->_lastTime = t;
}