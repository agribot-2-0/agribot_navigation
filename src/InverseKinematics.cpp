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
    this->_pdCartAngle[FRONT_CART] = 0;
    this->_pdCartAngle[BACK_CART] = 0;
    this->_dChassisAngle = 0;
    this->_dWheelRadius = 0.16;
    this->_dWheelSeparation = 1.05 + 0.025;
    this->_dChassisLength = 2.42 - 2 * _dWheelRadius;
    //  TO DO
    //  SET UP PID GAINS
    //                              KP      KD      KI      LL      HL
    this->_Front_Cart_PID = new PID(0.002, 0.03, 0.00001, -PI / 4, PI / 4); 
    this->_Back_Cart_PID = new PID(0.002, 0.03, 0.00001, -PI / 4, PI / 4);
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
    _pdControl[0] = _pdInputLinear[X];
    if (_pdInputAngular[Z] != 0)
    {
        if (_pdInputLinear[X] != 0)
        {
            if (_pdInputAngular[Z] * _dChassisLength / 2 / _pdInputLinear[X] > 1)
            {
                this->_pFrontCartPID->vSetSP(PI / 2);
                ROS_ERROR_NAMED("agribot_navigation","Radius = %f, lower than half of chasis length = %f", _pdInputLinear[X]/_pdInputAngular[Z], _dChassisLength / 2);
            }
            else 
            {
                if (_pdInputAngular[Z] * _dChassisLength / 2 / _pdInputLinear[X] < -1)
                {
                    this->_pFrontCartPID->vSetSP(-PI / 2);
                    ROS_ERROR_NAMED("agribot_navigation","Radius = %f, lower than half of chasis length = %f", _pdInputLinear[X]/_pdInputAngular[Z], _dChassisLength / 2);
                }
                else
                {

                    // this->_Front_Cart_PID->vSetSP(asin(_pfInputAngular[Z] * chasis_length_ / 2 / _pfInputLinear[X]));
                    if (_pfInputLinear[X] > 0)
                    {
                        this->_Front_Cart_PID->vSetSP(asin(_pfInputAngular[Z] * chasis_length_ / 2 / _pfInputLinear[X]));
                    }
                    else
                    {
                        this->_Back_Cart_PID->vSetSP(-asin(_pfInputAngular[Z] * chasis_length_ / 2 / _pfInputLinear[X]));
                    }
                }
            }
            // this->_Back_Cart_PID->vSetSP(-_pfCartAngle[FRONT_CART]);
            if (_pfInputLinear[X] > 0)
            {
                this->_Back_Cart_PID->vSetSP(-_pfCartAngle[FRONT_CART]);
            }
            else
            {
                this->_Front_Cart_PID->vSetSP(-_pfCartAngle[BACK_CART]);
            }
        }
        else
        {
            _pdControl[0] = _pdInputAngular[Z] * _dChassisLength / 2;
            this->_pFrontCartPID->vSetSP(PI / 2);
            this->_pBackCartPID->vSetSP(-PI / 2);
        }
    }    
    else
    {    
        if (_pdInputLinear[X] != 0)
        {
            this->_pFrontCartPID->vSetSP(0);
            this->_pBackCartPID->vSetSP(0);
        }
        else
        {
            this->_pFrontCartPID->vResetError();
            this->_pBackCartPID->vResetError();
            this->_pFrontCartPID->vSetSP(_pdCartAngle[FRONT_CART]);
            this->_pBackCartPID->vSetSP(_pdCartAngle[BACK_CART]);
        }
    }
    vUpdatePIDs(ros::Time::now().toSec());
    if (_pdInputLinear[X] == 0 && _pdInputAngular[Z] == 0)
    {
        _pdControl[1] = 0;
        _pdControl[2] = 0;
    }
    else
    {
        _pdControl[1] = this->_pFrontCartPID->dGetCV();
        _pdControl[2] = this->_pBackCartPID->dGetCV();
    }
    _pdCaterpillarVelocities[FRONT_LINEAR] = _pdControl[0];
    _pdCaterpillarVelocities[FRONT_ANGULAR] = _pdControl[1];
    _pdCaterpillarVelocities[BACK_LINEAR] = cos(_pdCartAngle[FRONT_CART]) / cos(_pdCartAngle[BACK_CART]) * _pdControl[0];
    _pdCaterpillarVelocities[BACK_ANGULAR] = _pdControl[2];
}
double *InverseKinematics::pdGetCaterpillarVelocities()
{
    return this->_pdCaterpillarVelocities;
}
void InverseKinematics::vSetInputLinear(double x, double y, double z)
{
    this->_pdInputLinear[X] = x;
    this->_pdInputLinear[Y] = y;
    this->_pdInputLinear[Z] = z;
}
void InverseKinematics::vSetInputAngular(double x, double y, double z)
{
    this->_pdInputAngular[X] = x;
    this->_pdInputAngular[Y] = y;
    this->_pdInputAngular[Z] = z;
}
void InverseKinematics::vSetChassisAngle(double theta)
{
    this->_dChassisAngle = theta;
}
void InverseKinematics::vSetFrontCartAngle(double theta)
{
    this->_pdCartAngle[FRONT_CART] = theta;
}
void InverseKinematics::vSetBackCartAngle(double theta)
{
    this->_pdCartAngle[BACK_CART] = theta;
}
void InverseKinematics::vUpdatePIDs(double t)
{
    double dt = t - this->_dLastTime;
    this->_pFrontCartPID->Update(_pdCartAngle[FRONT_CART], dt, true);
    this->_pBackCartPID->Update(_pdCartAngle[BACK_CART], dt, false);
    this->_dLastTime = t;
}