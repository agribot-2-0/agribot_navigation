#include <agribot_navigation/InverseKinematics.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>

#define HISTERESIS 0.05

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
    this-> _fChassisAngle=0;
    this-> wheel_radius_ = 0.16;
    this-> wheel_separation_ = 1.05 + 0.025;
    this-> chasis_length_ = 2.42 - 2 * wheel_radius_;
}
int InverseKinematics::configure() {}
bool InverseKinematics::startup() {}
bool InverseKinematics::update()
{
    this->vCalculateInverseKinematics();
}
bool InverseKinematics::stop() {}
bool InverseKinematics::cleanup() {}
void InverseKinematics::vCalculateInverseKinematics()
{
    //n1 = cos(theta0)*vx+sin(theta0)*vy
    //n2=YAW - (sin(1-2)/cos(2)*n1/2l)
    //n3=(sin(1-2)/cos(2)*n1/2l) - YAW
    //v_r_p = ((2 * v_p) + (w_p * L)) / (2 * R) = (2*v+w*L/2)/R
    //v_l_p = ((2 * v_p) - (w_p * L)) / (2 * R) = (2*v-w*L/2)/R
    //v_r_p = ((2 * v_t) + (w_t * L)) / (2 * R) = (2*v+w*L/2)/R
    //v_l_p = ((2 * v_t) - (w_t * L)) / (2 * R) = (2*v-w*L/2)/R
    //v_t=sin(theta_1)/sin(theta_2)*v_p
    // ROS_ERROR_NAMED("agribot_drive", "LINEAR  %f %f %f", _pfInputLinear[X], _pfInputLinear[Y], _pfInputLinear[Z]);
    // ROS_ERROR_NAMED("agribot_drive", "ANGULAR %f %f %f", _pfInputAngular[X], _pfInputAngular[Y], _pfInputAngular[Z]);
    // ROS_ERROR_NAMED("agribot_drive", "COND    %f %f %f", chasis_length_ * _pfInputAngular[Z] / 0.5 , asin(chasis_length_ * _pfInputAngular[Z] / 0.5) ,  _pfCartAngle[FRONT_CART]);

    if (_pfInputAngular[Z]!=0 )
        if (abs(asin(chasis_length_ * _pfInputAngular[Z] / 0.5) - _pfCartAngle[FRONT_CART]) > HISTERESIS)
        {
            _pfControl[0] = 0;
            if(_pfInputAngular[Z])
                _pfControl[1] = HISTERESIS;
            else
                _pfControl[1] = HISTERESIS;
            
            _pfControl[2] = -_pfControl[1];
        }
        else
        {
            _pfControl[0] = _pfInputLinear[X];
            _pfControl[1] = 0;
            _pfControl[2] = 0;
        }
    else
        if(_pfInputLinear[X]!=0)
            if (  abs( _pfCartAngle[FRONT_CART]) > HISTERESIS)
            {
                _pfControl[0] = 0;
                if( _pfCartAngle[FRONT_CART] < 0)
                    _pfControl[1] = HISTERESIS;
                else
                    _pfControl[1] = HISTERESIS;
            
                _pfControl[2] = -_pfControl[1];
            }
            else
            {
                _pfControl[0] = _pfInputLinear[X];
                _pfControl[1] = 0;
                _pfControl[2] = 0;
            }
        else
        {
            _pfControl[0] = 0;
            _pfControl[1] = 0;
            _pfControl[2] = 0;
        }
            



    _pfCaterpillarVelocities[FRONT_LINEAR] = _pfControl[0];
    _pfCaterpillarVelocities[FRONT_ANGULAR] = _pfControl[1];
    _pfCaterpillarVelocities[BACK_LINEAR] = cos(_pfCartAngle[FRONT_CART]) / cos(_pfCartAngle[BACK_CART]) * _pfControl[0];
    _pfCaterpillarVelocities[BACK_ANGULAR] = _pfControl[2];
}
double* InverseKinematics::vGetCaterpilareVelocities()
{
    return this->_pfCaterpillarVelocities;
    // for (int i = 0; i++; i < 4)
    // {
    //     output[i] = this->_pfCaterpillarVelocities[i];
    //     printf("%f %f %f %f \n", output[0], output[1], output[2], output[3]);
    // }
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