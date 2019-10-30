#include <agribot_navigation/InverseKinematics.h>
#include <math.h>

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
    //n1=sqrt(x^2+y^2)
    //n2=YAW - (sin(1-2)/cos(2)*n1/2l)
    //n3=(sin(1-2)/cos(2)*n1/2l) - YAW
    //v_r_p = ((2 * v_p) + (w_p * L)) / (2 * R) = (2*v+w*L/2)/R
    //v_l_p = ((2 * v_p) - (w_p * L)) / (2 * R) = (2*v-w*L/2)/R
    //v_r_p = ((2 * v_t) + (w_t * L)) / (2 * R) = (2*v+w*L/2)/R
    //v_l_p = ((2 * v_t) - (w_t * L)) / (2 * R) = (2*v-w*L/2)/R
    //v_t=sin(theta_1)/sin(theta_2)*v_p

    _pfControl[0] = sqrt(_pfInputLinear[X] * _pfInputLinear[X] + _pfInputLinear[Y] * _pfInputLinear[Y]);
    _pfControl[1] = _pfInputAngular[Z] - sin(_pfCartAngle[FRONT_CART] - _pfCartAngle[BACK_CART]) * _pfControl[0] / chasis_length_ / cos(_pfCartAngle[BACK_CART]);
    _pfControl[2] = -_pfControl[1];
    _pfCaterpillarVelocities[FRONT_LINEAR] = _pfControl[0];
    _pfCaterpillarVelocities[FRONT_ANGULAR] = (_pfControl[1] + sin(_pfCartAngle[FRONT_CART] - _pfCartAngle[BACK_CART]) / cos(_pfCartAngle[BACK_CART]) * _pfControl[0] / chasis_length_);
    _pfCaterpillarVelocities[BACK_LINEAR] = cos(_pfCartAngle[FRONT_CART]) / cos(_pfCartAngle[BACK_CART]) * _pfControl[0];
    _pfCaterpillarVelocities[BACK_ANGULAR] = -1 * (_pfControl[2] + sin(_pfCartAngle[FRONT_CART] - _pfCartAngle[BACK_CART]) / cos(_pfCartAngle[BACK_CART]) * _pfControl[0] / chasis_length_);
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
void InverseKinematics::vSetFrontCartAngle(double theta)
{
    this->_pfCartAngle[FRONT_CART] = theta;
}
void InverseKinematics::vSetBackCartAngle(double theta)
{
    this->_pfCartAngle[BACK_CART] = theta;
}