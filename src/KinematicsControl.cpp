#include <agribot_navigation/KinematicsControl.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#define HISTERESIS 0.05
#define PI 3.14159265359
#define RAD_TO_DEG (180/PI)
#define DEG_TO_RAD (PI/180)
#define ANGLE_RANGE (PI/2)
#define LIMIT (20.0*DEG_TO_RAD)
#define KP_val 0.75
#define KI_val 0.208333
#define KD_val (1.378125/2)
#define KP_PRECISE_MULTIPLIER (2.0/7)
#define KI_PRECISE_MULTIPLIER (8.0/35)
#define KD_PRECISE_MULTIPLIER (40.0/63)
#define PRECISE_PID_RANGE 2.0
#define LINEARITY_RANGE 5.0
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
KinematicsControl::KinematicsControl()
{
    this->_pdCartAngle[FRONT_CART] = 0;
    this->_pdCartAngle[BACK_CART] = 0;
    this->_dChassisAngle = 0;
    this->_dWheelRadius = 0.16;
    this->_dWheelSeparation = 1.05 + 0.025;
    this->_dChassisLength = 2.42 - 2 * _dWheelRadius;
    this->_dLinearMultiplier = 1;
    this->_pFrontCartPID = new PID(KP_val, KD_val, KI_val, -LIMIT, LIMIT); 
    this->_pBackCartPID = new PID(KP_val, KD_val, KI_val, -LIMIT, LIMIT);
}
int KinematicsControl::configure() 
{
    return 0;
}
bool KinematicsControl::startup() 
{
    return false;
}
bool KinematicsControl::update()
{
    this->vCalculateKinematicsControl();
    return true;
}
bool KinematicsControl::stop() 
{
    return false;
}
bool KinematicsControl::cleanup() 
{
    return false;
}
void KinematicsControl::vRegulatePIDs()
{
    //  in case of being close to Set Point change PID settings to prevent overshoot
    if (abs(_pFrontCartPID->dGetError()*RAD_TO_DEG)>PRECISE_PID_RANGE)
    {
        this->_pFrontCartPID->vSetKp(KP_val);
        this->_pFrontCartPID->vSetKi(KI_val);
        this->_pFrontCartPID->vSetKd(KD_val);
    }
    else
    {
        this->_pFrontCartPID->vSetKp(KP_val*KP_PRECISE_MULTIPLIER);
        this->_pFrontCartPID->vSetKi(KI_val*KI_PRECISE_MULTIPLIER);
        this->_pFrontCartPID->vSetKd(KD_val*KD_PRECISE_MULTIPLIER);
    } 
    if (abs(_pBackCartPID->dGetError()*RAD_TO_DEG)>PRECISE_PID_RANGE)
    {
        this->_pBackCartPID->vSetKp(KP_val);
        this->_pBackCartPID->vSetKi(KI_val);
        this->_pBackCartPID->vSetKd(KD_val);
    }
    else
    {
        this->_pBackCartPID->vSetKp(KP_val*KP_PRECISE_MULTIPLIER);
        this->_pBackCartPID->vSetKi(KI_val*KI_PRECISE_MULTIPLIER);
        this->_pBackCartPID->vSetKd(KD_val*KD_PRECISE_MULTIPLIER);
    }
    this->_pFrontCartPID->vSetSPAcceleration(0.15*DEG_TO_RAD,20);
    this->_pBackCartPID->vSetSPAcceleration(0.15*DEG_TO_RAD,20);
}
void KinematicsControl::vSetPIDSetPoints()
{
    // provide PIDs with Set Points, basing on received v and w
    _pdControl[0] = _pdInputLinear[X];
    if (_pdInputAngular[Z] != 0)
    {
        // Angular - yes
        if (_pdInputLinear[X] != 0)
        {
            // Angular - yes / Linear -yes
            if (_pdInputAngular[Z] * _dChassisLength / 2 / _pdInputLinear[X] > 1)
            {
                // Desired radius too small
                this->_pFrontCartPID->vSetSP(PI / 2);
                this->_pBackCartPID->vSetSP(-PI / 2);
                ROS_WARN_NAMED("agribot_navigation","Radius = %f, lower than half of chasis length = %f", _pdInputLinear[X]/_pdInputAngular[Z], _dChassisLength / 2);
            }
            else 
            {
                if (_pdInputAngular[Z] * _dChassisLength / 2 / _pdInputLinear[X] < -1)
                {
                    // Desired radius too small
                    this->_pFrontCartPID->vSetSP(-PI / 2);
                    this->_pBackCartPID->vSetSP(PI / 2);
                    ROS_WARN_NAMED("agribot_navigation","Radius = %f, lower than half of chasis length = %f", _pdInputLinear[X]/_pdInputAngular[Z], _dChassisLength / 2);
                }
                else
                {
                    // Desired radius within proper range
                    if (_pdInputLinear[X] > 0)
                    {
                        // Moving forward - back cart is pulled
                        this->_pFrontCartPID->vSetSP(asin(_pdInputAngular[Z] * _dChassisLength / 2 / _pdInputLinear[X]));
                        this->_pBackCartPID->vSetSP(-_pdCartAngle[FRONT_CART]);
                        this->_pBackCartPID->vSetSPAcceleration(0.15*DEG_TO_RAD,20);
                    }
                    else
                    {
                        // Moving backwards - front cart is pulled
                        this->_pBackCartPID->vSetSP(-asin(_pdInputAngular[Z] * _dChassisLength / 2 / _pdInputLinear[X]));
                        this->_pFrontCartPID->vSetSP(-_pdCartAngle[BACK_CART]);
                        this->_pFrontCartPID->vSetSPAcceleration(0.15*DEG_TO_RAD,20);
                    }
                }
            }
        }
        else
        {
            // Angular - yes / Linear -no
            _pdControl[0] = _pdInputAngular[Z] * _dChassisLength / 2;
            this->_pFrontCartPID->vSetSP(PI / 2);
            this->_pBackCartPID->vSetSP(-PI / 2);
        }
    }    
    else
    {    
        // Angular - no
        if (_pdInputLinear[X] != 0)
        {
            // Angular - no / Linear -yes
            this->_pFrontCartPID->vSetSP(0);
            this->_pBackCartPID->vSetSP(0);
        }
        else
        {
            // Angular - no / Linear -no
            this->_pFrontCartPID->vResetError();
            this->_pBackCartPID->vResetError();
            this->_pFrontCartPID->vSetSP(_pdCartAngle[FRONT_CART]);
            this->_pBackCartPID->vSetSP(_pdCartAngle[BACK_CART]);
        }
    }
}
void KinematicsControl::vCalculateAngularVelocities()
{
    vUpdatePIDs(ros::Time::now().toSec());
    if (_pdInputLinear[X] == 0 && _pdInputAngular[Z] == 0)
    {
        // Stopped
        _pdControl[1] = 0;
        _pdControl[2] = 0;
    }
    else
    {
        // Moving
        _pdControl[1] = this->_pFrontCartPID->dGetCV();
        _pdControl[2] = this->_pBackCartPID->dGetCV();
    }
}
double KinematicsControl::dMultiplierLinear(double x)
{
    return 1-x;
}
double KinematicsControl::dMultiplierGaussian(double x)
{
    return 0.04379209 + (0.97703891)/(1 + pow((x/0.1482077),2.181746));
}
double KinematicsControl::dMultiplierSigmoid(double x)
{
    return 1.092919 - (5.091961/4.758651)*(1 - exp(-4.758651*x));
}
double KinematicsControl::dMultiplierExponential(double x)
{
    return 1.0875*exp(-4.52068*x);
}
double KinematicsControl::dMultiplierRoot2(double x)
{
    return 1-(pow(x,(1/2.0)));
}
double KinematicsControl::dMultiplierRoot3(double x)
{
    return 1-(pow(x,(1/3.0)));
}
double KinematicsControl::dMultiplierRoot4(double x)
{
    return 1-(pow(x,(1/4.0)));
}
void KinematicsControl::vCalculateLimitMultiplier()
{
    double angle_diff_front = abs(_pdCartAngle[FRONT_CART] - _pFrontCartPID->dGetDesiredSP());
    double angle_diff_back = abs(_pdCartAngle[BACK_CART] - _pBackCartPID->dGetDesiredSP());
    this->_dLinearMultiplier=this->dMultiplierRoot4(std::max(angle_diff_front,angle_diff_back)/ANGLE_RANGE);
    this->_dLinearMultiplier=1;
    ROS_INFO_NAMED("kinematics_control", "LINEAR;FV=%f;BV=%f;mp=%f;SP=%f;dF=%f;dB=%f", _pdCaterpillarVelocities[FRONT_LINEAR], _pdCaterpillarVelocities[BACK_LINEAR],this->_dLinearMultiplier,_pdControl[0],angle_diff_front*RAD_TO_DEG,angle_diff_back*RAD_TO_DEG);
}
void KinematicsControl::vCalculateKinematicsControl()
{
    this->vRegulatePIDs();
    this->vSetPIDSetPoints();
    this->vCalculateAngularVelocities();
    this->vCalculateLimitMultiplier();
    _pdCaterpillarVelocities[FRONT_LINEAR] = this->_dLinearMultiplier * _pdControl[0];
    _pdCaterpillarVelocities[FRONT_ANGULAR] = _pdControl[1];
    _pdCaterpillarVelocities[BACK_LINEAR] = this->_dLinearMultiplier * cos(_pdCartAngle[FRONT_CART]) / cos(_pdCartAngle[BACK_CART]) * _pdControl[0];
    _pdCaterpillarVelocities[BACK_ANGULAR] = _pdControl[2];
    }
double *KinematicsControl::pdGetCaterpillarVelocities()
{
    return this->_pdCaterpillarVelocities;
}
void KinematicsControl::vSetInputLinear(double x, double y, double z)
{
    this->_pdInputLinear[X] = x;
    this->_pdInputLinear[Y] = y;
    this->_pdInputLinear[Z] = z;
}
void KinematicsControl::vSetInputAngular(double x, double y, double z)
{
    this->_pdInputAngular[X] = x;
    this->_pdInputAngular[Y] = y;
    this->_pdInputAngular[Z] = z;
}
void KinematicsControl::vSetChassisAngle(double theta)
{
    this->_dChassisAngle = theta;
}
void KinematicsControl::vSetFrontCartAngle(double theta)
{
    this->_pdCartAngle[FRONT_CART] = theta;
}
void KinematicsControl::vSetBackCartAngle(double theta)
{
    this->_pdCartAngle[BACK_CART] = theta;
}
void KinematicsControl::vUpdatePIDs(double t)
{
    double dt = t - this->_dLastTime;
    this->_pFrontCartPID->Update(_pdCartAngle[FRONT_CART], dt, true);
    this->_pBackCartPID->Update(_pdCartAngle[BACK_CART], dt, false);
    this->_dLastTime = t;
}