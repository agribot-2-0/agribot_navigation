#include <agribot_navigation/KinematicsControl.h>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#define DEBUG
#define HISTERESIS 0.05
#define RAD_TO_DEG (180/M_PI)
#define DEG_TO_RAD (M_PI/180)
#define ANGLE_RANGE (M_PI/2)
#define LIMIT (10.0*DEG_TO_RAD)
#define KP_VAL 0.75
#define KI_VAL 0.208333
#define KD_VAL (1.378125/2)
#define KP_PRECISE (KP_VAL*2.0/7)
#define KI_PRECISE (KI_VAL*8.0/35)
#define KD_PRECISE (KD_VAL*40.0/63)
#define PRECISE_PID_RANGE 2.0
#define LINEARITY_RANGE 5.0
#define SP_RAMP_SUBSTEP 0.225
#define SP_RAMP_SUBSTEPS_NUMBER 20
#define ANGULAR_CHANGE_THRESHOLD 0.5
#define ANGULAR_MULTIPLIER 2.0
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
    this->_pFrontCartPID = new PID(KP_VAL, KD_VAL, KI_VAL, -LIMIT, LIMIT); 
    this->_pBackCartPID = new PID(KP_VAL, KD_VAL, KI_VAL, -LIMIT, LIMIT);
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
PID_terms_coefficients KinematicsControl::xSelectPIDCoefficients(double Error)
{
    PID_terms_coefficients coefficients;
    if (abs(Error*RAD_TO_DEG)>PRECISE_PID_RANGE)
    {
        coefficients = {KP_VAL, KI_VAL, KD_VAL};
    }
    else
    {
        coefficients = {KP_PRECISE, KI_PRECISE, KD_PRECISE};
    }
    return coefficients;
}
void KinematicsControl::vSetPIDsParameters()
{
    PID_terms_coefficients coefficients;
    coefficients=xSelectPIDCoefficients(_pFrontCartPID->dGetError());
    this->_pFrontCartPID->vSetKp(coefficients.dKp);
    this->_pFrontCartPID->vSetKi(coefficients.dKi);
    this->_pFrontCartPID->vSetKd(coefficients.dKd);
    coefficients=xSelectPIDCoefficients(_pFrontCartPID->dGetError());
    this->_pBackCartPID->vSetKp(coefficients.dKp);
    this->_pBackCartPID->vSetKi(coefficients.dKi);
    this->_pBackCartPID->vSetKd(coefficients.dKd);
    this->_pFrontCartPID->vSetSPAcceleration(SP_RAMP_SUBSTEP*DEG_TO_RAD,SP_RAMP_SUBSTEPS_NUMBER);
    this->_pBackCartPID->vSetSPAcceleration(SP_RAMP_SUBSTEP*DEG_TO_RAD,SP_RAMP_SUBSTEPS_NUMBER);
}
Carts_Setpoints KinematicsControl::xCalculatePIDSetPoints(double linear_vel, double angular_vel)
{
    Carts_Setpoints SP;
    // provide PIDs with Set Points, basing on received v and w
    if (angular_vel != 0)
    {
        // Angular - yes
        if (linear_vel != 0)
        {
            // Angular - yes / Linear -yes
            if (angular_vel * _dChassisLength / 2 / linear_vel > 1)
            {
                // Desired radius too small
                SP = {M_PI / 2 , -M_PI / 2};
                ROS_WARN_NAMED("agribot_navigation","Radius = %f, lower than half of chasis length = %f", linear_vel/angular_vel, _dChassisLength / 2);
            }
            else 
            {
                if (angular_vel * _dChassisLength / 2 / linear_vel < -1)
                {
                    // Desired radius too small
                    SP = {-M_PI / 2 ,M_PI / 2};
                    ROS_WARN_NAMED("agribot_navigation","Radius = %f, lower than half of chasis length = %f", linear_vel/angular_vel, _dChassisLength / 2);
                }
                else
                {
                    // Desired radius within proper range
                    if (linear_vel > 0)
                    {
                        // Moving forward - back cart is pulled
                       SP = {this->dRadiusToAngle(this->dVelocitiesToRadius(linear_vel,angular_vel)) , -_pdCartAngle[FRONT_CART]};
                    }
                    else
                    {
                        // Moving backwards - front cart is pulled
                        SP = {-this->dRadiusToAngle(this->dVelocitiesToRadius(linear_vel,angular_vel)) , -_pdCartAngle[BACK_CART]};
                    }
                }
            }
        }
        else
        {
            // Angular - yes / Linear -no
            SP = {M_PI / 2 , -M_PI / 2};
        }
    }    
    else
    {    
        // Angular - no
        if (linear_vel != 0)
        {
            // Angular - no / Linear -yes
            SP = {0 , 0};
        }
        else
        {
            // Angular - no / Linear -no
            SP = {0 , 0};
        }
    }
    return SP;
}
double KinematicsControl::dCalculateLinearControls(double linear_vel, double angular_vel)
{
    if (angular_vel != 0)
    {
        // Angular - yes
        if (linear_vel != 0)
        {
            // Angular - yes / Linear -yes
            _pdControl[0] = linear_vel;
        }
        else
        {
            // Angular - yes / Linear -no
            _pdControl[0] = angular_vel * _dChassisLength / 2;
        }
    }
    else
    {
        if (linear_vel != 0)
        {
            // Angular - no / Linear -yes
            _pdControl[0] = linear_vel;
        }
        else
        {
            // Angular - no / Linear -no
            _pdControl[0] = 0.0;
        }
    }
}
double KinematicsControl::dCalculateFrontAngularControl(double linear_vel, double angular_vel)
{
    vUpdatePIDs(ros::Time::now().toSec());
    if (linear_vel == 0 && angular_vel == 0)
    {
        // Stopped
        return 0;
    }
    else
    {
        // Moving
        return this->_pFrontCartPID->dGetControlValue();
    }
}
double KinematicsControl::dCalculateBackAngularControl(double linear_vel, double angular_vel)
{
    vUpdatePIDs(ros::Time::now().toSec());
    if (linear_vel == 0 && angular_vel == 0)
    {
        // Stopped
       return 0;
    }
    else
    {
        // Moving
        return this->_pBackCartPID->dGetControlValue();
    }
}
void KinematicsControl::vSetPIDsSetpoints()
{
    Carts_Setpoints CartsSetPoints;
    CartsSetPoints = this->xCalculatePIDSetPoints(_pdInputLinear[X], _pdInputAngular[Z]);
    this->_pFrontCartPID->vSetSP(CartsSetPoints.dFrontCartSP);
    this->_pBackCartPID->vSetSP(CartsSetPoints.dBackCartSP);
    if (_pdInputLinear[X] == 0 && _pdInputAngular[Z] == 0)
    {
        this->_pFrontCartPID->vResetError();
        this->_pBackCartPID->vResetError();
        this->_pFrontCartPID->vResetSetPoints(_pdCartAngle[FRONT_CART]);
        this->_pBackCartPID->vResetSetPoints(_pdCartAngle[BACK_CART]);
    }
}
double KinematicsControl::dCalculateAngularMultiplier(double linear_multiplier)
{
    if (linear_multiplier < ANGULAR_CHANGE_THRESHOLD)
    {
        return ANGULAR_MULTIPLIER;
    }
    else
    {
        return 1.0;
    }
}
double KinematicsControl::dVelocitiesToRadius(double linear, double angular)
{
    return linear/angular;
}
double KinematicsControl::dRadiusToAngle(double Radius)
{
    return asin(_dChassisLength / 2 / Radius);
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
double KinematicsControl::dFrontCartAngleDifference()
{
    return abs(_pdCartAngle[FRONT_CART] - _pFrontCartPID->dGetDesiredSetPoint());
}
double KinematicsControl::dBackCartAngleDifference()
{
    return abs(_pdCartAngle[BACK_CART] - _pBackCartPID->dGetDesiredSetPoint());
}
double KinematicsControl::dMaximumCartAngleDifference(double diff_front, double diff_back)
{
    return std::max(diff_front,diff_back);
}  
void KinematicsControl::vCalculateKinematicsControl()
{
    this->vSetPIDsParameters();
    this->vSetPIDsSetpoints();
    _pdControl[0] = this->dCalculateLinearControls(_pdInputLinear[X], _pdInputAngular[Z]);
    _pdControl[1] = this->dCalculateFrontAngularControl(_pdInputLinear[X], _pdInputAngular[Z]);
    _pdControl[2] = this->dCalculateBackAngularControl(_pdInputLinear[X], _pdInputAngular[Z]);
    this->_dLinearMultiplier = this->dMultiplierGaussian(this->dMaximumCartAngleDifference(this->dFrontCartAngleDifference(),this->dBackCartAngleDifference())/ANGLE_RANGE);
    this->_dAngularMultiplier = this->dCalculateAngularMultiplier(this->_dLinearMultiplier);
    _pdCaterpillarVelocities[FRONT_LINEAR] = this->_dLinearMultiplier * _pdControl[0];
    _pdCaterpillarVelocities[FRONT_ANGULAR] = this->_dAngularMultiplier * _pdControl[1];
    _pdCaterpillarVelocities[BACK_LINEAR] = this->_dLinearMultiplier * cos(_pdCartAngle[FRONT_CART]) / cos(_pdCartAngle[BACK_CART]) * _pdControl[0];
    _pdCaterpillarVelocities[BACK_ANGULAR] = this->_dAngularMultiplier * _pdControl[2];
    #ifdef DEBUG
        ROS_INFO_NAMED("kinematics_control", "FRONT;%s", _pFrontCartPID->cPrintDebugInfo());
        ROS_INFO_NAMED("kinematics_control", "BACK;%s", _pBackCartPID->cPrintDebugInfo());
        ROS_INFO_NAMED("kinematics_control", "LINEAR;FV=%f;BV=%f;mp=%f;SP=%f;dF=%f;dB=%f", _pdCaterpillarVelocities[FRONT_LINEAR], _pdCaterpillarVelocities[BACK_LINEAR],this->_dLinearMultiplier,_pdControl[0],this->dFrontCartAngleDifference()*RAD_TO_DEG,this->dBackCartAngleDifference()*RAD_TO_DEG);
    #endif
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
    this->_pFrontCartPID->Update(_pdCartAngle[FRONT_CART], dt);
    this->_pBackCartPID->Update(_pdCartAngle[BACK_CART], dt);
    this->_dLastTime = t;
}