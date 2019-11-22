#include <agribot_navigation/PID.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#define PI 3.14159265359
#define RAD_TO_DEG (180/PI)
#define DEG_TO_RAD (PI/180)
#define DT 0.15
#define SP_PRECISION 0.05
#define PV_vs_SP_PRECISION 0.05
PID::PID()
{
    this->_dErrorSum = 0;
    this->_dProcessValuePrior = 0;
    this->_dDerPrior = 0;
    this->_dDerPrevious = 0;
    this->_dControlValue = 0;
    this->_dBias = 0;
    this->_dDesiredSetPoint = 0;
    this->_dSetPoint = 0;
}
PID::PID(double kp, double kd, double ki, double LL, double HL)
{
    this->_dKp = kp;
    this->_dKd = kd;
    this->_dKi = ki;
    this->_dLowLimit = LL;
    this->_dHighLimit = HL;
    this->_dErrorSum = 0;
    this->_dProcessValuePrior = 0;
    this->_dDerPrior = 0;
    this->_dDerPrevious = 0;
    this->_dControlValue = 0;
    this->_dBias = 0;
    this->_dDesiredSetPoint = 0;
    this->_dSetPoint = 0;
}
void PID::vSetKp(double kp)
{
    this->_dKp = kp;
}
void PID::vSetKd(double kd)
{
    this->_dKd = kd;
}
void PID::vSetKi(double ki)
{
    this->_dKi = ki;
}
void PID::vSetLowLimit(double LL)
{
    this->_dLowLimit = LL;
}
void PID::vSetHighLimit(double HL)
{
    this->_dHighLimit = HL;
}
void PID::vSetSP(double SP)
{
    this->_dDesiredSetPoint = SP;
}
void PID::vSetBias(double Bias)
{
    this->_dBias = Bias;
}
void PID::vResetError()
{
    this->_dErrorSum = 0;
}
void PID::vSetSPAcceleration(double StepSize, double StepTime)
{
    this->_dMaxStep = StepSize;
    this->_iEndSubstep = StepTime/DT;
}
double PID::dLogisticFunction(int n)
{
    double a=this->_dLastSetPoint;
    double b=this->_dLastSetPoint+this->_dStepSize;
    double c=(this->_dStepTime/2)/10;
    double r=7.5*c;
    double t=n/10.0;
    return((a*exp(c*r)+b*exp(r*t))/(exp(c*r)+exp(r*t)));
}
void PID::vComparePVandSP(double ProcessValue)
{
    if(abs(ProcessValue-(this->_dLastSetPoint + this->_dStepSize)) < PV_vs_SP_PRECISION)
    {
        this->_iSubstep=0;
        this->_dLastSetPoint = this->_dSetPoint;
        if(abs(this->_dDesiredSetPoint-this->_dLastSetPoint) < this->_dMaxStep)
        {
            this->_dStepSize=this->_dDesiredSetPoint-this->_dLastSetPoint;
        }
        else
        {
            if(this->_dDesiredSetPoint > this->_dLastSetPoint)
            {
                this->_dStepSize=this->_dMaxStep;
            }
            else
            {
                this->_dStepSize=-this->_dMaxStep;
            }
            
        }
        
    }
}
void PID::vSPRamp(double ProcessValue)
{
    if (abs(this->_dSetPoint - (this->_dLastSetPoint + this->_dStepSize)) < SP_PRECISION)
    {
        this->vComparePVandSP(ProcessValue);
    }
    else
    {
        if (this->_iSubstep<this->_iEndSubstep)
        {
            this->_iSubstep++;
        }
        this->_dSetPoint=dLogisticFunction(this->_iSubstep++);
    }
      
}
void PID::vCVLimiter()
{
     if (this->_dControlValue > this->_dHighLimit)
    {
        this->_dControlValue = this->_dHighLimit;
    }
    if (this->_dControlValue < this->_dLowLimit)
    {
        this->_dControlValue = this->_dLowLimit;
    }
}
double PID::dGetCV()
{
    return this->_dControlValue;
}
double PID::dGetError()
{
    return this->_dError;
}
double PID::dGetDesiredSP()
{
    return this->_dDesiredSetPoint;
}
void PID::vCalculateError(double ProcessValue)
{
    this->_dError = this->_dSetPoint - ProcessValue;
}
double PID::dCalculatePVDerivative(double ProcessValue, double dt)
{
    return (this->_dProcessValuePrior - ProcessValue) / dt;
}
void PID::vIntegralPart(double dt)
{
    //ANTI-WINDUP
    if ((this->_dControlValue > this->_dHighLimit || this->_dControlValue < this->_dLowLimit) 
     && (this->_dControlValue  * this->_dError > 0))
    {}
    else
    {
        this->_dErrorSum += this->_dError * dt;
        this->_dControlValue=_dKi * this->_dErrorSum; ;
    }
}
void PID::vProportionalPart()
{
    this->_dControlValue += this->_dKp * this->_dError; 
}
double PID::dPVFilter(double value_now, double value_back_1, double value_back_2)
{
    return value_now/3+value_back_1/3+value_back_2/3;
}
void PID::vDerivativePart(double Derrivative)
{
    this->_dControlValue += this->_dKd * this->dPVFilter(Derrivative,this->_dDerPrior,this->_dDerPrevious); 
    this->_dControlValue += this->_dBias;
}
double PID::Update(double ProcessValue, double dt, bool isFront)
{
    this->vSPRamp(ProcessValue);
    this->vCalculateError(ProcessValue);
    double Derrivative = dCalculatePVDerivative(ProcessValue, dt);
    this->_dControlValue = 0;
    this->vIntegralPart(dt);
    this->vProportionalPart();
    this->vDerivativePart(Derrivative);
    this->vCVLimiter();
    if (isFront)
    {
        ROS_INFO_NAMED("PID", "FRONT;dt=%f;SP=%f;PV=%f;CV=%f;ER=%f;IE=%f;DE=%f;KP=%f;KI=%f;KD=%f",
                    dt,this->_dSetPoint,ProcessValue,this->_dControlValue,this->_dError,this->_dErrorSum,Derrivative,this->_dKp,this->_dKi,this->_dKd);
    }
    else
    {
        ROS_INFO_NAMED("PID", "BACK;dt=%f;SP=%f;PV=%f;CV=%f;ER=%f;IE=%f;DE=%f;KP=%f;KI=%f;KD=%f",
                    dt,this->_dSetPoint,ProcessValue,this->_dControlValue,this->_dError,this->_dErrorSum,Derrivative,this->_dKp,this->_dKi,this->_dKd);
    }
    this->_dDerPrevious=this->_dDerPrior;
    this->_dDerPrior=Derrivative;
    this->_dProcessValuePrior = ProcessValue;                 
    return this->_dControlValue;
}