#include <agribot_navigation/PID.h>
#include <cmath>
#include <ros/ros.h>
#include <ros/console.h>
#define DEBUG
#define RAD_TO_DEG (180/M_PI)
#define DEG_TO_RAD (M_PI/180)
PID::PID()
{
    this->_dKp = 0;
    this->_dKd = 0;
    this->_dKi = 0;
    this->_dLowLimit = -20*RAD_TO_DEG;
    this->_dHighLimit = 20*RAD_TO_DEG;
    this->_dBias=0;
    this->_dError=0;
    this->_dProcessValue=0;
    this->_dProcessValuePrior=0;
    this->_dDerPrior=0;
    this->_dDerPrevious=0;
    this->_dControlValue=0;
    this->_dErrorSum=0;
    this->_dSetPoint=0;
    this->_dNextSetPoint=0;
    this->_dDesiredSetPoint=0;
    this->_dSubstepStepSize=0;
    this->_iSubsteps=0;
}
PID::PID(double kp, double kd, double ki, double LL, double HL)
{
    this->_dKp = kp;
    this->_dKd = kd;
    this->_dKi = ki;
    this->_dLowLimit = LL;
    this->_dHighLimit = HL;
    this->_dBias=0;
    this->_dError=0;   
    this->_dProcessValue=0;
    this->_dProcessValuePrior=0;
    this->_dDerPrior=0;
    this->_dDerPrevious=0;
    this->_dControlValue=0;
    this->_dErrorSum=0;
    this->_dSetPoint=0;
    this->_dNextSetPoint=0;
    this->_dDesiredSetPoint=0;
    this->_dSubstepStepSize=0;
    this->_iSubsteps=0;
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
void PID::vResetSetPoints(double SP)
{
    this->_dDesiredSetPoint = SP;
    this->_dNextSetPoint = SP;
    this->_dSetPoint = SP;
}
void PID::vSetBias(double Bias)
{
    this->_dBias = Bias;
}
void PID::vUpdateProcessValue(double PV)
{
    this->_dProcessValue = PV;
}
void PID::vResetError()
{
    this->_dErrorSum = 0;
}
void PID::vSetSPAcceleration(double SubstepStepSize, int Substeps)
{
    this->_dSubstepStepSize = SubstepStepSize;
    this->_iSubsteps = Substeps;
}
void PID::vCalculateNextSetPoint(double ProcessValue)
{
     if (abs(ProcessValue-this->_dNextSetPoint)<this->_iSubsteps*this->_dSubstepStepSize)
    {
        if (abs(this->_dDesiredSetPoint-this->_dNextSetPoint)>this->_iSubsteps*this->_dSubstepStepSize)
        {
            if (this->_dDesiredSetPoint-this->_dNextSetPoint>0)
            {
                this->_dNextSetPoint+=this->_iSubsteps*this->_dSubstepStepSize;
            }
            else
            {
                this->_dNextSetPoint-=this->_iSubsteps*this->_dSubstepStepSize;
            }
        }
        else
        {
            this->_dNextSetPoint=this->_dDesiredSetPoint;
        }
    } 
}
void PID::vSetPointSubstep()
{
    if (this->_dNextSetPoint - this->_dSetPoint < 0)
    {
        if (this->_dNextSetPoint - this->_dSetPoint > -this->_dSubstepStepSize)
        {
            this->_dSetPoint = this->_dNextSetPoint;
        }
        else
        {
            this->_dSetPoint -= this->_dSubstepStepSize;
        }  
    }
    else
    {
        if (this->_dNextSetPoint - this->_dSetPoint < this->_dSubstepStepSize)
        {
            this->_dSetPoint = this->_dNextSetPoint;
        }
        else
        {
            this->_dSetPoint += this->_dSubstepStepSize;
        }
    }
}
void PID::vSetPointRamp(double ProcessValue)
{
     this->vCalculateNextSetPoint(ProcessValue);
     this->vSetPointSubstep();
}
void PID::vControlValueLimiter()
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
double PID::dGetControlValue()
{
    return this->_dControlValue;
}
double PID::dGetError()
{
    return this->_dError;
}
double PID::dGetDesiredSetPoint()
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
void PID::vAntiWindup(double dt)
{
    if ((this->_dControlValue > this->_dHighLimit || this->_dControlValue < this->_dLowLimit) 
     && (this->_dControlValue  * this->_dError > 0))
    {}
    else
    {
        this->_dErrorSum += this->_dError * dt;
    }
}
void PID::vIntegralPart()
{
    if ((this->_dControlValue > this->_dHighLimit || this->_dControlValue < this->_dLowLimit) 
     && (this->_dControlValue  * this->_dError > 0))
    {}
    else
    {
        this->_dControlValue=_dKi * this->_dErrorSum; ;
    }
}
void PID::vProportionalPart()
{
    this->_dControlValue += this->_dKp * this->_dError; 
    this->_dControlValue += this->_dBias;
}
double PID::dAverageFilter(double value_now, double value_back_1, double value_back_2)
{
    return (value_now+value_back_1+value_back_2)/3;
}
void PID::vDerivativePart(double Derrivative)
{
    this->_dControlValue += this->_dKd * this->dAverageFilter(Derrivative,this->_dDerPrior,this->_dDerPrevious); 
}
double PID::Update(double ProcessValue, double dt, bool isFront)
{
    this->vUpdateProcessValue(ProcessValue);
    this->vSetPointRamp(this->_dProcessValue);
    this->vCalculateError(this->_dProcessValue);
    this->_dDerivative = dCalculatePVDerivative(this->_dProcessValue, dt);
    this->_dControlValue = 0;
    this->vAntiWindup(dt);
    this->vIntegralPart();
    this->vProportionalPart();
    this->vDerivativePart(this->_dDerivative);
    this->vControlValueLimiter();
    #ifdef DEBUG
        this->vPrintDebugInfo(isFront, dt);
    #endif
    this->_dDerPrevious=this->_dDerPrior;
    this->_dDerPrior=this->_dDerivative;
    this->_dProcessValuePrior = this->_dProcessValue;                 
    return this->_dControlValue;
}
void PID::vPrintDebugInfo(bool isFront, double dt)
{
    if (isFront)
        {
            ROS_INFO_NAMED("PID", "FRONT;dt=%f;SP=%f;PV=%f;CV=%f;ER=%f;IE=%f;DE=%f;KP=%f;KI=%f;KD=%f",
                        dt,this->_dSetPoint,this->_dProcessValue,this->_dControlValue,this->_dError,this->_dErrorSum,this->_dDerivative,this->_dKp,this->_dKi,this->_dKd);
        }
        else
        {
            ROS_INFO_NAMED("PID", "BACK;dt=%f;SP=%f;PV=%f;CV=%f;ER=%f;IE=%f;DE=%f;KP=%f;KI=%f;KD=%f",
                        dt,this->_dSetPoint,this->_dProcessValue,this->_dControlValue,this->_dError,this->_dErrorSum,this->_dDerivative,this->_dKp,this->_dKi,this->_dKd);
        }
}
