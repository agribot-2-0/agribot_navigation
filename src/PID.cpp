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
double PID::dCalculateNextSetPoint(double ActualNextSetPoint, double ProcessValue, double DesiredSetPoint, int Substeps, double SubstepStepSize)
{
     if (abs(ProcessValue-ActualNextSetPoint)<Substeps*SubstepStepSize)
    {
        if (abs(DesiredSetPoint-ActualNextSetPoint)>Substeps*SubstepStepSize)
        {
            if (DesiredSetPoint-ActualNextSetPoint>0)
            {
                return ActualNextSetPoint+Substeps*SubstepStepSize;
            }
            else
            {
                return ActualNextSetPoint-Substeps*SubstepStepSize;
            }
        }
        else
        {
            return DesiredSetPoint;
        }
    } 
}
double PID::dSetPointSubstep(double ActualSetPoint, double NextSetPoint, double SubstepStepSize)
{
    if (NextSetPoint - ActualSetPoint < 0)
    {
        if (NextSetPoint - ActualSetPoint > -SubstepStepSize)
        {
            return NextSetPoint;
        }
        else
        {
            return ActualSetPoint - SubstepStepSize;
        }  
    }
    else
    {
        if (NextSetPoint - ActualSetPoint < SubstepStepSize)
        {
            return NextSetPoint;
        }
        else
        {
            return ActualSetPoint + SubstepStepSize;
        }
    }
}
double PID::dSetPointRamp(double DesiredSetPoint, double ProcessValue, double ActualSetPoint, double SubstepStepSize, double ActualNextSetPoint, int Substeps)
{
     this->_dNextSetPoint = this->dCalculateNextSetPoint(ActualNextSetPoint,ProcessValue, DesiredSetPoint, Substeps, SubstepStepSize);
     return this->dSetPointSubstep(ActualSetPoint, this->_dNextSetPoint, SubstepStepSize);
}
double PID::dLimiter(double Value)
{
     if (Value > this->_dHighLimit)
    {
        return this->_dHighLimit;
    }
    else
        if (Value < this->_dLowLimit)
        {
            return this->_dLowLimit;
        }
        else
        {
            return Value;
        }
}
double PID::dCalculateError(double SetPoint, double ProcessValue)
{
    return SetPoint - ProcessValue;
}
double PID::dCalculateDerivative(double Actual, double Previous, double dt)
{
    return (Actual - Previous) / dt;
}
double PID::dAverageFilter(double value_now, double value_back_1, double value_back_2)
{
    return (value_now+value_back_1+value_back_2)/3;
}
double PID::dAntiWindup(double Error, double ControlValue, double dt)
{
    if ((ControlValue > this->_dHighLimit || ControlValue < this->_dLowLimit) 
     && (ControlValue  * Error > 0))
    {
        return 0.0;
    }
    else
    {
         return Error * dt;
    }
}
double PID::dIntegralPart(double ErrorSum)
{
        return this->_dKi * ErrorSum;
}
double PID::dProportionalPart(double Error)
{
    return this->_dKp * Error; 
}
double PID::dBiasPart()
{
    return this->_dBias;
}
double PID::dDerivativePart(double Derrivative)
{
    return this->_dKd * Derrivative; 
}
void PID::vRememberPriorValues()
{
    this->_dDerPrevious=this->_dDerPrior;
    this->_dDerPrior=this->_dDerivative;
    this->_dProcessValuePrior = this->_dProcessValue;   
}
double PID::Update(double ProcessValue, double dt, bool isFront)
{
    this->vUpdateProcessValue(ProcessValue);
    this->_dSetPoint=this->dSetPointRamp(this->_dDesiredSetPoint, this->_dProcessValue, this->_dSetPoint, this->_dSubstepStepSize, this->_dNextSetPoint, this->_iSubsteps);
    this->_dError=this->dCalculateError(this->_dSetPoint, this->_dProcessValue);
    this->_dDerivative = -dCalculateDerivative(this->_dProcessValue,this->_dProcessValuePrior, dt);
    this->_dErrorSum += this->dAntiWindup(this->_dError,this->_dControlValue, dt);
    this->_dControlValue =  this->dIntegralPart(this->_dErrorSum) +
                            this->dProportionalPart(this->_dError) +
                            this->dDerivativePart(this->dAverageFilter(this->_dDerivative, this->_dDerPrior, this->_dDerPrevious)) +
                            this->dBiasPart();
    this->_dControlValue = this->dLimiter(this->_dControlValue);
    this->vRememberPriorValues();
    #ifdef DEBUG
        this->vPrintDebugInfo(isFront, dt);
    #endif
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
