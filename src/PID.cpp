#include <agribot_navigation/PID.h>
#include <cmath>
#include <sstream>
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
    this->_dBias = 0;
    this->_dError = 0;   
    this->_dProcessValue = 0;
    this->_dProcessValuePrior = 0;
    this->_dDerPrior = 0;
    this->_dDerPrevious = 0;
    this->_dControlValue = 0;
    this->_dErrorIntegral = 0;
    this->_dSetPoint = 0;
    this->_dNextSetPoint =0 ;
    this->_dDesiredSetPoint = 0;
    this->_dSubstepStepSize = 0;
    this->_iSubsteps = 0;
}
PID::PID(double Kp, double Kd, double Ki, double LL, double HL)
{
    this->_dKp = Kp;
    this->_dKd = Kd;
    this->_dKi = Ki;
    this->_dLowLimit = LL;
    this->_dHighLimit = HL;
    this->_dBias = 0;
    this->_dError = 0;   
    this->_dProcessValue = 0;
    this->_dProcessValuePrior = 0;
    this->_dDerPrior = 0;
    this->_dDerPrevious = 0;
    this->_dControlValue = 0;
    this->_dErrorIntegral = 0;
    this->_dSetPoint = 0;
    this->_dNextSetPoint =0 ;
    this->_dDesiredSetPoint = 0;
    this->_dSubstepStepSize = 0;
    this->_iSubsteps = 0;
}
void PID::vSetKp(double Kp)
{
    this->_dKp = Kp;
}
void PID::vSetKd(double Kd)
{
    this->_dKd = Kd;
}
void PID::vSetKi(double Ki)
{
    this->_dKi = Ki;
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
    this->_dErrorIntegral = 0;
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
     if (abs(ProcessValue - ActualNextSetPoint) < Substeps * SubstepStepSize)
    {
        if (abs(DesiredSetPoint - ActualNextSetPoint) > Substeps * SubstepStepSize)
        {
            if (DesiredSetPoint - ActualNextSetPoint > 0)
            {
                return ActualNextSetPoint + Substeps * SubstepStepSize;
            }
            else
            {
                return ActualNextSetPoint - Substeps * SubstepStepSize;
            }
        }
        else
        {
            return DesiredSetPoint;
        }
    } 
    else
    {
        return ActualNextSetPoint;
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
    else if (Value < this->_dLowLimit)
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
    double derrivative = (Actual - Previous) / dt;
    if (isnan(derrivative))
    {
        return 0;
    }
    else
    {
        return derrivative;
    }
}
double PID::dAverageFilter(double ValueNow, double ValueBack1, double ValueBack2)
{
    return (ValueNow + ValueBack1 + ValueBack2) / 3;
}
double PID::dErrorIntegration(double Error, double dt)
{
    return Error * dt;
}
bool PID::bAntiWindup(double Error, double ControlValue, double dt)
{
    /*
    Condition taken from 
    Visioli, Antonio. Practical PID control. Springer Science & Business Media, 2006.
    Chapter 3.3.2 Conditional Integration

    Implemented conditions:

        the integral term is limited to a predefined value;

        the integration is stopped when the control variable saturates and the
        control error and the control variable have the same sign (i.e., when u · e >
        0).

    */
    return ((ControlValue > this->_dHighLimit || ControlValue < this->_dLowLimit) && (ControlValue  * Error > 0));
}
double PID::dIntegralPart(double ErrorIntegral)
{
    return this->_dKi * ErrorIntegral;
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
    this->_dDerPrevious = this->_dDerPrior;
    this->_dDerPrior = this->_dDerivative;
    this->_dProcessValuePrior = this->_dProcessValue;   
}
double PID::Update(double ProcessValue, double dt)
{
    this->vUpdateProcessValue(ProcessValue);
    this->_dSetPoint = this->dSetPointRamp(this->_dDesiredSetPoint, this->_dProcessValue, this->_dSetPoint, 
                                        this->_dSubstepStepSize, this->_dNextSetPoint, this->_iSubsteps);
    this->_dError = this->dCalculateError(this->_dSetPoint, this->_dProcessValue);
    if(! this->bAntiWindup(this->_dError,this->_dControlValue, dt))
    {
        this->_dErrorIntegral += this->dErrorIntegration(this->_dError, dt);
    }
    /*
    Derrivative = Error - Error_prev = (SP - PV) - (SP_prev - PV_prev) = SP - PV - SP_prev + PV_prev
    Assuming SP_prev = SP:
    Derrivative = PV_prev - PV =  -(PV - PV_prev) = -dPV.
    */
    this->_dDerivative = -dCalculateDerivative(this->_dProcessValue,this->_dProcessValuePrior, dt);
    this->_dControlValue =  this->dProportionalPart(this->_dError) +
                            this->dIntegralPart(this->_dErrorIntegral) +
                            this->dDerivativePart(this->dAverageFilter(this->_dDerivative, this->_dDerPrior, this->_dDerPrevious)) +
                            this->dBiasPart();
    this->_dControlValue = this->dLimiter(this->_dControlValue);
    this->vRememberPriorValues();
    return this->_dControlValue;
}
std::string PID::cPrintDebugInfo()
{
    std::stringstream ss;
    ss << "SP=" << this->_dSetPoint << ";PV=" << this->_dProcessValue << ";CV=" << this->_dControlValue <<
         ";ER=" << this->_dError << ";IE=" << this->_dErrorIntegral << ";DE=" << this->_dDerivative <<
         ";KP=" << this->_dKp << ";KI=" << this->_dKi << ";KD=" << this->_dKd;
    return ss.str();
}
