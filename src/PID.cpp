#include <agribot_navigation/PID.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>

PID::PID()
{
    this->_SetPoint = 0;
    this->_ErrorSum = 0;
    this->_ErrorPrior = 0;
    this->_ControlValue = 0;
    this->_Bias = 0;
}
PID::PID(double kp, double kd, double ki, double LL, double HL)
{
    this->_Kp = kp;
    this->_Kd = kd;
    this->_Ki = ki;
    this->_LowLimit = LL;
    this->_HighLimit = HL;
    this->_SetPoint = 0;
    this->_ErrorSum = 0;
    this->_ErrorPrior = 0;
    this->_ControlValue = 0;
    this->_Bias = 0;
}
void PID::vSetKp(double kp)
{
    this->_Kp = kp;
}
void PID::vSetKd(double kd)
{
    this->_Kd = kd;
}
void PID::vSetKi(double ki)
{
    this->_Ki = ki;
}
void PID::vSetLowLimit(double LL)
{
    this->_LowLimit = LL;
}
void PID::vSetHighLimit(double HL)
{
    this->_HighLimit = HL;
}
void PID::vSetSP(double SP)
{
    this->_SetPoint = SP;
}
void PID::vSetBias(double Bias)
{
    this->_Bias = Bias;
}
void PID::vResetError()
{
    this->_ErrorSum = 0;
}
double PID::dGetCV()
{
    return this->_ControlValue;
}
double PID::Update(double PresentValue, double dt, bool isFront)
{
    this->_Error = this->_SetPoint - PresentValue;
    if (this->_ControlValue < this->_HighLimit && this->_ControlValue > this->_LowLimit)
    {
        this->_ErrorSum += this->_Error * dt;
    }
    double Derrivative = (this->_Error - this->_ErrorPrior) / dt;
    //ANTI-WINDUP
    if ((this->_ControlValue > this->_HighLimit || this->_ControlValue < this->_LowLimit) 
     && (this->_ControlValue  * this->_Error > 0))
    {
        this->_ControlValue=0;
    }           
    else
    {
        this->_ControlValue=_Ki * this->_ErrorSum; ;
    }
    this->_ControlValue += this->_Kp * this->_Error; 
    this->_ControlValue += this->_Kd * Derrivative; 
    this->_ControlValue += this->_Bias;
    if (isFront)
    {
        ROS_INFO_NAMED("PID", "FRONT;dt=%f;SP=%f;PV=%f;CV=%f;ER=%f;IE=%f;DE=%f",
                    dt,this->_SetPoint,PresentValue,this->_ControlValue,this->_Error,this->_ErrorSum,Derrivative);
    }
    else
    {
        ROS_INFO_NAMED("PID", "BACK;dt=%f;SP=%f;PV=%f;CV=%f;ER=%f;IE=%f;DE=%f",
                    dt,this->_SetPoint,PresentValue,this->_ControlValue,this->_Error,this->_ErrorSum,Derrivative);
    }
    this->_ErrorPrior = this->_Error;                 
    return this->_ControlValue;
}