#include <agribot_navigation/PID.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>

PID::PID()
{
    this->_DesiredSetPoint = 0;
    this->_NextSetPoint = 0;
    this->_SetPoint = 0;
    this->_ErrorSum = 0;
    this->_ProcessValuePrior = 0;
    this->_DerPrior = 0;
    this->_DerPrevious = 0;
    this->_ControlValue = 0;
    this->_Bias = 0;
    this->_SpAcceleration = 0.15*3.14159265359/180;
    this->_SpAccMultiplier = 20;
}
PID::PID(double kp, double kd, double ki, double LL, double HL)
{
    this->_Kp = kp;
    this->_Kd = kd;
    this->_Ki = ki;
    this->_LowLimit = LL;
    this->_HighLimit = HL;
    this->_DesiredSetPoint = 0;
    this->_NextSetPoint = 0;
    this->_SetPoint = 0;
    this->_ErrorSum = 0;
    this->_ProcessValuePrior = 0;
    this->_DerPrior = 0;
    this->_DerPrevious = 0;
    this->_ControlValue = 0;
    this->_Bias = 0;
    this->_SpAcceleration = 0.15*3.14159265359/180;
    this->_SpAccMultiplier = 20;
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
    this->_DesiredSetPoint = SP;
}
void PID::vSetBias(double Bias)
{
    this->_Bias = Bias;
}
void PID::vSetSPAcceleration(double Acceleration, double Multiplier)
{
    this->_SpAcceleration = Acceleration;
    this->_SpAccMultiplier = Multiplier;
}
void PID::vResetError()
{
    this->_ErrorSum = 0;
}
double PID::dGetCV()
{
    return this->_ControlValue;
}
double PID::dGetError()
{
    return this->_Error;
}
double PID::dGetDesiredSP()
{
    return this->_DesiredSetPoint;
}
double PID::Update(double ProcessValue, double dt, bool isFront)
{
    //SP RAMP
    if (abs(ProcessValue-this->_NextSetPoint)<this->_SpAccMultiplier*this->_SpAcceleration)
    {
        if (abs(this->_DesiredSetPoint-this->_NextSetPoint)>this->_SpAccMultiplier*this->_SpAcceleration)
        {
            if (this->_DesiredSetPoint-this->_NextSetPoint>0)
            {
                this->_NextSetPoint+=this->_SpAccMultiplier*this->_SpAcceleration;
            }
            else
            {
                this->_NextSetPoint-=this->_SpAccMultiplier*this->_SpAcceleration;
            }
            
        }
        else
        {
            this->_NextSetPoint=this->_DesiredSetPoint;
        }
        
    }
    
    if (this->_NextSetPoint - this->_SetPoint < 0)
    {
        if (this->_NextSetPoint - this->_SetPoint > -this->_SpAcceleration)
        {
            this->_SetPoint = this->_NextSetPoint;
        }
        else
        {
            this->_SetPoint -= this->_SpAcceleration;
        }  
    }
    else
    {
        if (this->_NextSetPoint - this->_SetPoint < this->_SpAcceleration)
        {
            this->_SetPoint = this->_NextSetPoint;
        }
        else
        {
            this->_SetPoint += this->_SpAcceleration;
        }
    }

    //ERROR CALCULTATION
    this->_Error = this->_SetPoint - ProcessValue;
    double Derrivative = (this->_ProcessValuePrior - ProcessValue) / dt;
    //ANTI-WINDUP
    if ((this->_ControlValue > this->_HighLimit || this->_ControlValue < this->_LowLimit) 
     && (this->_ControlValue  * this->_Error > 0))
    {
        this->_ControlValue=0;
    }           
    else
    {
        this->_ErrorSum += this->_Error * dt;
        this->_ControlValue=_Ki * this->_ErrorSum; ;
    }
    this->_ControlValue += this->_Kp * this->_Error; 
    this->_ControlValue += this->_Kd * (Derrivative/3+this->_DerPrior/3+this->_DerPrevious/3); 
    this->_ControlValue += this->_Bias;
    //INFO TO PLOT
    if (this->_ControlValue > this->_HighLimit)
    {
        this->_ControlValue = this->_HighLimit;
    }
    if (this->_ControlValue < this->_LowLimit)
    {
        this->_ControlValue = this->_LowLimit;
    }
    if (isFront)
    {
        ROS_INFO_NAMED("PID", "FRONT;dt=%f;SP=%f;PV=%f;CV=%f;ER=%f;IE=%f;DE=%f;KP=%f;KI=%f;KD=%f",
                    dt,this->_SetPoint,ProcessValue,this->_ControlValue,this->_Error,this->_ErrorSum,Derrivative,this->_Kp,this->_Ki,this->_Kd);
    }
    else
    {
        ROS_INFO_NAMED("PID", "BACK;dt=%f;SP=%f;PV=%f;CV=%f;ER=%f;IE=%f;DE=%f;KP=%f;KI=%f;KD=%f",
                    dt,this->_SetPoint,ProcessValue,this->_ControlValue,this->_Error,this->_ErrorSum,Derrivative,this->_Kp,this->_Ki,this->_Kd);
    }
    this->_DerPrevious=this->_DerPrior;
    this->_DerPrior=Derrivative;
    this->_ProcessValuePrior = ProcessValue;                 
    return this->_ControlValue;
}