#include <agribot_navigation/PID.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>

#define SP_ACCELERATION 2*3.14159265359/180

PID::PID()
{
    this->_dDesiredSetPoint = 0;
    this->_dNextSetPoint = 0;
    this->_dSetPoint = 0;
    this->_dErrorSum = 0;
    this->_dProcessValuePrior = 0;
    this->_dDerPrior = 0;
    this->_dDerPrevious = 0;
    this->_dControlValue = 0;
    this->_dBias = 0;
    this->_dSpAcceleration = 0.15*3.14159265359/180;
    this->_dSpAccMultiplier = 20;
}
PID::PID(double kp, double kd, double ki, double LL, double HL)
{
    this->_dKp = kp;
    this->_dKd = kd;
    this->_dKi = ki;
    this->_dLowLimit = LL;
    this->_dHighLimit = HL;
    this->_dDesiredSetPoint = 0;
    this->_dNextSetPoint = 0;
    this->_dSetPoint = 0;
    this->_dErrorSum = 0;
    this->_dProcessValuePrior = 0;
    this->_dDerPrior = 0;
    this->_dDerPrevious = 0;
    this->_dControlValue = 0;
    this->_dBias = 0;
    this->_dSpAcceleration = 0.15*3.14159265359/180;
    this->_dSpAccMultiplier = 20;
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
void PID::vSetSPAcceleration(double Acceleration, double Multiplier)
{
    this->_dSpAcceleration = Acceleration;
    this->_dSpAccMultiplier = Multiplier;
}
void PID::vResetError()
{
    this->_dErrorSum = 0;
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
double PID::Update(double ProcessValue, double dt, bool isFront)
{
    //SP RAMP
    if (abs(ProcessValue-this->_dNextSetPoint)<this->_dSpAccMultiplier*this->_dSpAcceleration)
    {
        if (abs(this->_dDesiredSetPoint-this->_dNextSetPoint)>this->_dSpAccMultiplier*this->_dSpAcceleration)
        {
            if (this->_dDesiredSetPoint-this->_dNextSetPoint>0)
            {
                this->_dNextSetPoint+=this->_dSpAccMultiplier*this->_dSpAcceleration;
            }
            else
            {
                this->_dNextSetPoint-=this->_dSpAccMultiplier*this->_dSpAcceleration;
            }
            
        }
        else
        {
            this->_dNextSetPoint=this->_dDesiredSetPoint;
        }
        
    }
    
    if (this->_dNextSetPoint - this->_dSetPoint < 0)
    {
        if (this->_dNextSetPoint - this->_dSetPoint > -this->_dSpAcceleration)
        {
            this->_dSetPoint = this->_dNextSetPoint;
        }
        else
        {
            this->_dSetPoint -= this->_dSpAcceleration;
        }  
    }
    else
    {
        if (this->_dNextSetPoint - this->_dSetPoint < this->_dSpAcceleration)
        {
            this->_dSetPoint = this->_dNextSetPoint;
        }
        else
        {
            this->_dSetPoint += this->_dSpAcceleration;
        }
    }

    //ERROR CALCULTATION
    this->_dError = this->_dSetPoint - ProcessValue;
    double Derrivative = (this->_dProcessValuePrior - ProcessValue) / dt;
    //ANTI-WINDUP
    if ((this->_dControlValue > this->_dHighLimit || this->_dControlValue < this->_dLowLimit) 
     && (this->_dControlValue  * this->_dError > 0))
    {
        this->_dControlValue=0;
    }           
    else
    {
        this->_dErrorSum += this->_dError * dt;
        this->_dControlValue=_dKi * this->_dErrorSum; ;
    }
    this->_dControlValue += this->_dKp * this->_dError; 
    this->_dControlValue += this->_dKd * (Derrivative/3+this->_dDerPrior/3+this->_dDerPrevious/3); 
    this->_dControlValue += this->_dBias;
    //INFO TO PLOT
    if (this->_dControlValue > this->_dHighLimit)
    {
        this->_dControlValue = this->_dHighLimit;
    }
    if (this->_dControlValue < this->_dLowLimit)
    {
        this->_dControlValue = this->_dLowLimit;
    }
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