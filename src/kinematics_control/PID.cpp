/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_navigation/kinematics_control/PID.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <cmath>
#include <sstream>

#define DEBUG
#define RAD_TO_DEG (180 / M_PI)
#define DEG_TO_RAD (M_PI / 180)

PID::PID()
{
  this->kp_ = 0;
  this->kd_ = 0;
  this->ki_ = 0;
  this->low_limit_ = -20 * RAD_TO_DEG;
  this->high_limit_ = 20 * RAD_TO_DEG;
  this->bias_ = 0;
  this->error_ = 0;
  this->process_value_ = 0;
  this->process_value_prior_ = 0;
  this->der_prior_ = 0;
  this->der_previous_ = 0;
  this->control_value_ = 0;
  this->error_integral_ = 0;
  this->set_point_ = 0;
  this->next_set_point_ = 0;
  this->desired_set_point_ = 0;
  this->last_desired_set_point_ = 0;
  this->substep_step_size_ = 0;
  this->substeps_ = 0;
}

PID::PID(double Kp, double Kd, double Ki, double LL, double HL)
{
  this->kp_ = Kp;
  this->kd_ = Kd;
  this->ki_ = Ki;
  this->low_limit_ = LL;
  this->high_limit_ = HL;
  this->bias_ = 0;
  this->error_ = 0;
  this->process_value_ = 0;
  this->process_value_prior_ = 0;
  this->der_prior_ = 0;
  this->der_previous_ = 0;
  this->control_value_ = 0;
  this->error_integral_ = 0;
  this->set_point_ = 0;
  this->next_set_point_ = 0;
  this->desired_set_point_ = 0;
  this->last_desired_set_point_ = 0;
  this->substep_step_size_ = 0;
  this->substeps_ = 0;
}

void PID::setKp(double Kp)
{
  this->kp_ = Kp;
}

void PID::setKd(double Kd)
{
  this->kd_ = Kd;
}

void PID::setKi(double Ki)
{
  this->ki_ = Ki;
}

void PID::setLowLimit(double LL)
{
  this->low_limit_ = LL;
}

void PID::setHighLimit(double HL)
{
  this->high_limit_ = HL;
}

void PID::setSP(double SP)
{
  this->last_desired_set_point_ = this->desired_set_point_;
  this->desired_set_point_ = SP;
}

void PID::resetSetPoints(double SP)
{
  this->desired_set_point_ = SP;
  this->next_set_point_ = SP;
  this->set_point_ = SP;
}

void PID::setBias(double Bias)
{
  this->bias_ = Bias;
}

void PID::updateProcessValue(double PV)
{
  this->process_value_ = PV;
}

void PID::resetError()
{
  this->error_integral_ = 0;
}

void PID::setSPAcceleration(double SubstepStepSize, int Substeps)
{
  this->substep_step_size_ = SubstepStepSize;
  this->substeps_ = Substeps;
}

bool PID::desiredSetPointCrossedNextSetPoint(double ProcessValue, double DesiredSetPoint, double LastDesiredSetPoint)
{
  return (ProcessValue - DesiredSetPoint) * (ProcessValue - LastDesiredSetPoint) <= 0;
}

double PID::calculateNextSetPoint(double ActualNextSetPoint, double ProcessValue, double DesiredSetPoint, int Substeps,
                                   double SubstepStepSize)
{
  if (this->desiredSetPointCrossedNextSetPoint(ProcessValue, DesiredSetPoint, this->last_desired_set_point_))
  {
    this->resetError();
    ActualNextSetPoint = ProcessValue;
    this->last_desired_set_point_ = this->desired_set_point_;
  }

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

double PID::setPointSubstep(double ActualSetPoint, double NextSetPoint, double SubstepStepSize)
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

double PID::setPointRamp(double DesiredSetPoint, double ProcessValue, double ActualSetPoint, double SubstepStepSize,
                          double ActualNextSetPoint, int Substeps)
{
  this->next_set_point_ =
      this->calculateNextSetPoint(ActualNextSetPoint, ProcessValue, DesiredSetPoint, Substeps, SubstepStepSize);
  return this->setPointSubstep(ActualSetPoint, this->next_set_point_, SubstepStepSize);
}

double PID::limiter(double Value)
{
  if (Value > this->high_limit_)
  {
    return this->high_limit_;
  }
  else if (Value < this->low_limit_)
  {
    return this->low_limit_;
  }
  else
  {
    return Value;
  }
}

double PID::calculateError(double SetPoint, double ProcessValue)
{
  return SetPoint - ProcessValue;
}

double PID::calculateDerivative(double Actual, double Previous, double dt)
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

double PID::averageFilter(double ValueNow, double ValueBack1, double ValueBack2)
{
  return (ValueNow + ValueBack1 + ValueBack2) / 3;
}

double PID::errorIntegration(double Error, double dt)
{
  return Error * dt;
}

bool PID::antiWindup(double Error, double ControlValue, double dt)
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
  static_cast<void>(dt);
  return ((ControlValue > this->high_limit_ || ControlValue < this->low_limit_) && (ControlValue * Error > 0));
}

double PID::integralPart(double ErrorIntegral)
{
  return this->ki_ * ErrorIntegral;
}

double PID::proportionalPart(double Error)
{
  return this->kp_ * Error;
}

double PID::biasPart()
{
  return this->bias_;
}

double PID::derivativePart(double Derrivative)
{
  return this->kd_ * Derrivative;
}

void PID::rememberPriorValues()
{
  this->der_previous_ = this->der_prior_;
  this->der_prior_ = this->derivative_;
  this->process_value_prior_ = this->process_value_;
}

double PID::Update(double ProcessValue, double dt)
{
  this->updateProcessValue(ProcessValue);
  this->set_point_ = this->setPointRamp(this->desired_set_point_, this->process_value_, this->set_point_,
                                         this->substep_step_size_, this->next_set_point_, this->substeps_);
  this->error_ = this->calculateError(this->set_point_, this->process_value_);
  if (!this->antiWindup(this->error_, this->control_value_, dt))
  {
    this->error_integral_ += this->errorIntegration(this->error_, dt);
  }
  /*
  Derrivative = Error - Error_prev = (SP - PV) - (SP_prev - PV_prev) = SP - PV - SP_prev + PV_prev
  Assuming SP_prev = SP:
  Derrivative = PV_prev - PV =  -(PV - PV_prev) = -dPV.
  */
  this->derivative_ = -calculateDerivative(this->process_value_, this->process_value_prior_, dt);
  this->control_value_ =
      this->proportionalPart(this->error_) + this->integralPart(this->error_integral_) +
      this->derivativePart(this->averageFilter(this->derivative_, this->der_prior_, this->der_previous_)) +
      this->biasPart();
  this->control_value_ = this->limiter(this->control_value_);
  this->rememberPriorValues();
  return this->control_value_;
}

std::string PID::printDebugInfo()
{
  std::stringstream ss;
  ss << "DSP=" << this->desired_set_point_ * RAD_TO_DEG << ";LDSP=" << this->last_desired_set_point_ * RAD_TO_DEG
     << ";NSP=" << this->next_set_point_ * RAD_TO_DEG << ";dSP=" << this->substep_step_size_ * substeps_ * RAD_TO_DEG
     << ";SP=" << this->set_point_ * RAD_TO_DEG << ";PV=" << this->process_value_ * RAD_TO_DEG
     << ";CV=" << this->control_value_ * RAD_TO_DEG << ";ER=" << this->error_ * RAD_TO_DEG
     << ";IE=" << this->error_integral_ * RAD_TO_DEG << ";DE=" << this->derivative_ * RAD_TO_DEG
     << ";KP=" << this->kp_ << ";KI=" << this->ki_ << ";KD=" << this->kd_;
  return ss.str();
}

double PID::getSetPoint()
{
  return this->set_point_;
}

double PID::getControlValue()
{
  return this->control_value_;
}

double PID::getProcessValue()
{
  return this->process_value_;
}

double PID::getError()
{
  return this->error_;
}

double PID::getIntegrationError()
{
  return this->error_integral_;
}

double PID::getDerivativeError()
{
  return this->derivative_;
}

double PID::getProportionalPart()
{
  return this->proportionalPart(this->error_);
}

double PID::getIntegralPart()
{
  return this->integralPart(this->error_integral_);
}

double PID::getDerivativePart()
{
  return this->derivativePart(this->averageFilter(this->derivative_, this->der_prior_, this->der_previous_));
}

double PID::getDesiredSetPoint()
{
  return this->desired_set_point_;
}
