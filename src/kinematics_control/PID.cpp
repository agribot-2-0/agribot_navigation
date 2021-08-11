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

PID::PID(double kp, double kd, double ki, double ll, double hl)
{
  this->kp_ = kp;
  this->kd_ = kd;
  this->ki_ = ki;
  this->low_limit_ = ll;
  this->high_limit_ = hl;
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

void PID::setKp(double kp)
{
  this->kp_ = kp;
}

void PID::setKd(double kd)
{
  this->kd_ = kd;
}

void PID::setKi(double ki)
{
  this->ki_ = ki;
}

void PID::setLowLimit(double ll)
{
  this->low_limit_ = ll;
}

void PID::setHighLimit(double hl)
{
  this->high_limit_ = hl;
}

void PID::setSp(double sp)
{
  this->last_desired_set_point_ = this->desired_set_point_;
  this->desired_set_point_ = sp;
}

void PID::resetSetPoints(double sp)
{
  this->desired_set_point_ = sp;
  this->next_set_point_ = sp;
  this->set_point_ = sp;
}

void PID::setBias(double bias)
{
  this->bias_ = bias;
}

void PID::updateProcessValue(double pv)
{
  this->process_value_ = pv;
}

void PID::resetError()
{
  this->error_integral_ = 0;
}

void PID::setSpAcceleration(double substep_step_size, int substeps)
{
  this->substep_step_size_ = substep_step_size;
  this->substeps_ = substeps;
}

bool PID::desiredSetPointCrossedNextSetPoint(double process_value, double desired_set_point,
                                             double last_desired_set_point)
{
  return (process_value - desired_set_point) * (process_value - last_desired_set_point) <= 0;
}

double PID::calculateNextSetPoint(double actual_next_set_point, double process_value, double desired_set_point,
                                  int substeps, double substep_step_size)
{
  if (this->desiredSetPointCrossedNextSetPoint(process_value, desired_set_point, this->last_desired_set_point_))
  {
    this->resetError();
    actual_next_set_point = process_value;
    this->last_desired_set_point_ = this->desired_set_point_;
  }

  if (abs(process_value - actual_next_set_point) < substeps * substep_step_size)
  {
    if (abs(desired_set_point - actual_next_set_point) > substeps * substep_step_size)
    {
      if (desired_set_point - actual_next_set_point > 0)
      {
        return actual_next_set_point + substeps * substep_step_size;
      }
      else
      {
        return actual_next_set_point - substeps * substep_step_size;
      }
    }
    else
    {
      return desired_set_point;
    }
  }
  else
  {
    return actual_next_set_point;
  }
}

double PID::setPointSubstep(double actual_set_point, double next_set_point, double substep_step_size)
{
  if (next_set_point - actual_set_point < 0)
  {
    if (next_set_point - actual_set_point > -substep_step_size)
    {
      return next_set_point;
    }
    else
    {
      return actual_set_point - substep_step_size;
    }
  }
  else
  {
    if (next_set_point - actual_set_point < substep_step_size)
    {
      return next_set_point;
    }
    else
    {
      return actual_set_point + substep_step_size;
    }
  }
}

double PID::setPointRamp(double desired_set_point, double process_value, double actual_set_point,
                         double substep_step_size, double actual_next_set_point, int substeps)
{
  this->next_set_point_ =
      this->calculateNextSetPoint(actual_next_set_point, process_value, desired_set_point, substeps, substep_step_size);
  return this->setPointSubstep(actual_set_point, this->next_set_point_, substep_step_size);
}

double PID::limiter(double value)
{
  if (value > this->high_limit_)
  {
    return this->high_limit_;
  }
  else if (value < this->low_limit_)
  {
    return this->low_limit_;
  }
  else
  {
    return value;
  }
}

double PID::calculateError(double set_point, double process_value)
{
  return set_point - process_value;
}

double PID::calculateDerivative(double actual, double previous, double dt)
{
  double derrivative = (actual - previous) / dt;
  if (isnan(derrivative))
  {
    return 0;
  }
  else
  {
    return derrivative;
  }
}

double PID::averageFilter(double value_now, double value_back1, double value_back2)
{
  return (value_now + value_back1 + value_back2) / 3;
}

double PID::errorIntegration(double error, double dt)
{
  return error * dt;
}

bool PID::antiWindup(double error, double control_value, double dt)
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
  return ((control_value > this->high_limit_ || control_value < this->low_limit_) && (control_value * error > 0));
}

double PID::integralPart(double error_integral)
{
  return this->ki_ * error_integral;
}

double PID::proportionalPart(double error)
{
  return this->kp_ * error;
}

double PID::biasPart()
{
  return this->bias_;
}

double PID::derivativePart(double derrivative)
{
  return this->kd_ * derrivative;
}

void PID::rememberPriorValues()
{
  this->der_previous_ = this->der_prior_;
  this->der_prior_ = this->derivative_;
  this->process_value_prior_ = this->process_value_;
}

double PID::Update(double process_value, double dt)
{
  this->updateProcessValue(process_value);
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

std::string PID::getDebugInfo()
{
  std::stringstream ss;
  ss << "DSP=" << this->desired_set_point_ * RAD_TO_DEG << ";LDSP=" << this->last_desired_set_point_ * RAD_TO_DEG
     << ";NSP=" << this->next_set_point_ * RAD_TO_DEG << ";dSP=" << this->substep_step_size_ * substeps_ * RAD_TO_DEG
     << ";SP=" << this->set_point_ * RAD_TO_DEG << ";PV=" << this->process_value_ * RAD_TO_DEG
     << ";CV=" << this->control_value_ * RAD_TO_DEG << ";ER=" << this->error_ * RAD_TO_DEG
     << ";IE=" << this->error_integral_ * RAD_TO_DEG << ";DE=" << this->derivative_ * RAD_TO_DEG << ";KP=" << this->kp_
     << ";KI=" << this->ki_ << ";KD=" << this->kd_;
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
