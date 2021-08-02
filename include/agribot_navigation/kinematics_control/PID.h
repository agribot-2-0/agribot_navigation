/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#pragma once

#include <string>

class PID
{
private:
  double kp_;
  double kd_;
  double ki_;
  double bias_;
  double error_;
  double process_value_;
  double process_value_prior_;
  double derivative_;
  double der_prior_;
  double der_previous_;
  double control_value_;
  double error_integral_;
  double low_limit_;
  double high_limit_;
  double set_point_;
  double next_set_point_;
  double desired_set_point_;
  double last_desired_set_point_;
  double max_step_;
  double substep_step_size_;
  int substeps_;
  void updateProcessValue(double PV);
  void rememberPriorValues();
  bool antiWindup(double Error, double ControlValue, double dt);
  bool desiredSetPointCrossedNextSetPoint(double ProcessValue, double DesiredSetPoint, double LastDesiredSetPoint);
  double calculateNextSetPoint(double actual_next_set_point, double ProcessValue, double DesiredSetPoint, int Substeps,
                               double SubstepSize);
  double setPointSubstep(double ActualSetPoint, double NextSetPoint, double SubstepStepSize);
  double setPointRamp(double DesiredSetPoint, double ProcessValue, double ActualSetPoint, double SubstepStepSize,
                      double actual_next_set_point, int Substeps);
  double calculateError(double SetPoint, double ProcessValue);
  double derivativePart(double Derrivative);
  double errorIntegration(double Error, double dt);
  double integralPart(double ErrorIntegral);
  double proportionalPart(double Error);
  double biasPart();
  double limiter(double Value);
  double calculateDerivative(double Actual, double Previous, double dt);

public:
  PID();
  PID(double kp, double kd, double ki, double ll, double hl);
  void setKp(double kp);
  void setKd(double kd);
  void setKi(double ki);
  void setLowLimit(double ll);
  void setHighLimit(double hl);
  void setSp(double sp);
  void setBias(double bias);
  void setSpAcceleration(double subste_size, int substeps);
  void resetError();
  void resetSetPoints(double sp);
  double Update(double pv, double dt);
  std::string getDebugInfo();
  double averageFilter(double value_now, double value_back1, double value_back2);
  double getSetPoint();
  double getControlValue();
  double getProcessValue();
  double getError();
  double getIntegrationError();
  double getDerivativeError();
  double getProportionalPart();
  double getIntegralPart();
  double getDerivativePart();
  double getDesiredSetPoint();
};
