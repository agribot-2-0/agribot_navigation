class PID
{
private:
    double _dKp;
    double _dKd;
    double _dKi;
    double _dBias;
    double _dError;
    double _dProcessValuePrior;
    double _dDerPrior;
    double _dDerPrevious;
    double _dControlValue;
    double _dErrorSum;
    double _dLowLimit;
    double _dHighLimit;
    double _dSetPoint;
    double _dNextSetPoint;
    double _dDesiredSetPoint;
    double _dMaxStep;
    double _dSubstepStepSize;
    int _iSubsteps;
public:
    PID();
    PID(double kp, double kd, double ki, double LL, double HL);
    void vSetKp(double Kp);
    void vSetKd(double Kd);
    void vSetKi(double Ki);
    void vSetLowLimit(double SP);
    void vSetHighLimit(double SP);
    void vSetSP(double SP);
    void vResetSP(double SP);
    void vSetBias(double Bias);
    void vSetSPAcceleration(double SubstepSize, int Substeps);
    void vResetError();
    void vSPRamp(double ProcessValue);
    void vCVLimiter();
    void vCalculateError(double ProcessValue);
    void vDerivativePart(double Derrivative);
    void vAntiWindup(double dt);
    void vIntegralPart();
    void vProportionalPart();
    double dAverageFilter(double value_now, double value_back_1, double value_back_2);
    double dCalculatePVDerivative(double ProcessValue,double dt);
    double dGetCV();
    double dGetError();
    double dGetDesiredSP();
    double Update(double PV, double dt, bool isFront);
};