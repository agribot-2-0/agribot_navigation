#include <string> 

class PID
{
private:
    double _dKp;
    double _dKd;
    double _dKi;
    double _dBias;
    double _dError;
    double _dProcessValue;
    double _dProcessValuePrior;
    double _dDerivative;
    double _dDerPrior;
    double _dDerPrevious;
    double _dControlValue;
    double _dErrorIntegral;
    double _dLowLimit;
    double _dHighLimit;
    double _dSetPoint;
    double _dNextSetPoint;
    double _dDesiredSetPoint;
    double _dMaxStep;
    double _dSubstepStepSize;
    int _iSubsteps;
    void vUpdateProcessValue(double PV);
    void vRememberPriorValues();
    bool bAntiWindup(double Error, double ControlValue, double dt);
    double dCalculateNextSetPoint(double ActualNextSetPoint, double ProcessValue, double DesiredSetPoint, int Substeps, double SubstepSize);
    double dSetPointSubstep(double ActualSetPoint, double NextSetPoint, double SubstepStepSize);
    double dSetPointRamp(double DesiredSetPoint, double ProcessValue, double ActualSetPoint, double SubstepStepSize, double ActualNextSetPoint, int Substeps);
    double dCalculateError(double SetPoint, double ProcessValue);
    double dDerivativePart(double Derrivative);
    double dErrorIntegration(double Error, double dt);
    double dIntegralPart(double ErrorIntegral);
    double dProportionalPart(double Error);
    double dBiasPart();
    double dLimiter(double Value);
    double dAverageFilter(double ValueNow, double ValueBack1, double ValueBack2);
    double dCalculateDerivative(double Actual, double Previous,double dt);
public:
    PID();
    PID(double Kp, double Kd, double Ki, double LL, double HL);
    void vSetKp(double Kp);
    void vSetKd(double Kd);
    void vSetKi(double Ki);
    void vSetLowLimit(double LL);
    void vSetHighLimit(double HL);
    void vSetSP(double SP);
    void vSetBias(double Bias);
    void vSetSPAcceleration(double SubstepSize, int Substeps);
    void vResetError();
    void vResetSetPoints(double SP);
    double dGetControlValue();
    double dGetError();
    double dGetDesiredSetPoint();
    double Update(double PV, double dt);
    std::string cPrintDebugInfo();
};