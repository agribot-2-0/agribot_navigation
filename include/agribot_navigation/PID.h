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
    double _dDesiredSetPoint;
    double _dSetPoint;
    double _dNextSetPoint;
    double _dErrorSum;
    double _dLowLimit;
    double _dHighLimit;
    double _dSpAcceleration;
    double _dSpAccMultiplier;
public:
    PID();
    PID(double kp, double kd, double ki, double LL, double HL);
    void vSetKp(double Kp);
    void vSetKd(double Kd);
    void vSetKi(double Ki);
    void vSetLowLimit(double SP);
    void vSetHighLimit(double SP);
    void vSetSP(double SP);
    void vSetBias(double Bias);
    void vSetSPAcceleration(double Acceleration, double Multiplier);
    void vResetError();
    double dGetCV();
    double dGetError();
    double dGetDesiredSP();
    double Update(double PV, double dt, bool isFront);
};