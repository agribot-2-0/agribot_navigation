class PID
{
private:
    double _Kp;
    double _Kd;
    double _Ki;
    double _Bias;
    double _Error;
    double _ErrorPrior;
    double _DerPrior;
    double _DerPrevious;
    double _ControlValue;
    double _SetPoint;
    double _ErrorSum;
    double _LowLimit;
    double _HighLimit;

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
    void vResetError();
    double dGetCV();
    double dGetError();
    double Update(double PV, double dt, bool isFront);
};