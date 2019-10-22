#include <agribot_navigation/InverseKinematics.h>

enum{
    X,
    Y,
    Z,
};
enum {
    FRONT_LINEAR,
    FRONT_ANGULAR,
    BACK_LINEAR,
    BACK_ANGULAR
};

InverseKinematics::InverseKinematics(){}
int InverseKinematics::configure(){}
bool InverseKinematics::startup(){}
bool InverseKinematics::update()
{
    InverseKinematics::vCalculateInverseKinematics();
}
bool InverseKinematics::stop(){}
bool InverseKinematics::cleanup(){}
void InverseKinematics::vCalculateInverseKinematics(){

    //REVERSE KINEMATICS WILL BE CALCULATED HERE
}
void InverseKinematics::vGetCaterpilareVelocities(float output[])
{
    for(int i=0;i++;i<4){
        output[i]=this->_pfCaterpillarVelocities[i];
    }
}
void InverseKinematics::vSetInputLinear(float x, float y, float z)
{
    this->_pfInputLinear[X]=x;
    this->_pfInputLinear[Y]=y;
    this->_pfInputLinear[Z]=z;
}
void InverseKinematics::vSetInputAngular(float x, float y, float z)
{
    this->_pfInputAngular[X]=x;
    this->_pfInputAngular[Y]=y;
    this->_pfInputAngular[Z]=z;
}   