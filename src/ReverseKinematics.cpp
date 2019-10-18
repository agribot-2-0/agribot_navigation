#include <agribot_navigation/ReverseKinematics.h>

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


ReverseKinematics::ReverseKinematics(){}
    
int ReverseKinematics::configure(){}
bool ReverseKinematics::startup(){}
bool ReverseKinematics::update(x y z R P Y){
    ReverseKinematics::vCalculateReverseKinematics();
}
bool ReverseKinematics::stop(){}
bool ReverseKinematics::cleanup(){}

void ReverseKinematics::vCalculateReverseKinematics(){
    //REVERSE KINEMATICS WILL BE CALCULATED HERE
}

void ReverseKinematics::fGetCaterpilareVelocities(float* output){
    for(i=0;i++;i<4){
        output[i]=this->_pfCaterpillarVelocities[i];
    }
}
void ReverseKinematics::vSetInputLinear(float x, float y, float z){
    this->_pfInputLinear[X]=x;
    this->_pfInputLinear[Y]=y;
    this->_pfInputLinear[Z]=z;
}
void ReverseKinematics::vSetInputAngular(float x, float y, float z){
    this->_pfInputAngular[X]=x;
    this->_pfInputAngular[Y]=y;
    this->_pfInputAngular[Z]=z;
}   