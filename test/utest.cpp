#include <agribot_navigation/PID.h>
#include <gtest/gtest.h>
#include <cmath>
#define dt 0.1
#define RAD_TO_DEG (180/M_PI)
#define DEG_TO_RAD (M_PI/180)

TEST(RampTest, ZeroProcessValue)
{
    double kp = 0.1;
    double limit = 10;
    PID pid = PID(kp, 0.0, 0.0,
    -limit * DEG_TO_RAD,
    limit * DEG_TO_RAD);
    for (int i = 0; i < 10; i++)
    {
        pid.Update(0.0,dt);
        EXPECT_DOUBLE_EQ(
            0.0,
            pid.dGetControlValue()*RAD_TO_DEG);
    }
}

TEST(RampTest, RisingProcessValue)
{
    double kp = 0.1;
    double limit = 10;
    PID pid = PID(kp, 0.0, 0.0,
    -limit * DEG_TO_RAD,
    limit * DEG_TO_RAD);
    pid.vSetSP(0.0);
    for (double i = 0; i < limit / kp; i+=0.1)
    {
        pid.Update(i*DEG_TO_RAD,dt);
        EXPECT_DOUBLE_EQ(
            -kp * i,
            pid.dGetControlValue()*RAD_TO_DEG);
    }
}

TEST(RampTest, SetPointRampStop)
{
    double kp = 0.1;
    double limit = 10;
    double substepsize = 0.1;
    int substeps = 10;
    PID pid = PID(kp, 0.0, 0.0,
    -limit * DEG_TO_RAD,
    limit * DEG_TO_RAD);
    pid.vSetSP(10.0 * DEG_TO_RAD);
    pid.vSetSPAcceleration(substepsize * DEG_TO_RAD, substeps);
    for (int i = 0; i < 1000; i++)
    {
        pid.Update(0.0,dt);  
    }
    EXPECT_DOUBLE_EQ(
        substepsize * substeps * kp,
        pid.dGetControlValue() * RAD_TO_DEG);
}

TEST(RampTest, SetPointRampContinue)
{
    double kp = 0.1;
    double limit = 10;
    double substepsize = 0.1;
    int substeps = 10;
    PID pid = PID(kp, 0.0, 0.0,
    -limit * DEG_TO_RAD,
    limit * DEG_TO_RAD);
    pid.vSetSP(10.0 * DEG_TO_RAD);
    pid.vSetSPAcceleration(substepsize * DEG_TO_RAD, substeps);
    for (int i = 0; i < 1000; i++)
    {
        pid.Update(0.0,dt);  
    }
    EXPECT_DOUBLE_EQ(
        substepsize * substeps * kp,
        pid.dGetControlValue() * RAD_TO_DEG);

    double newPV = 0.1;
    for (int i = 0; i < 100; i++)
    {
        pid.Update(newPV * DEG_TO_RAD,dt);  
    }
    EXPECT_DOUBLE_EQ(
        (2*substepsize * substeps - newPV) * kp,
        pid.dGetControlValue() * RAD_TO_DEG);
}

TEST(RampTest, SetPointSaturation)
{
    double kp = 0.1;
    double limit = 10;
    double substepsize = 0.1;
    int substeps = 10;
    PID pid = PID(kp, 0.0, 0.0,
    -limit * DEG_TO_RAD,
    limit * DEG_TO_RAD);
    pid.vSetSP(10.0 * DEG_TO_RAD);
    pid.vSetSPAcceleration(substepsize * DEG_TO_RAD, substeps);
    for (double newPV = 0.0; newPV <= 10.0; newPV+=1.0)
    {
        for (int i = 0; i < 100; i++)
        {
            pid.Update(newPV * DEG_TO_RAD,dt);  
        }
    }
    EXPECT_DOUBLE_EQ(
        0.0,
        pid.dGetControlValue() * RAD_TO_DEG);
}

TEST(RampTest, SetPointChangeSign)
{
    double kp = 0.1;
    double limit = 10;
    double substepsize = 0.1;
    int substeps = 10;
    PID pid = PID(kp, 0.0, 0.0,
    -limit * DEG_TO_RAD,
    limit * DEG_TO_RAD);
    pid.vSetSP(10.0 * DEG_TO_RAD);
    pid.vSetSPAcceleration(substepsize * DEG_TO_RAD, substeps);
    for (int i = 0; i < 1000; i++)
    {
        pid.Update(0.0,dt);  
    }
    EXPECT_DOUBLE_EQ(
        substepsize * substeps * kp,
        pid.dGetControlValue() * RAD_TO_DEG);

    pid.vSetSP(-10.0 * DEG_TO_RAD);
    for (int i = 0; i < 1000; i++)
    {
        pid.Update(0.0,dt);  
    }
    EXPECT_DOUBLE_EQ(
        -substepsize * substeps * kp,
        pid.dGetControlValue() * RAD_TO_DEG);

}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}