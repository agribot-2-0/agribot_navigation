/*
 * Copyright (C) 2020 Agribot. All rights reserved.
 */

#include <agribot_navigation/kinematics_control/PID.h>
#include <agribot_common/CommonFunctions/common_functions.h>
#include <gtest/gtest.h>
#include <cmath>
#define dt 0.1

TEST(RampTest, ZeroProcessValue)
{
  double kp = 0.1;
  double limit = 10;
  PID pid = PID(kp, 0.0, 0.0, -common_functions::degrees(limit), common_functions::radians(limit));
  for (int i = 0; i < 10; i++)
  {
    pid.Update(0.0, dt);
    EXPECT_DOUBLE_EQ(0.0, common_functions::radians(pid.getControlValue()));
  }
}

TEST(RampTest, RisingProcessValue)
{
  double kp = 0.1;
  double limit = 10;
  PID pid = PID(kp, 0.0, 0.0, -common_functions::radians(limit), common_functions::radians(limit));
  pid.setSp(0.0);
  for (double i = 0; i < limit / kp; i += 0.1)
  {
    pid.Update(common_functions::radians(i), dt);
    EXPECT_DOUBLE_EQ(-kp * i, common_functions::radians(pid.getControlValue()));
  }
}

TEST(RampTest, SetPointRampStop)
{
  double kp = 0.1;
  double limit = 10;
  double substepsize = 0.1;
  int substeps = 10;
  PID pid = PID(kp, 0.0, 0.0, -common_functions::radians(limit), common_functions::radians(limit));
  pid.setSp(common_functions::radians(10.0));
  pid.setSpAcceleration(common_functions::radians(substepsize), substeps);
  for (int i = 0; i < 1000; i++)
  {
    pid.Update(0.0, dt);
  }
  EXPECT_DOUBLE_EQ(substepsize * substeps * kp, common_functions::radians(pid.getControlValue()));
}

TEST(RampTest, SetPointRampContinue)
{
  double kp = 0.1;
  double limit = 10;
  double substepsize = 0.1;
  int substeps = 10;
  PID pid = PID(kp, 0.0, 0.0, -common_functions::radians(limit), common_functions::radians(limit));
  pid.setSp(common_functions::radians(10.0));
  pid.setSpAcceleration(common_functions::radians(substepsize), substeps);
  for (int i = 0; i < 1000; i++)
  {
    pid.Update(0.0, dt);
  }
  EXPECT_DOUBLE_EQ(substepsize * substeps * kp, common_functions::radians(pid.getControlValue()));

  double newPV = 0.1;
  for (int i = 0; i < 100; i++)
  {
    pid.Update(common_functions::radians(newPV), dt);
  }
  EXPECT_DOUBLE_EQ((2 * substepsize * substeps - newPV) * kp, common_functions::radians(pid.getControlValue()));
}

TEST(RampTest, SetPointSaturation)
{
  double kp = 0.1;
  double limit = 10;
  double substepsize = 0.1;
  int substeps = 10;
  PID pid = PID(kp, 0.0, 0.0, -common_functions::radians(limit), common_functions::radians(limit));
  pid.setSp(common_functions::radians(10.0));
  pid.setSpAcceleration(common_functions::radians(substepsize), substeps);
  for (double newPV = 0.0; newPV <= 10.0; newPV += 1.0)
  {
    for (int i = 0; i < 100; i++)
    {
      pid.Update(common_functions::radians(newPV), dt);
    }
  }
  EXPECT_DOUBLE_EQ(0.0, common_functions::radians(pid.getControlValue()));
}

TEST(RampTest, SetPointChangeSign)
{
  double kp = 0.1;
  double limit = 10;
  double substepsize = 0.1;
  int substeps = 10;
  PID pid = PID(kp, 0.0, 0.0, -common_functions::radians(limit), common_functions::radians(limit));
  pid.setSp(common_functions::radians(10.0));
  pid.setSpAcceleration(common_functions::radians(substepsize), substeps);
  for (int i = 0; i < 1000; i++)
  {
    pid.Update(0.0, dt);
  }
  EXPECT_DOUBLE_EQ(substepsize * substeps * kp, common_functions::radians(pid.getControlValue()));

  pid.setSp(-common_functions::radians(10.0));
  for (int i = 0; i < 1000; i++)
  {
    pid.Update(0.0, dt);
  }
  EXPECT_DOUBLE_EQ(-substepsize * substeps * kp, common_functions::radians(pid.getControlValue()));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
