#include <agribot_navigation/PID.h>
#include <gtest/gtest.h>

TEST(TestPID, Zeros)
{
    PID pid;
    EXPECT_EQ(0.0, pid.dAverageFilter(0.0, 0.0, 0.0));
}

TEST(TestPID, Positives)
{
    PID pid;
    EXPECT_EQ(3.0, pid.dAverageFilter(1.0, 2.0, 6.0));
}

TEST(TestPID, PositiveNegativeZero)
{
    PID pid;
    EXPECT_EQ(0.0, pid.dAverageFilter(-1.0, 0.0, 1.0));
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}