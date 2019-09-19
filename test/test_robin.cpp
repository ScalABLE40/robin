#include <gtest/gtest.h>
#include <ros/ros.h>
TEST(RobinTestSuite, testCase1)
{
  ROS_INFO("Hello, test world!");
  EXPECT_TRUE(true);
}
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "test_robin");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
