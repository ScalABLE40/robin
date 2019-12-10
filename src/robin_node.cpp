#include "ros/ros.h"
#include "robin/robin_inst.cpp"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robin");
  ros::NodeHandle nh;
  RobinSubscriber<double, std_msgs::Float64> double_to_codesys(nh, "double_to_codesys");
  RobinSubscriber<AccelStamped, geometry_msgs::AccelStamped> struct1_to_codesys(nh, "struct1_to_codesys");
  RobinSubscriber<TestStruct, robin::TestStruct> struct2_to_codesys(nh, "struct2_to_codesys");
  RobinPublisher<double, std_msgs::Float64> double_to_ros(nh, "double_to_ros");
  RobinPublisher<AccelStamped, geometry_msgs::AccelStamped> struct1_to_ros(nh, "struct1_to_ros");
  RobinPublisher<TestStruct, robin::TestStruct> struct2_to_ros(nh, "struct2_to_ros");
  ros::spin();
  return 0;
}
