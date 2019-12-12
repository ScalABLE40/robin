#include "ros/ros.h"
#include "robin_bridge/robin_inst.cpp"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robin");
  ros::NodeHandle nh;
  RobinSubscriber<double, std_msgs::Float64> double_to_codesys(nh, "double_to_codesys");
  RobinSubscriber<char[5][81], robin_bridge::StringVarLenArray> string_varlen_array_to_codesys(nh, "string_varlen_array_to_codesys");
  RobinSubscriber<TestStruct, robin_bridge::TestStruct> struct1_to_codesys(nh, "struct1_to_codesys");
  RobinSubscriber<AccelStamped[2], robin_bridge::AccelStampedArray> struct2_array_to_codesys(nh, "struct2_array_to_codesys");
  RobinPublisher<double, std_msgs::Float64> double_to_ros(nh, "double_to_ros");
  RobinPublisher<char[5][81], robin_bridge::StringVarLenArray> string_varlen_array_to_ros(nh, "string_varlen_array_to_ros");
  RobinPublisher<TestStruct, robin_bridge::TestStruct> struct1_to_ros(nh, "struct1_to_ros");
  RobinPublisher<AccelStamped[2], robin_bridge::AccelStampedArray> struct2_array_to_ros(nh, "struct2_array_to_ros");
  ros::spin();
  return 0;
}
