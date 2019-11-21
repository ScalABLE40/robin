#include "ros/ros.h"
#include "robin/robin_publisher.h"
#include "robin/robin_subscriber.h"
#include "robin/structs.h"
#include "robin/TestStruct_arr.h"
#include "robin/TestStruct_foo.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robin");
  ros::NodeHandle nh;
  RobinPublisher<double, std_msgs::Float64> double_to_ros(nh, "double_to_ros");
  RobinPublisher<TestStruct_foo, robin::TestStruct_foo> struct_to_ros(nh, "struct_to_ros");
  RobinPublisher<char[80], std_msgs::String> string_to_ros(nh, "string_to_ros");
  // RobinPublisher<double[5], std_msgs::Float64MultiArray> double_array_to_ros(nh, "double_array_to_ros");
  // RobinPublisher<TestStruct_arr, robin::TestStruct_arr> struct_to_ros_arr(nh, "struct_to_ros_arr");
  RobinSubscriber<double, std_msgs::Float64> double_to_codesys(nh, "double_to_codesys");
  RobinSubscriber<TestStruct_foo, robin::TestStruct_foo> struct_to_codesys(nh, "struct_to_codesys");
  RobinSubscriber<char[80], std_msgs::String> string_to_codesys(nh, "string_to_codesys");
  // RobinPublisher<double[5], std_msgs::Float64MultiArray> double_array_to_codesys(nh, "double_array_to_codesys");
  // RobinSubscriber<TestStruct_arr, robin::TestStruct_arr> struct_to_codesys_arr(nh, "struct_to_codesys_arr");
  ros::spin();
  return 0;
}
