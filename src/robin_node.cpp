#include "ros/ros.h"
#include "robin/robin_publisher.h"
#include "robin/robin_subscriber.h"
#include "robin/structs.h"
#include "robin/TestStruct_arr.h"
#include "robin/TestStruct_foo.h"
#include "std_msgs/Float64.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robin");
  ros::NodeHandle nh;
  RobinPublisher<double, std_msgs::Float64> double_to_ros(nh, "double_to_ros");
  RobinPublisher<TestStruct_foo, robin::TestStruct_foo> struct_to_ros_1(nh, "struct_to_ros_1");
  RobinPublisher<TestStruct_arr, robin::TestStruct_arr> struct_to_ros_arr(nh, "struct_to_ros_arr");//, true, 0);
  RobinSubscriber<double, std_msgs::Float64> double_to_codesys(nh, "double_to_codesys");
  RobinSubscriber<TestStruct_foo, robin::TestStruct_foo> struct_to_codesys_1(nh, "struct_to_codesys_1");
  RobinSubscriber<TestStruct_arr, robin::TestStruct_arr> struct_to_codesys_arr(nh, "struct_to_codesys_arr");
  ros::spin();
  return 0;
}
