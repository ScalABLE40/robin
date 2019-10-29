#include "ros/ros.h"
#include "robin/robin_publisher.h"
#include "robin/robin_subscriber.h"
#include "robin/structs.h"
#include "robin/TestStruct.h"
#include "std_msgs/Float64.h"
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robin");
  RobinPublisher<double, std_msgs::Float64> double_to_ros("double_to_ros");
  RobinPublisher<TestStruct, robin::TestStruct> struct_to_ros("struct_to_ros");
  RobinSubscriber<double, std_msgs::Float64> double_to_codesys("double_to_codesys");
  RobinSubscriber<TestStruct, robin::TestStruct> struct_to_codesys("struct_to_codesys");
  ros::spin();
  return 0;
}
