#include "robin/robin_publisher.h"  //TODO try without
#include "robin/robin_subscriber.h"  //TODO try without
#include "robin/structs.h"
#include "robin/TestStruct.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
int main(int argc, char **argv)
{
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //  ros::console::notifyLoggerLevelsChanged();
  // }
  ros::init(argc, argv, "robin");
  RobinPublisher<double, std_msgs::Float64> double_to_ros("double_to_ros");
  RobinPublisher<TestStruct, robin::TestStruct> struct_to_ros("struct_to_ros");
  RobinSubscriber<double, std_msgs::Float64> double_to_codesys("double_to_codesys");
  RobinSubscriber<TestStruct, robin::TestStruct> struct_to_codesys("struct_to_codesys");
  ros::spin();
  return 0;
}
