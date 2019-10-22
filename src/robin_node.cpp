#include "robin/robin_reader.h"
#include "robin/robin_writer.h"
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
  RobinReader<double, std_msgs::Float64> double_to_ros("double_to_ros");
  RobinReader<TestStruct, robin::TestStruct> struct_to_ros("struct_to_ros");
  RobinWriter<double, std_msgs::Float64> double_to_codesys("double_to_codesys");
  RobinWriter<TestStruct, robin::TestStruct> struct_to_codesys("struct_to_codesys");
  ros::spin();
  return 0;
}
