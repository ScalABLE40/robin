#include "robin/robin.h"
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
  RobinWriter<double, std_msgs::Float64> double_to_codesys("double_to_codesys");
  ros::Rate read_rate(10);
  while (ros::ok())
  {
    double_to_ros.read();
    ros::spinOnce();
    read_rate.sleep();
  }
  return 0;
}
