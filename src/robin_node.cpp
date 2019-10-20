#include "robin/robin.h"
#include <ros/ros.h>
int main(int argc, char **argv)
{
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //  ros::console::notifyLoggerLevelsChanged();
  // }
  ros::init(argc, argv, "robin");
  Robin<double, std_msgs::Float64> double_to_ros("double_to_ros", READ);
  Robin<double, std_msgs::Float64> double_to_codesys("double_to_codesys", WRITE);
  ros::Rate read_rate(10);
  while (ros::ok())
  {
    double_to_ros.read();
    ros::spinOnce();
    read_rate.sleep();
  }
  return 0;
}
