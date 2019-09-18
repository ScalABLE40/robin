#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robin");
  
  ROS_INFO("Hello, world!");

  while (ros::ok())
  {
    ros::spin();
  }

  return 0;
}