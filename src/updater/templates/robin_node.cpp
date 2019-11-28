#include "ros/ros.h"
#include "robin/robin_inst.cpp"{}
int main(int argc, char **argv)
{{
  ros::init(argc, argv, "robin");
  ros::NodeHandle nh;{}
  ros::spin();
  return 0;
}}
