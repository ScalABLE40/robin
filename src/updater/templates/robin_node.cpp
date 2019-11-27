#include "ros/ros.h"
#include "robin/robin_publisher.h"
#include "robin/robin_subscriber.h"
#include "robin/structs.h"{}
int main(int argc, char **argv)
{{
  ros::init(argc, argv, "robin");
  ros::NodeHandle nh;{}
  ros::spin();
  return 0;
}}
