#include "robin/robin.h"
#include <ros/ros.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robin");
  Robin move_conveyor("move_conveyor", WRITE);
  Robin wait_conveyor("conveyor_finished", READ);
  ros::Rate read_rate(10);
  while (ros::ok())
  {
    wait_conveyor.read();
    ros::spinOnce();
    read_rate.sleep();
  }
  return 0;
}
