#include "robin/robin.h"
#include <ros/ros.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robin");
  Robin msg_in("msg_in");
  Robin msg_out("msg_out");
  ros::Rate read_rate(10);
  while (ros::ok())
  {
    msg_in.read();
    ros::spinOnce();
    read_rate.sleep();
  }
  return 0;
}
