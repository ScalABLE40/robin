/*
 * ROBIN: A ROS-CODESYS shared memory bridge
 */
#include <robin/robin.h>
#include <ros/ros.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robin");
  ros::NodeHandle nh;
  Robin robin();
  while (ros::ok())
  {
    ros::spin();
  }
  return 0;
}
