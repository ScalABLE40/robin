/*
 * ROBIN: A ROS-CODESYS shared memory bridge.
 */
#include "robin/robin.h"
int main(int argc, char **argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
   ros::console::notifyLoggerLevelsChanged();
  }
  ros::init(argc, argv, "robin");
  ros::NodeHandle nh;
  Robin robin("robin");
  while (ros::ok())
  {
    ros::spin();
  }
  return 0;
}
