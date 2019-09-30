#ifndef ROBIN_H
#define ROBIN_H
#include "robin/semaphore.h"
#include "robin/shared_memory.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
// #include <thread>
class Robin
{
  std::string name_;
  Semaphore semaphore_;
  SharedMemory shared_memory_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  // std::thread *read_thread_;
  std_msgs::Bool msg_;
  const uint32_t queue_size_ = 100;
  const bool latch_ = true;
  // void readLoop(int rate);
  void write(const std_msgs::Bool::ConstPtr& msg);
public:
  Robin(std::string name);
  bool isOpen();
  bool isClosed();
  void read();
  void open(bool mode=READ);//, int read_rate = 10);
  void close();
  ~Robin();
};
#endif
