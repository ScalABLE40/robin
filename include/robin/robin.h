/*
 * ROBIN: A ROS-CODESYS shared memory bridge.
 */
#ifndef ROBIN_H
#define ROBIN_H
#include "robin/semaphore.h"
#include "robin/shared_memory.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <string>  // for std::string
// #include <thread>
template <typename T1, typename T2>
class Robin
{
  std::string name_;
  Semaphore semaphore_;
  SharedMemory<T1> shared_memory_;
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  // std::thread *read_thread_;
  T2 msg_;
  const uint32_t queue_size_ = 100;
  const bool latch_ = true;
  // void readLoop(int rate);
  void write(const boost::shared_ptr< T2 const>& msg);
public:
  Robin(std::string name, bool mode=READ, bool open=true);
  bool isOpen();
  bool isClosed();
  void read();
  void open(bool mode=READ);//, int read_rate = 10);
  void close();
  ~Robin();
};
#endif
