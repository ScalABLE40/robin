#ifndef ROBIN_H
#define ROBIN_H
#include "robin/robin_reader.h"
#include "robin/robin_writer.h"
#include "robin/semaphore.h"
#include "robin/shared_memory.h"
#include <ros/ros.h>
#include <string>  // for std::string
template <typename T1, typename T2>
class Robin
{
protected:
  std::string name_;
  Semaphore semaphore_;
  SharedMemory<T1> shared_memory_;
  ros::NodeHandle nh_;
  const uint32_t queue_size_ = 100;  //TODO? pass as argument in constructor?
public:
  Robin(std::string name);
  virtual void open() = 0;//, int read_rate = 10);
  virtual void close();
  bool isOpen();
  bool isClosed();
  ~Robin();
};
#endif
