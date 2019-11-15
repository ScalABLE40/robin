#ifndef ROBIN_H
#define ROBIN_H
#include "robin/shared_memory.h"
#include <ros/ros.h>
#include <string>  // for std::string
template <typename T1, typename T2>
class Robin
{
protected:
  const static uint32_t QUEUE_SIZE = 100;  //TODO? pass as argument in constructor?
  std::string name_;
  SharedMemory<T1, T2> shared_memory_;
  ros::NodeHandle nh_;
public:
  Robin(std::string name);
  virtual void open() = 0;
  bool isOpen();
  virtual void close();
  virtual ~Robin();
};
#endif
