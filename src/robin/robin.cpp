#include "robin/robin.h"
Robin::Robin(std::string name):
  name_(name), semaphore_(new Semaphore(name_))
{
  ROS_DEBUG("Hello, world!");
}
Robin::~Robin(void)
{
  delete semaphore_;
}
