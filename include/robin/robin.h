#ifndef ROBIN_H
#define ROBIN_H
#include "robin/semaphore.h"
#include <ros/ros.h>
class Robin
{
private:
  std::string name_;
  Semaphore *semaphore_;
public:
  Robin(std::string name);
  ~Robin(void);
};
#endif
