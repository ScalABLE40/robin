#ifndef SEMAPHORE_H
#define SEMAPHORE_H
#include <fcntl.h>        // for O_* constants
// #include <pthread.h>   // for sem_* functions
#include <ros/ros.h>
#include <semaphore.h>    // for sem_* functions
#include <string>         // for std::string
class Semaphore
{
  std::string name_;
  sem_t *semaphore_ = NULL;
public:
  Semaphore(std::string name);
  bool isOpen();
  void open();
  void wait();
  void post();
  void close();
  ~Semaphore();
};
#endif