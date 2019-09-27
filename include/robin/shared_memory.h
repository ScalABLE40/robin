#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H
#include <fcntl.h>      // for O_* constants
#include <ros/ros.h>
#include <sys/mman.h>   // for shm_* functions
class SharedMemory
{
  std::string name_;
  double *shm_ptr_ = NULL;
  bool write_ = false;
  uint32_t queue_size_ = 100;
  bool latch_ = true;
public:
  SharedMemory(std::string name);
  bool isOpen();
  void open(bool write=false);
  void write(double data);
  double read();
  void close();
  ~SharedMemory();
};
#endif
