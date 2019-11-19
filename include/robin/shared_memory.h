#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H
#include <fcntl.h>    // for oflag constants
#include "robin/semaphore.h"
#include <ros/ros.h>
#include <string>     // for std::string
#include <sys/mman.h> // for shm_* functions
template <typename T1, typename T2>
class SharedMemory
{
  std::string name_;
  // Semaphore semaphore_;
  T1 *shm_ptr_ = NULL;
  size_t max_size_;
  size_t arr_size_;
public:
  SharedMemory(std::string name);
  void open();
  bool isOpen();
  void read(T2 *msg_ptr);
  void write(T2 const *msg_ptr);
  void close();
  ~SharedMemory();
};
#endif
