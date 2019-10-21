#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H
#include <fcntl.h>    // for O_* constants
#include <ros/ros.h>
#include <string>     // for std::string
#include <sys/mman.h> // for shm_* functions
template <typename T1>
class SharedMemory  //TODO put semaphore inside; pass 'destination' ptr to read/write()
{
  std::string name_;
  T1 *shm_ptr_ = NULL;
public:
  SharedMemory(std::string name);
  void open();
  void write(T1 *data_ptr);
  T1 *read();
  bool isOpen();
  void close();
  ~SharedMemory();
};
#endif
