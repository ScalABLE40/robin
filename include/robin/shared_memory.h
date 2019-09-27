#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H
#include <fcntl.h>      // for O_* constants
#include <ros/ros.h>
#include <sys/mman.h>   // for shm_* functions
const bool WRITE = true;
const bool READ = false;
class SharedMemory
{
  std::string name_;
  bool *shm_ptr_ = NULL;
  bool mode_ = READ;
public:
  SharedMemory(std::string name);
  bool isOpen();
  void open(bool mode=READ);
  void write(bool data);
  bool read();
  void close();
  ~SharedMemory();
};
#endif
