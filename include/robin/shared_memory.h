#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H
#include <fcntl.h>    // for O_* constants
#include <ros/ros.h>
#include <string>     // for std::string
#include <sys/mman.h> // for shm_* functions
const bool WRITE = true;
const bool READ = false;
template <typename T1>
class SharedMemory
{
  std::string name_;
  T1 *shm_ptr_ = NULL;
  bool mode_ = READ;
public:
  SharedMemory(std::string name);
  bool isOpen();
  void open(bool mode=READ);
  void write(T1 *data_ptr);
  T1 *read();
  void close();
  ~SharedMemory();
};
#endif
