/*
 * Handles opening and closing of shared memory.
 */
#ifndef ROBIN_SHARED_MEMORY_H
#define ROBIN_SHARED_MEMORY_H
#include <cstdio>     // for printf()
#include <cstring>    // for strerror()
#include <fcntl.h>    // for oflag constants
#include <string>     // for std::string
#include <sys/mman.h> // for shm_*()
#include <unistd.h>
class SharedMemory
{
  std::string name_;
  size_t size_;
public:
  void *ptr_ = NULL;
  SharedMemory(std::string name, size_t size);
  void open();
  bool isOpen();
  void close();
  ~SharedMemory();
};
#endif
