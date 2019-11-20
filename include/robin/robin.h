/*
 * Abstract class that handles SharedMemory and Semaphore objects.
 */
#ifndef ROBIN_H
#define ROBIN_H
#include <stdio.h>  // for printf()
#include <string>   // for std::string
#include "robin/semaphore.h"
#include "robin/shared_memory.h"
class Robin
{
protected:
  const static uint32_t QUEUE_SIZE = 100;  //TODO? pass as argument in constructor
  std::string name_;
  Semaphore semaphore_;
  SharedMemory shared_memory_;
  size_t shm_size_;
public:
  Robin(std::string name, size_t size);
  virtual void open() = 0;
  virtual void close() = 0;
  bool isOpen();
};
#endif
