/*
 * Handles opening, closing, waiting and posting of named semaphore.
 */
#ifndef SEMAPHORE_H
#define SEMAPHORE_H
#include <cstdio>       // for printf()
#include <cstring>      // for strerror()
#include <fcntl.h>      // for O_* constants
#include <semaphore.h>  // for sem_*()
#include <string>       // for std::string
class Semaphore
{
  std::string name_;
  sem_t *semaphore_ptr_ = NULL;
public:
  Semaphore(std::string name);
  void open();
  bool isOpen();
  void wait();
  void post();
  void close();
  ~Semaphore();
};
#endif
