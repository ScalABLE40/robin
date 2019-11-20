#include "robin/robin.h"
Robin::Robin(std::string name, size_t size)
  : name_(name), semaphore_(Semaphore(name_)), shared_memory_(SharedMemory(name_, size))
{ }
// opens shared memory and semaphore
void Robin::open()
{
  if (isOpen())
  {
    printf("Robin: opening failed. Bridge '%s' is already open.", name_.c_str());
    throw 2;
  }
  semaphore_.open();
  shared_memory_.open();
}
// closes shared memory and semaphore
void Robin::close()
{
  if (!isOpen())
  {
    printf("Robin: closing failed. Bridge '%s' is not open.", name_.c_str());
    throw 2;
  }
  //TODO? wait for semaphore?
  shared_memory_.close();
  semaphore_.close();
}
// checks if robin bridge is open
bool Robin::isOpen()
{
  return semaphore_.isOpen() && shared_memory_.isOpen();
}
