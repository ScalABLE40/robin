#include "robin/shared_memory.h"
SharedMemory::SharedMemory(std::string name, size_t size)
  : name_(name), size_(size)
{ }
// opens, truncates and maps shared memory
void SharedMemory::open()
{
  if (isOpen())
  {
    printf("SharedMemory: shared memory '%s' is already open.", name_.c_str());
    throw 2;
  }
  // open
  errno = 0;
  int fd = shm_open(name_.c_str(), O_CREAT | O_RDWR, 00700);  // open or create; 00700=700 like chmod
  // int fd = shm_open(name_.c_str(), O_RDWR, 00700);  // open, don't create
  if (fd == -1)
  {
    printf("SharedMemory: failed to open file descriptor '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  // truncate
  errno = 0;
  if (ftruncate(fd, size_) == -1)
  {
    printf("SharedMemory: failed to truncate shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  // map
  errno = 0;
  ptr_ = mmap(0, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (ptr_ == MAP_FAILED)
  {
    printf("SharedMemory: failed to map shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  errno = 0;
  if (::close(fd) == -1)  //TODO? use shm_close
  {
    printf("SharedMemory: failed to close file descriptor '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
}
// checks if shared memory is open
bool SharedMemory::isOpen()
{
  return !(ptr_ == NULL || ptr_ == MAP_FAILED);
}
// unmaps and unlinks shared memory
void SharedMemory::close()
{
  if (!isOpen())
  {
    printf("SharedMemory: shared memory '%s' is not open.", name_.c_str());
    throw 2;
  }
  // unmap
  errno = 0;
  if (munmap(ptr_, size_) == -1)
  {
    printf("SharedMemory: failed to unmap shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    // throw 1;
  }
  // unlink
  errno = 0;
  if (shm_unlink(name_.c_str()) == -1 && errno != 2)  // errno 2: not found (possibly already unlinked) 
  {
    printf("SharedMemory: failed to unlink shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ptr_ = NULL;  //NEEDED?
}
// unmaps and unlinks shared memory if open
SharedMemory::~SharedMemory()
{
  if (isOpen())
  {
    close();
  }
}
