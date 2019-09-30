#include "robin/shared_memory.h"
SharedMemory::SharedMemory(std::string name) : name_(name) { }
// checks if shared memory is open
bool SharedMemory::isOpen()
{
  return !(shm_ptr_ == NULL || shm_ptr_ == MAP_FAILED);
}
// opens, truncates and maps shared memory
void SharedMemory::open(bool mode)
{
  if (isOpen())
  {
    ROS_ERROR("Shared memory '%s' is already open.", name_.c_str());
    throw 2;
  }
  mode_ = mode;
  // open
  errno = 0;
  int fd = shm_open(name_.c_str(), O_CREAT | O_RDWR, 0600 );
  if (fd == -1)
  {
    ROS_ERROR("Failed to open file descriptor '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("File descriptor '%s' opened.", name_.c_str());
  // truncate
  errno = 0;
  if (ftruncate(fd, sizeof(*shm_ptr_)) == -1)  //TODO? use shm_close
  {
    ROS_ERROR("Failed to truncate shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("Shared memory '%s' truncated.", name_.c_str());
  // map
  errno = 0;
  shm_ptr_ = (bool *)mmap(0, sizeof(*shm_ptr_), (mode_ ? PROT_WRITE : PROT_READ), MAP_SHARED, fd, 0);
  if (shm_ptr_ == MAP_FAILED)
  {
    ROS_ERROR("Failed to map shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("Shared memory '%s' mapped.", name_.c_str());
  errno = 0;
  if (::close(fd) == -1)  //TODO? use shm_close
  {
    ROS_ERROR("Failed to close file descriptor '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("File descriptor '%s' closed.", name_.c_str());
}
// writes data to shared memory
void SharedMemory::write(bool data)
{
  if (!isOpen())
  {
    ROS_ERROR("Shared memory '%s' is not open.", name_.c_str());
    throw 2;
  }
  else if (!mode_)
  {
    ROS_ERROR("Shared memory '%s' is open in read mode.", name_.c_str());
    throw 2;
  }
  ROS_DEBUG("Shared memory '%s' written: %s.", name_.c_str(), data ? "true" : "false");
  *shm_ptr_ = data;
}
// reads data from shared memory
bool SharedMemory::read()
{
  if (!isOpen())
  {
    ROS_ERROR("Shared memory '%s' is not open.", name_.c_str());
    throw 2;
  }
  else if (mode_)
  {
    ROS_ERROR("Shared memory '%s' is open in write mode.", name_.c_str());
    throw 2;
  }
  ROS_DEBUG("Shared memory '%s' read: %s.", name_.c_str(), *shm_ptr_ ? "true" : "false");
  return *shm_ptr_;
}
// unmaps and unlinks shared memory
void SharedMemory::close()
{
  if (!isOpen())
  {
    ROS_ERROR("Shared memory '%s' is not open.", name_.c_str());
    throw 2;
  }
  // unmap
  errno = 0;
  if (munmap(shm_ptr_, sizeof(*shm_ptr_)) == -1)
  {
    ROS_ERROR("Failed to unmap shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("Shared memory '%s' unmaped.", name_.c_str());
  // unlink
  errno = 0;
  if (shm_unlink(name_.c_str()) == -1 && errno != 2)  // errno 2: not found (possibly already unlinked) 
  {
    ROS_ERROR("Failed to unlink shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("Shared memory '%s' unlinked.", name_.c_str());
  shm_ptr_ = NULL;  // NEEDED?
}
// unmaps and unlinks shared memory if open
SharedMemory::~SharedMemory()
{
  if (isOpen())
  {
    close();
  }
}
