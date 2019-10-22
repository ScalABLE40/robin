#include "robin/shared_memory.h"
template <typename T1>
SharedMemory<T1>::SharedMemory(std::string name) : name_(name), semaphore_(Semaphore(name_)) { }
// opens, truncates and maps shared memory
template <typename T1>
void SharedMemory<T1>::open()
{
  if (isOpen())
  {
    ROS_ERROR("Shared memory '%s' is already open.", name_.c_str());
    throw 2;
  }
  semaphore_.open();
  // open
  errno = 0;
  // int fd = shm_open(name_.c_str(), O_CREAT | O_RDWR, 0700);
  int fd = shm_open(name_.c_str(), O_RDWR, 0700);
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
  shm_ptr_ = (T1 *)mmap(0, sizeof(*shm_ptr_), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
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
// checks if shared memory is open
template <typename T1>
bool SharedMemory<T1>::isOpen()
{
  return (semaphore_.isOpen() && !(shm_ptr_ == NULL || shm_ptr_ == MAP_FAILED));
}
// reads data from shared memory
template <typename T1>
void SharedMemory<T1>::read(T1 *data_ptr)
{
  if (!isOpen())
  {
    ROS_ERROR("Shared memory '%s' is not open.", name_.c_str());
    throw 2;
  }
  ROS_DEBUG("Shared memory '%s' read.", name_.c_str());
  semaphore_.wait();
  memcpy(data_ptr, shm_ptr_, sizeof(*shm_ptr_));
  semaphore_.post();
}
// writes data to shared memory
template <typename T1>
void SharedMemory<T1>::write(T1 *data_ptr)
{
  if (!isOpen())
  {
    ROS_ERROR("Shared memory '%s' is not open.", name_.c_str());
    throw 2;
  }
  ROS_DEBUG("Shared memory '%s' written.", name_.c_str());
  semaphore_.wait();
  memcpy(shm_ptr_, data_ptr, sizeof(*shm_ptr_));
  semaphore_.post();
}
// unmaps and unlinks shared memory
template <typename T1>
void SharedMemory<T1>::close()
{
  if (!isOpen())
  {
    ROS_ERROR("Shared memory '%s' is not open.", name_.c_str());
    throw 2;
  }
  //TODO? wait for semaphore?
  // unmap
  errno = 0;
  if (munmap(shm_ptr_, sizeof(*shm_ptr_)) == -1)
  {
    ROS_ERROR("Failed to unmap shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("Shared memory '%s' unmaped.", name_.c_str());
  // // unlink
  // errno = 0;
  // if (shm_unlink(name_.c_str()) == -1 && errno != 2)  // errno 2: not found (possibly already unlinked) 
  // {
  //   ROS_ERROR("Failed to unlink shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
  //   throw 1;
  // }
  // ROS_DEBUG("Shared memory '%s' unlinked.", name_.c_str());
  shm_ptr_ = NULL;  // NEEDED?
  semaphore_.close();
}
// unmaps and unlinks shared memory if open
template <typename T1>
SharedMemory<T1>::~SharedMemory()
{
  if (isOpen())
  {
    close();
  }
}


