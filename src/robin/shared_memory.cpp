#include "robin/shared_memory.h"
template <typename T1, typename T2>
SharedMemory<T1, T2>::SharedMemory(std::string name) : name_(name), semaphore_(Semaphore(name_)) { }
// opens, truncates and maps shared memory
template <typename T1, typename T2>
void SharedMemory<T1, T2>::open()
{
  if (isOpen())
  {
    ROS_ERROR("Shared memory '%s' is already open.", name_.c_str());
    throw 2;
  }
  semaphore_.open();
  // open
  errno = 0;
  int fd = shm_open(name_.c_str(), O_CREAT | O_RDWR, 00700);  // open or create; 00700=700 like chmod
  // int fd = shm_open(name_.c_str(), O_RDWR, 00700);  // open, don't create
  if (fd == -1)
  {
    ROS_ERROR("Failed to open file descriptor '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  // truncate
  errno = 0;
  if (ftruncate(fd, sizeof(*shm_ptr_)) == -1)
  {
    ROS_ERROR("Failed to truncate shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  // map
  errno = 0;
  shm_ptr_ = (T1 *)mmap(0, sizeof(*shm_ptr_), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (shm_ptr_ == MAP_FAILED)
  {
    ROS_ERROR("Failed to map shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  errno = 0;
  if (::close(fd) == -1)  //TODO? use shm_close
  {
    ROS_ERROR("Failed to close file descriptor '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
}
// checks if shared memory is open
template <typename T1, typename T2>
bool SharedMemory<T1, T2>::isOpen()
{
  return (semaphore_.isOpen() && !(shm_ptr_ == NULL || shm_ptr_ == MAP_FAILED));
}
// reads data from shared memory
template <typename T1, typename T2>
void SharedMemory<T1, T2>::read(T2 *msg_ptr)
{
  if (!isOpen())
  {
    ROS_ERROR("Shared memory '%s' is not open.", name_.c_str());
    throw 2;
  }
  semaphore_.wait();
  memcpy(msg_ptr, shm_ptr_, sizeof(*shm_ptr_));
  semaphore_.post();
}
// writes data to shared memory
template <typename T1, typename T2>
void SharedMemory<T1, T2>::write(T2 const *msg_ptr)
{
  if (!isOpen())
  {
    ROS_ERROR("Shared memory '%s' is not open.", name_.c_str());
    throw 2;
  }
  semaphore_.wait();
  memcpy(shm_ptr_, msg_ptr, sizeof(*shm_ptr_));
  semaphore_.post();
}
// unmaps and unlinks shared memory
template <typename T1, typename T2>
void SharedMemory<T1, T2>::close()
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
    // throw 1;
  }
  // unlink
  errno = 0;
  if (shm_unlink(name_.c_str()) == -1 && errno != 2)  // errno 2: not found (possibly already unlinked) 
  {
    ROS_ERROR("Failed to unlink shared memory '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  shm_ptr_ = NULL;  // NEEDED?
  semaphore_.close();
}
// unmaps and unlinks shared memory if open
template <typename T1, typename T2>
SharedMemory<T1, T2>::~SharedMemory()
{
  if (isOpen())
  {
    close();
  }
}
