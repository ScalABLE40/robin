#include "robin/semaphore.h"
// opens semaphore
Semaphore::Semaphore(std::string name):
  name_(name)
{
  errno = 0;
  semaphore_ = sem_open(name_.c_str(), O_CREAT, 0600, 1);  // 0600: r/w permission by owner
  if (semaphore_ != SEM_FAILED)
  {
    ROS_DEBUG("Semaphore '%s' opened.", name_.c_str());
  }
  else
  {
    ROS_FATAL("Failed to open semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
  }
}
// waits semaphore
void Semaphore::wait(void)
{
  errno = 0;
  int result = sem_wait(semaphore_);
  if (result == 0)
  {
    ROS_DEBUG("Semaphore '%s' waited.", name_.c_str());
  }
  else if (result == -1)
  {
    ROS_ERROR("Failed to wait semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
  }
}
// posts semaphore
void Semaphore::post(void)
{
  errno = 0;
  int result = sem_post(semaphore_);
  if (result == 0)
  {
    ROS_DEBUG("Semaphore '%s' posted.", name_.c_str());
  }
  else if (result == -1)
  {
    ROS_ERROR("Failed to post semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
  }
}
// unlinks semaphore
Semaphore::~Semaphore(void)
{
  errno = 0;
  int result = sem_close(semaphore_);
  if (result == 0)
  {
    ROS_DEBUG("Semaphore '%s' closed.", name_.c_str());
  }
  else if (result == -1)
  {
    ROS_WARN("Failed to close semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
  }
  errno = 0;
  result = sem_unlink(name_.c_str());
  if (result == 0)
  {
    ROS_DEBUG("Semaphore '%s' unlinked.", name_.c_str());
  }
  else if (result == -1)
  {
    ROS_WARN("Failed to unlink semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
  }
}
