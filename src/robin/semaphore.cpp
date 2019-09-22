#include "robin/semaphore.h"
Semaphore::Semaphore(std::string name) : name_(name) { }
// checks if semaphore is open
bool Semaphore::isOpen(void)
{
  return !(semaphore_ == NULL || semaphore_ == SEM_FAILED);
}
// opens semaphore
void Semaphore::open(void)
{
  if (isOpen())
  {
    ROS_ERROR("Semaphore '%s' is already open.", name_.c_str());
    throw 2;
  }
  errno = 0;
  semaphore_ = sem_open(name_.c_str(), O_CREAT, 0600, 1);  // 0600: r/w permission by owner
  if (semaphore_ != SEM_FAILED)
  {
    ROS_DEBUG("Semaphore '%s' opened.", name_.c_str());
  }
  else
  {
    ROS_ERROR("Failed to open semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
}
// waits semaphore
void Semaphore::wait(void)
{
  if (!isOpen())
  {
    ROS_ERROR("Semaphore '%s' is not open.", name_.c_str());
    throw 2;
  }
  errno = 0;
  int result = sem_wait(semaphore_);
  if (result == 0)
  {
    ROS_DEBUG("Semaphore '%s' waited.", name_.c_str());
  }
  else if (result == -1)
  {
    ROS_ERROR("Failed to wait semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
}
// posts semaphore
void Semaphore::post(void)
{
  if (!isOpen())
  {
    ROS_ERROR("Semaphore '%s' is not open.", name_.c_str());
    throw 2;
  }
  errno = 0;
  int result = sem_post(semaphore_);
  if (result == 0)
  {
    ROS_DEBUG("Semaphore '%s' posted.", name_.c_str());
  }
  else if (result == -1)
  {
    ROS_ERROR("Failed to post semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
}
// closes semaphore
void Semaphore::close(void)
{
  if (!isOpen())
  {
    ROS_ERROR("Semaphore '%s' is not open.", name_.c_str());
    throw 2;
  }
  errno = 0;
  int result = sem_close(semaphore_);
  if (result == 0)
  {
    ROS_DEBUG("Semaphore '%s' closed.", name_.c_str());
    semaphore_ = NULL;  // NEEDED?
  }
  else if (result == -1)
  {
    ROS_ERROR("Failed to close semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
}
// closes if open and unlinks semaphore
Semaphore::~Semaphore(void)
{
  if (isOpen())
  {
    close();
  }
  // unlink
  errno = 0;
  int result = sem_unlink(name_.c_str());
  if (result == 0)
  {
    ROS_DEBUG("Semaphore '%s' unlinked.", name_.c_str());
  }
  else if (result == -1)
  {
    ROS_ERROR("Failed to unlink semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
}
