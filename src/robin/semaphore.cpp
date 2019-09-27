#include "robin/semaphore.h"
Semaphore::Semaphore(std::string name) : name_(name) { }
// checks if semaphore is open
bool Semaphore::isOpen()
{
  return !(semaphore_ == NULL || semaphore_ == SEM_FAILED);
}
// opens semaphore
void Semaphore::open()
{
  if (isOpen())
  {
    ROS_ERROR("Semaphore '%s' is already open.", name_.c_str());
    throw 2;
  }
  errno = 0;
  semaphore_ = sem_open(name_.c_str(), O_CREAT, 0600, 1);  // 0600: r/w permission by owner
  if (semaphore_ == SEM_FAILED)
  {
    ROS_ERROR("Failed to open semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("Semaphore '%s' opened.", name_.c_str());
}
// waits semaphore
void Semaphore::wait()
{
  if (!isOpen())
  {
    ROS_ERROR("Semaphore '%s' is not open.", name_.c_str());
    throw 2;
  }
  errno = 0;
  if (sem_wait(semaphore_) == -1)
  {
    ROS_ERROR("Failed to wait semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("Semaphore '%s' waited.", name_.c_str());
}
// posts semaphore
void Semaphore::post()
{
  if (!isOpen())
  {
    ROS_ERROR("Semaphore '%s' is not open.", name_.c_str());
    throw 2;
  }
  errno = 0;
  if (sem_post(semaphore_) == -1)
  {
    ROS_ERROR("Failed to post semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("Semaphore '%s' posted.", name_.c_str());
}
// closes and unlinks semaphore
void Semaphore::close()
{
  if (!isOpen())
  {
    ROS_ERROR("Semaphore '%s' is not open.", name_.c_str());
    throw 2;
  }
  // close
  errno = 0;
  if (sem_close(semaphore_) == -1)
  {
    ROS_ERROR("Failed to close semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("Semaphore '%s' closed.", name_.c_str());
  // unlink
  errno = 0;
  if (sem_unlink(name_.c_str()) == -1)
  {
    ROS_ERROR("Failed to unlink semaphore '%s'. errno %d: %s", name_.c_str(), errno, strerror(errno));
    throw 1;
  }
  ROS_DEBUG("Semaphore '%s' unlinked.", name_.c_str());
  semaphore_ = NULL;  // NEEDED?
}
// closes and unlinks semaphore if open
Semaphore::~Semaphore()
{
  if (isOpen())
  {
    close();
  }
}
