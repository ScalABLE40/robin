#include "robin/robin_subscriber.h"
template <typename T1, typename T2>
RobinSubscriber<T1, T2>::RobinSubscriber(ros::NodeHandle &nh, std::string name, bool open)
  : nh_(nh), Robin::Robin(name, sizeof(T1))
{
  if (open)
  {
    this->open();
  }
}
template <typename T1, typename T2>
void RobinSubscriber<T1, T2>::open()
{
  Robin::open();
  shm_ptr_ = (T1 *)shared_memory_.ptr_;
  subscriber_ = nh_.subscribe<T2>(name_, QUEUE_SIZE, &RobinSubscriber<T1, T2>::subscriberCallback, this);
}
template <typename T1, typename T2>
void RobinSubscriber<T1, T2>::subscriberCallback(const boost::shared_ptr<T2 const>& msg)
{
  if (!isOpen())
  {
    ROS_ERROR("Write failed. Bridge '%s' is not open.", name_.c_str());
    throw 2;
  }
  semaphore_.wait();
  write(msg.get());
  semaphore_.post();
}
template <typename T1, typename T2>
void RobinSubscriber<T1, T2>::write(T2 const *msg_ptr)
{
  memcpy(shm_ptr_, msg_ptr, sizeof(T1));
}
// zeroes unsent elements
template <typename T1, typename T2> template <typename T>
void RobinSubscriber<T1, T2>::zeroUnsentElements(T *ptr, size_t msg_size, size_t shm_size)
{
  if (msg_size < shm_size)
  {
    memset(ptr + msg_size, 0, sizeof(T) * (shm_size - msg_size));
  }
}
template <typename T1, typename T2>
void RobinSubscriber<T1, T2>::close()
{
  subscriber_.shutdown();
  shm_ptr_ = NULL;
  Robin::close();
}
template <typename T1, typename T2>
RobinSubscriber<T1, T2>::~RobinSubscriber()
{
  if (isOpen())
  {
    close();
  }
}
