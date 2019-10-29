#include "robin/robin_subscriber.h"
template <typename T1, typename T2>
RobinSubscriber<T1, T2>::RobinSubscriber(std::string name, bool open)
  : Robin<T1, T2>::Robin(name)
{
  if (open)
  {
    this->open();
  }
}
template <typename T1, typename T2>
void RobinSubscriber<T1, T2>::open()//int read_rate)
{
  Robin<T1, T2>::open();
  sub_ = this->nh_.template subscribe<T2>(this->name_, this->queue_size_, &RobinSubscriber<T1, T2>::write, this);
}
template <typename T1, typename T2>
void RobinSubscriber<T1, T2>::write(const boost::shared_ptr< T2 const>& msg)
{
  if (!this->isOpen())
  {
    ROS_ERROR("Write failed. Bridge '%s' is not open.", this->name_.c_str());
    throw 2;
  }
  this->shared_memory_.write((T1 *)msg.get());
  ROS_DEBUG("Message received. Shared memory written.");
}
