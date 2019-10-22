#include "robin/robin_writer.h"
template <typename T1, typename T2>
RobinWriter<T1, T2>::RobinWriter(std::string name, bool open)
  : Robin<T1, T2>::Robin(name)
{
  if (open)
  {
    this->open();
  }
}
template <typename T1, typename T2>
void RobinWriter<T1, T2>::open()//int read_rate)
{
  Robin<T1, T2>::open();
  sub_ = this->nh_.template subscribe<T2>(this->name_, this->queue_size_, &RobinWriter<T1, T2>::write, this);
}
template <typename T1, typename T2>
void RobinWriter<T1, T2>::write(const boost::shared_ptr< T2 const>& msg)  //TODO check parameter declaration
{
  if (!this->isOpen())
  {
    ROS_ERROR("Write failed. Bridge '%s' is not open.", this->name_.c_str());
    throw 2;
  }
  this->shared_memory_.write((T1 *)msg.get());
}
