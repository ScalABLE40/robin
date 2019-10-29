#include "robin/robin_publisher.h"
template <typename T1, typename T2>
RobinPublisher<T1, T2>::RobinPublisher(std::string name, bool open, int read_rate)
  : Robin<T1, T2>::Robin(name)
{
  if (open)
  {
    this->open(read_rate);
  }
}
template <typename T1, typename T2>
void RobinPublisher<T1, T2>::open(int read_rate)
{
  Robin<T1, T2>::open();
  pub_ = this->nh_.template advertise<T2>(this->name_, this->queue_size_, latch_);
  if (read_rate > 0)
  {
    read_thread_ = new std::thread(&RobinPublisher<T1, T2>::readLoop, this, read_rate);
  }
}
template <typename T1, typename T2>
void RobinPublisher<T1, T2>::open()
{
  this->open(def_read_rate_);
}
template <typename T1, typename T2>
void RobinPublisher<T1, T2>::readLoop(int rate)  //TODO fix check condition: eg add 'closing_' flag to Robin base class
{
  ros::Rate ros_rate(rate);
  while (this->isOpen() and !closing_)
  {
    read();
    ros_rate.sleep();
  }
}
template <typename T1, typename T2>
void RobinPublisher<T1, T2>::read()
{
  if (!this->isOpen())
  {
    ROS_ERROR("Read failed. Bridge '%s' is not open.", this->name_.c_str());
    throw 2;
  }
  this->shared_memory_.read((T1 *)&msg_);
  pub_.publish(msg_);
  ROS_DEBUG("Shared memory read. Message sent.");
}
template <typename T1, typename T2>
void RobinPublisher<T1, T2>::close()
{
  closing_ = true;
  read_thread_->join();
  closing_ = false;
}
template <typename T1, typename T2>
RobinPublisher<T1, T2>::~RobinPublisher()
{
  if (this->isOpen())
  {
    this->close();
  }
}
