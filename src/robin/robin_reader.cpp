#include "robin/robin_reader.h"
template <typename T1, typename T2>
RobinReader<T1, T2>::RobinReader(std::string name, bool open)
  : Robin<T1, T2>::Robin(name)
{
  if (open)
  {
    this->open();
  }
}
template <typename T1, typename T2>
void RobinReader<T1, T2>::open()//int read_rate)
{
  Robin<T1, T2>::open();
  pub_ = this->nh_.template advertise<T2>(this->name_, this->queue_size_, latch_);
  // if (read_rate > 0)
  // {
  //   read_thread_ = new std::thread(&RobinReader<T1, T2>::readLoop, this, read_rate);
  // }
}
// template <typename T1, typename T2>
// void RobinReader<T1, T2>::readLoop(int rate)  //TODO fix check condition: eg add 'closing_' flag to Robin base class
// {
//   ros::Rate ros_rate(rate);
//   while (isOpen())
//   {
//     read();
//     ros_rate.sleep();
//   }
// }
template <typename T1, typename T2>
void RobinReader<T1, T2>::read()
{
  if (!this->isOpen())
  {
    ROS_ERROR("Read failed. Bridge '%s' is not open.", this->name_.c_str());
    throw 2;
  }
  this->semaphore_.wait();
  memcpy(&msg_, this->shared_memory_.read(), sizeof(msg_));
  this->semaphore_.post();
  pub_.publish(msg_);
}
// template <typename T1, typename T2>
// void RobinReader<T1, T2>::close()
// {
//   closing_ = true;
//   read_thread_->join();
//   close();
// }
