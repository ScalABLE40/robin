#include "robin/robin.h"
template <typename T1, typename T2>
Robin<T1, T2>::Robin(std::string name, bool mode, bool open):
  name_(name), semaphore_(Semaphore(name_)), shared_memory_(SharedMemory<T1>(name_))
{
  if (open)
  {
    this->open(mode);
  }
}
template <typename T1, typename T2>
bool Robin<T1, T2>::isOpen()
{
  if (semaphore_.isOpen() && shared_memory_.isOpen())
  {
    return true;
  }
  return false;
}
template <typename T1, typename T2>
bool Robin<T1, T2>::isClosed()
{
  if (!semaphore_.isOpen() && !shared_memory_.isOpen())
  {
    return true;
  }
  return false;
}
template <typename T1, typename T2>
void Robin<T1, T2>::open(bool mode)//, int read_rate)
{
  if (!isClosed())
  {
    ROS_ERROR("Robin '%s' is already open.", name_.c_str());
    throw 2;
  }
  semaphore_.open();
  shared_memory_.open(mode);
  if (mode == READ)
  {
    pub_ = nh_.advertise<T2>(name_, queue_size_);
    // if (read_rate > 0)
    // {
    //   read_thread_ = new std::thread(&Robin::readLoop, this, read_rate);
    // }
  }
  else if (mode == WRITE)
  {
    sub_ = nh_.subscribe<T2>(name_, queue_size_, &Robin::write, this);
  }
}
// void Robin::readLoop(int rate)
// {
//   ros::Rate ros_rate(rate);
//   while (isOpen())
//   {
//     read();
//     ros_rate.sleep();
//   }
// }
template <typename T1, typename T2>
void Robin<T1, T2>::read()
{
  if (!isOpen())
  {
    ROS_ERROR("Robin '%s' is not open.", name_.c_str());
    throw 2;
  }
  semaphore_.wait();
  memcpy(&msg_, shared_memory_.read(), sizeof(msg_));
  semaphore_.post();
  pub_.publish(msg_);
}
template <typename T1, typename T2>
void Robin<T1, T2>::write(const boost::shared_ptr< T2 const>& msg)  //TODO check parameter declaration
{
  if (!isOpen())
  {
    ROS_ERROR("Robin '%s' is not open.", name_.c_str());
    throw 2;
  }
  semaphore_.wait();
  shared_memory_.write((T1 *)msg.get());  // Must pass var istself? Can shm class access it through pointer?
  semaphore_.post();
}
template <typename T1, typename T2>
void Robin<T1, T2>::close()
{
  if (!isOpen())
  {
    ROS_ERROR("Robin '%s' is not open.", name_.c_str());
    throw 2;
  }
  nh_.shutdown();
  shared_memory_.close();
  semaphore_.close();
}
template <typename T1, typename T2>
Robin<T1, T2>::~Robin()
{
  if (isOpen())
  {
    close();
  }
  // read_thread_->join();
}
