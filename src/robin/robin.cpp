#include "robin/robin.h"
Robin::Robin(std::string name, bool mode, bool open):
  name_(name), semaphore_(Semaphore(name_)), shared_memory_(SharedMemory(name_))
{
  if (open)
  {
    this->open(mode);
  }
}
bool Robin::isOpen()
{
  if (semaphore_.isOpen() && shared_memory_.isOpen())
  {
    return true;
  }
  return false;
}
bool Robin::isClosed()
{
  if (!semaphore_.isOpen() && !shared_memory_.isOpen())
  {
    return true;
  }
  return false;
}
void Robin::open(bool mode)//, int read_rate)
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
    pub_ = nh_.advertise<std_msgs::Bool>(name_, queue_size_);
    // if (read_rate > 0)
    // {
    //   read_thread_ = new std::thread(&Robin::readLoop, this, read_rate);
    // }
  }
  else if (mode == WRITE)
  {
    sub_ = nh_.subscribe<std_msgs::Bool>(name_, queue_size_, &Robin::write, this);
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
void Robin::read()
{
  if (!isOpen())
  {
    ROS_ERROR("Robin '%s' is not open.", name_.c_str());
    throw 2;
  }
  semaphore_.wait();
  msg_.data = shared_memory_.read();
  semaphore_.post();
  pub_.publish(msg_);
}
void Robin::write(const std_msgs::Bool::ConstPtr& msg)
{
  if (!isOpen())
  {
    ROS_ERROR("Robin '%s' is not open.", name_.c_str());
    throw 2;
  }
  semaphore_.wait();
  shared_memory_.write((*msg).data);
  semaphore_.post();
}
void Robin::close()
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
Robin::~Robin()
{
  if (isOpen())
  {
    close();
  }
  // read_thread_->join();
}
