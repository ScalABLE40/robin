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
void RobinPublisher<T1, T2>::open()
{
  open(DEF_READ_RATE);
}
template <typename T1, typename T2>
void RobinPublisher<T1, T2>::open(int read_rate)
{
  Robin<T1, T2>::open();
  publisher_ = this->nh_.template advertise<T2>(this->name_, this->QUEUE_SIZE, LATCH);
  if (read_rate > 0)
  {
    read_thread_ = new std::thread(&RobinPublisher<T1, T2>::publishLoop, this, read_rate);
  }
}
template <typename T1, typename T2>
void RobinPublisher<T1, T2>::publishLoop(int rate)
{
  ros::Rate ros_rate(rate);
  while (!closing_)
  {
    publish();
    ros_rate.sleep();
  }
}
template <typename T1, typename T2>
void RobinPublisher<T1, T2>::publish()
{
  if (!this->isOpen())
  {
    ROS_ERROR("Read failed. Bridge '%s' is not open.", this->name_.c_str());
    throw 2;
  }
  // printf("Waiting for semaphore...");
  this->semaphore_.wait();
  // printf(" Done.\nReading shm...");
  this->shared_memory_.read(&msg_);
  // printf(" Done.\nPosting semaphore...");
  this->semaphore_.post();
  // printf(" Done.\nPublishing message...");
  publisher_.publish(msg_);
  // printf(" Done.\n");
}
template <typename T1, typename T2>
void RobinPublisher<T1, T2>::close()
{
  if (read_thread_ != NULL)
  {
    closing_ = true;
    read_thread_->join();
    read_thread_ = NULL;
    closing_ = false;
  }
}
template <typename T1, typename T2>
RobinPublisher<T1, T2>::~RobinPublisher()
{
  if (this->isOpen())
  {
    this->close();
  }
}
