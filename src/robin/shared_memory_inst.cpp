#include "shared_memory.cpp"
#include "robin/structs.h"
#include "robin/TestStruct_foo.h"
#include "std_msgs/Float64.h"
template<> void SharedMemory<double, std_msgs::Float64>::read(std_msgs::Float64 *msg_ptr)
{
  if (!isOpen())
  {
    ROS_ERROR("Shared memory '%s' is not open.", name_.c_str());
    throw 2;
  }
  semaphore_.wait();
  msg_ptr->data = *shm_ptr_;
  printf("read\n");
  semaphore_.post();
};
template<> void SharedMemory<double, std_msgs::Float64>::write(std_msgs::Float64 const *msg_ptr)
{
  if (!isOpen())
  {
    ROS_ERROR("Shared memory '%s' is not open.", name_.c_str());
    throw 2;
  }
  semaphore_.wait();
  *shm_ptr_ = msg_ptr->data;
  printf("written\n");
  semaphore_.post();
};
template class SharedMemory<double, std_msgs::Float64>;
template class SharedMemory<TestStruct_foo, robin::TestStruct_foo>;
