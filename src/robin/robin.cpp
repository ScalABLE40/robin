#include "robin/robin.h"
template <typename T1, typename T2>
Robin<T1, T2>::Robin(std::string name)
  : name_(name), shared_memory_(SharedMemory<T1>(name_))
{ }
template <typename T1, typename T2>
void Robin<T1, T2>::open()
{
  if (isOpen())
  {
    ROS_ERROR("Opening failed. Bridge '%s' is already open.", name_.c_str());
    throw 2;
  }
  shared_memory_.open();
}
template <typename T1, typename T2>
void Robin<T1, T2>::close()
{
  if (!isOpen())
  {
    ROS_ERROR("Closing failed. Bridge '%s' is not open.", name_.c_str());
    throw 2;
  }
  nh_.shutdown();
  shared_memory_.close();
}
template <typename T1, typename T2>
bool Robin<T1, T2>::isOpen()
{
  return shared_memory_.isOpen();
}
template <typename T1, typename T2>
Robin<T1, T2>::~Robin()
{
  if (isOpen())
  {
    close();
  }
}
