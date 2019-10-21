#ifndef ROBIN_WRITER_H
#define ROBIN_WRITER_H
#include "robin/robin.h"
#include <ros/ros.h>
template <typename T1, typename T2>
class RobinWriter : public Robin<T1, T2>
{
  ros::Subscriber sub_;
  void write(const boost::shared_ptr< T2 const>& msg);
public:
  RobinWriter(std::string name, bool open=true);
  void open();
};
#endif
