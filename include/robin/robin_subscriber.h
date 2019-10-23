#ifndef ROBIN_SUBSCRIBER_H
#define ROBIN_SUBSCRIBER_H
#include "robin/robin.h"
#include <ros/ros.h>
template <typename T1, typename T2>
class RobinSubscriber : public Robin<T1, T2>
{
  ros::Subscriber sub_;
  void write(const boost::shared_ptr< T2 const>& msg);
public:
  RobinSubscriber(std::string name, bool open=true);
  void open();
};
#endif
