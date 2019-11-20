/*
 * Handles receiving and writting data to shared memory.
 */
#ifndef ROBIN_SUBSCRIBER_H
#define ROBIN_SUBSCRIBER_H
#include "robin/robin.h"
#include "ros/ros.h"
template <typename T1, typename T2>
class RobinSubscriber : public Robin
{
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  T1 *shm_ptr_;
  size_t msg_size_;
  void subscriberCallback(const boost::shared_ptr<T2 const>& msg);
  void write(T2 const *msg_ptr);
  template<typename T> void zeroUnsentElements(T *ptr);
public:
  RobinSubscriber(ros::NodeHandle &nh, std::string name, bool open=true);
  void open();
  void close();
  ~RobinSubscriber();
};
#endif
