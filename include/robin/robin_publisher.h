#ifndef ROBIN_PUBLISHER_H
#define ROBIN_PUBLISHER_H
#include "robin/robin.h"
#include <ros/ros.h>
#include <thread>
template <typename T1, typename T2>
class RobinPublisher : public Robin<T1, T2>
{
  const static bool LATCH = true;  //TODO? pass as argument in constructor?
  const static int DEF_READ_RATE = 10;
  ros::Publisher publisher_;
  T2 msg_;
  std::thread *read_thread_;
  bool closing_ = false;
  void publishLoop(int rate);
  // void read();
public:
  RobinPublisher(std::string name, bool open=true, int read_rate=DEF_READ_RATE);
  void open();
  void open(int read_rate);
  void publish();
  void close();
  ~RobinPublisher();
};
#endif
