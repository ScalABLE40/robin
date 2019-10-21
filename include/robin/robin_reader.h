#ifndef ROBIN_READER_H
#define ROBIN_READER_H
#include "robin/robin.h"
#include <ros/ros.h>
// #include <thread>
template <typename T1, typename T2>
class RobinReader : public Robin<T1, T2>
{
  ros::Publisher pub_;
  T2 msg_;
  // std::thread *read_thread_;
  const bool latch_ = true;  //TODO? pass as argument in constructor?
  // void readLoop(int rate);
public:
  RobinReader(std::string name, bool open=true);
  void read();
  void open();//int read_rate = 10);
  // void close();
};
#endif
