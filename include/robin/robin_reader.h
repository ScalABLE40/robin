#ifndef ROBIN_READER_H
#define ROBIN_READER_H
#include "robin/robin.h"
#include <ros/ros.h>
#include <thread>
template <typename T1, typename T2>
class RobinReader : public Robin<T1, T2>
{
  ros::Publisher pub_;
  T2 msg_;
  std::thread *read_thread_;
  const static bool latch_ = true;  //TODO? pass as argument in constructor?
  const static int def_read_rate_ = 10;
  void readLoop(int rate);
  bool closing_ = false;
public:
  RobinReader(std::string name, bool open=true, int read_rate=def_read_rate_);
  void read();
  void open(int read_rate);
  void open();
  void close();
  ~RobinReader();
};
#endif
