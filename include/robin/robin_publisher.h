/*
 * Handles reading and publishing data from shared memory.
 */
#ifndef ROBIN_PUBLISHER_H
#define ROBIN_PUBLISHER_H
#include <thread>
#include "robin/robin.h"
#include "ros/ros.h"
template <typename T1, typename T2>
class RobinPublisher : public Robin
{
  const static bool LATCH = true;  //TODO? pass as argument in constructor?
  const static int DEF_READ_RATE = 10;
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  T1 *shm_ptr_;
  T2 msg_;
  std::thread *read_thread_ = NULL;
  bool closing_ = false;
  void publishLoop(int rate);
  void read();
public:
  RobinPublisher(ros::NodeHandle &nh, std::string name, bool open=true, int read_rate=DEF_READ_RATE);
  void open();
  void open(int read_rate);
  void publish();
  void close();
  ~RobinPublisher();
};
#endif
