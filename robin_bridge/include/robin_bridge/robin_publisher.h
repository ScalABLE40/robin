/**
 * Copyright 2019 INESC TEC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef ROBIN_PUBLISHER_H
#define ROBIN_PUBLISHER_H
#include <thread>
#include "robin_bridge/robin.h"
#include "ros/ros.h"
/**
 * Handles reading and publishing data from shared memory.
 */
template <typename T1, typename T2>
class RobinPublisher : public Robin
{
  const static bool LATCH = true;  //TODO? pass as argument in constructor
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
