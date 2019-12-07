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
#ifndef ROBIN_SUBSCRIBER_H
#define ROBIN_SUBSCRIBER_H
#include "robin_bridge/robin.h"
#include "ros/ros.h"
/**
 * Handles receiving and writting data to shared memory.
 */
template <typename T1, typename T2>
class RobinSubscriber : public Robin
{
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  T1 *shm_ptr_;
  void subscriberCallback(const boost::shared_ptr<T2 const>& msg);
  void write(T2 const *msg_ptr);
  template<typename T> void zeroUnsentElements(T *ptr, size_t msg_size, size_t shm_size);
public:
  RobinSubscriber(ros::NodeHandle &nh, std::string name, bool open=true);
  void open();
  void close();
  ~RobinSubscriber();
};
#endif
