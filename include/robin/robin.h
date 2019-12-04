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
#ifndef ROBIN_H
#define ROBIN_H
#include <stdio.h>  // for printf()
#include <string>   // for std::string
#include "robin/semaphore.h"
#include "robin/shared_memory.h"
/**
 * Abstract class that handles SharedMemory and Semaphore objects.
 */
class Robin
{
protected:
  const static uint32_t QUEUE_SIZE = 100;  //TODO? pass as argument in constructor
  std::string name_;
  Semaphore semaphore_;
  SharedMemory shared_memory_;
public:
  Robin(std::string name, size_t size);
  virtual void open() = 0;
  virtual void close() = 0;
  bool isOpen();
};
#endif
