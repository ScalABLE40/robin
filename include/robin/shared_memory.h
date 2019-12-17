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
#ifndef ROBIN_SHARED_MEMORY_H
#define ROBIN_SHARED_MEMORY_H
#include <cstdio>     // for printf()
#include <cstring>    // for strerror()
#include <fcntl.h>    // for oflag constants
#include <string>     // for std::string
#include <sys/mman.h> // for shm_*()
#include <unistd.h>
/**
 * Handles opening and closing of shared memory.
 */
class SharedMemory
{
  std::string name_;
  size_t size_;
public:
  void *ptr_ = NULL;
  SharedMemory(std::string name, size_t size);
  void open();
  bool isOpen();
  void close();
  ~SharedMemory();
};
#endif
