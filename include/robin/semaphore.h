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
#ifndef ROBIN_SEMAPHORE_H
#define ROBIN_SEMAPHORE_H
#include <cstdio>       // for printf()
#include <cstring>      // for strerror()
#include <fcntl.h>      // for O_* constants
#include <semaphore.h>  // for sem_*()
#include <string>       // for std::string
/**
 * Handles opening, closing, waiting and posting of named semaphore.
 */
class Semaphore
{
  std::string name_;
  sem_t *semaphore_ptr_ = NULL;
public:
  Semaphore(std::string name);
  void open();
  bool isOpen();
  void wait();
  void post();
  void close();
  ~Semaphore();
};
#endif
