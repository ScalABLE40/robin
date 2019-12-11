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
#include "robin_bridge/robin.h"
// stores robin name and creates semaphore and shared memory
Robin::Robin(std::string name, size_t size)
  : name_(name), semaphore_(Semaphore(name_)), shared_memory_(SharedMemory(name_, size))
{ }
// opens shared memory and semaphore
void Robin::open()
{
  if (isOpen())
  {
    printf("Robin: opening failed. Bridge '%s' is already open.", name_.c_str());
    throw 2;
  }
  semaphore_.open();
  shared_memory_.open();
}
// closes shared memory and semaphore
void Robin::close()
{
  if (!isOpen())
  {
    printf("Robin: closing failed. Bridge '%s' is not open.", name_.c_str());
    throw 2;
  }
  //TODO? wait for semaphore?
  shared_memory_.close();
  semaphore_.close();
}
// checks if robin bridge is open
bool Robin::isOpen()
{
  return semaphore_.isOpen() && shared_memory_.isOpen();
}
