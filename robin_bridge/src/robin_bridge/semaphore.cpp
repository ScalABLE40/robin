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
#include "robin_bridge/semaphore.h"
// stores semaphore name
Semaphore::Semaphore(std::string name)
  : name_(name)
{ }
// opens semaphore
void Semaphore::open()
{
  if (isOpen())
  {
    printf("Semaphore: semaphore '%s' is already open.", name_.c_str());
    throw 2;
  }
  errno = 0;  // assigned by sem_*() functions; google it
  semaphore_ptr_ = sem_open(name_.c_str(), O_CREAT, 00700, 1);  // open or create; 00700=700 like chmod
  // semaphore_ptr_ = sem_open(name_.c_str(), 0);  // open, don't create
  if (semaphore_ptr_ == SEM_FAILED)
  {
    printf("Semaphore: failed to open semaphore '%s'. errno %d: %s", name_.c_str(), errno, std::strerror(errno));
    throw 1;
  }
}
// checks if semaphore is open
bool Semaphore::isOpen()
{
  return !(semaphore_ptr_ == NULL || semaphore_ptr_ == SEM_FAILED);
}
// waits semaphore
void Semaphore::wait()
{
  // if (!isOpen())
  // {
  //   printf("Semaphore: semaphore '%s' is not open.", name_.c_str());
  //   throw 2;
  // }
  errno = 0;
  if (sem_wait(semaphore_ptr_) == -1)
  {
    printf("Semaphore: failed to wait semaphore '%s'. errno %d: %s", name_.c_str(), errno, std::strerror(errno));
    throw 1;
  }
}
// posts semaphore
void Semaphore::post()
{
  // if (!isOpen())
  // {
  //   printf("Semaphore: semaphore '%s' is not open.", name_.c_str());
  //   throw 2;
  // }
  errno = 0;
  if (sem_post(semaphore_ptr_) == -1)
  {
    printf("Semaphore: failed to post semaphore '%s'. errno %d: %s", name_.c_str(), errno, std::strerror(errno));
    throw 1;
  }
}
// closes and unlinks semaphore
void Semaphore::close()
{
  if (!isOpen())
  {
    printf("Semaphore: semaphore '%s' is not open.", name_.c_str());
    throw 2;
  }
  // close
  errno = 0;
  if (sem_close(semaphore_ptr_) == -1)
  {
    printf("Semaphore: failed to close semaphore '%s'. errno %d: %s", name_.c_str(), errno, std::strerror(errno));
    // throw 1;
  }
  // unlink
  errno = 0;
  if (sem_unlink(name_.c_str()) == -1)
  {
    printf("Semaphore: failed to unlink semaphore '%s'. errno %d: %s", name_.c_str(), errno, std::strerror(errno));
    throw 1;
  }
  semaphore_ptr_ = NULL;  //NEEDED?
}
// closes and unlinks semaphore if open
Semaphore::~Semaphore()
{
  if (isOpen())
  {
    close();
  }
}
