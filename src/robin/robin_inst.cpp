#include <cstdio>  // for snprintf()
#include "robin_publisher.cpp"
#include "robin_subscriber.cpp"
#include "robin/structs.h"
#include "robin/TestStruct_arr.h"
#include "robin/TestStruct_foo.h"
#include "std_msgs/Float64.h"
template<> void RobinPublisher<TestStruct_arr, robin::TestStruct_arr>::read()
{
  // char[] to std::string
  msg_.var_string = shm_ptr_->var_string;
  
  // pod array to boost::array (fixed length)
  memcpy(msg_.var_float64_array.data(), shm_ptr_->var_float64_array, sizeof(shm_ptr_->var_float64_array));
  
  // non-pod array to std::vector (variable length)
  shm_size_ = sizeof(shm_ptr_->var_bytemultiarray.layout.dim) / sizeof(MultiArrayDimension);
  msg_.var_bytemultiarray.layout.dim.resize(shm_size_);  //TODO execute only once, eg. in constructor
  for (int i = 0; i < shm_size_; i++)
  {
    msg_.var_bytemultiarray.layout.dim[i].label = shm_ptr_->var_bytemultiarray.layout.dim[i].label;
    msg_.var_bytemultiarray.layout.dim[i].size = shm_ptr_->var_bytemultiarray.layout.dim[i].size;
    msg_.var_bytemultiarray.layout.dim[i].stride = shm_ptr_->var_bytemultiarray.layout.dim[i].stride;
  }
  
  // pod to pod
  msg_.var_bytemultiarray.layout.data_offset = shm_ptr_->var_bytemultiarray.layout.data_offset;
  
  // pod array to std::vector
  msg_.var_bytemultiarray.data.assign(shm_ptr_->var_bytemultiarray.data, shm_ptr_->var_bytemultiarray.data + sizeof(shm_ptr_->var_bytemultiarray.data) / sizeof(uint8_t));
};
template<> void RobinSubscriber<TestStruct_arr, robin::TestStruct_arr>::write(robin::TestStruct_arr const *msg_ptr)
{
  // std::string to char[]
  snprintf(shm_ptr_->var_string, sizeof(shm_ptr_->var_string)+1, "%s", msg_ptr->var_string.c_str());

  // boost::array to pod array
  shm_size_ = sizeof(shm_ptr_->var_float64_array) / sizeof(double);
  msg_size_ = std::min(msg_ptr->var_float64_array.size(), shm_size_);
  memcpy(shm_ptr_->var_float64_array, msg_ptr->var_float64_array.data(), sizeof(double) * msg_size_);
  zeroUnsentElements(shm_ptr_->var_float64_array);

  // std::vector to non-pod array
  shm_size_ = sizeof(shm_ptr_->var_bytemultiarray.layout.dim) / sizeof(MultiArrayDimension);
  msg_size_ = std::min(msg_ptr->var_bytemultiarray.layout.dim.size(), shm_size_);
  for (int i = 0; i < msg_size_; i++)
  {
    snprintf(shm_ptr_->var_bytemultiarray.layout.dim[i].label,
            msg_ptr->var_bytemultiarray.layout.dim[i].label.size()+1,
            "%s", msg_ptr->var_bytemultiarray.layout.dim[i].label.c_str());
    shm_ptr_->var_bytemultiarray.layout.dim[i].size = msg_ptr->var_bytemultiarray.layout.dim[i].size;
    shm_ptr_->var_bytemultiarray.layout.dim[i].stride = msg_ptr->var_bytemultiarray.layout.dim[i].stride;
  }
  zeroUnsentElements(shm_ptr_->var_bytemultiarray.layout.dim);

  // pod to pod
  shm_ptr_->var_bytemultiarray.layout.data_offset = msg_ptr->var_bytemultiarray.layout.data_offset;
  
  // std::vector to pod array
  shm_size_ = sizeof(shm_ptr_->var_bytemultiarray.data) / sizeof(uint8_t);
  msg_size_ = std::min(msg_ptr->var_bytemultiarray.data.size(), shm_size_);
  memcpy(shm_ptr_->var_bytemultiarray.data, msg_ptr->var_bytemultiarray.data.data(), sizeof(uint8_t) * msg_size_);
  zeroUnsentElements(shm_ptr_->var_bytemultiarray.data);
};
template class RobinPublisher<double, std_msgs::Float64>;
template class RobinPublisher<TestStruct_foo, robin::TestStruct_foo>;
template class RobinPublisher<TestStruct_arr, robin::TestStruct_arr>;
template class RobinSubscriber<double, std_msgs::Float64>;
template class RobinSubscriber<TestStruct_foo, robin::TestStruct_foo>;
template class RobinSubscriber<TestStruct_arr, robin::TestStruct_arr>;
