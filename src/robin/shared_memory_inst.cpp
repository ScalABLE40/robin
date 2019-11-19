#include <algorithm>  // for std::copy()
#include <cstring>  // for strncpy()
#include <vector>  // for std::begin(), std::end()
#include "shared_memory.cpp"
#include "robin/structs.h"
#include "robin/TestStruct_arr.h"
#include "robin/TestStruct_foo.h"
#include "std_msgs/Float64.h"
// template<> void SharedMemory<double, std_msgs::Float64>::read(std_msgs::Float64 *msg_ptr)
// {
//   msg_ptr->data = *shm_ptr_;
// };
// template<> void SharedMemory<double, std_msgs::Float64>::write(std_msgs::Float64 const *msg_ptr)
// {
//   *shm_ptr_ = msg_ptr->data;
// };
// template<> void SharedMemory<TestStruct_foo, robin::TestStruct_foo>::read(robin::TestStruct_foo *msg_ptr)
// {
//   msg_ptr->var_bool_1 = shm_ptr_->var_bool_1;
//   msg_ptr->var_struct_1.var_float64_1 = shm_ptr_->var_struct_1.var_float64_1;
//   msg_ptr->var_struct_1.var_float64_2 = shm_ptr_->var_struct_1.var_float64_2;
//   msg_ptr->var_struct_2.var_float64_1 = shm_ptr_->var_struct_2.var_float64_1;
//   msg_ptr->var_struct_2.var_float64_2 = shm_ptr_->var_struct_2.var_float64_2;
// };
// template<> void SharedMemory<TestStruct_foo, robin::TestStruct_foo>::write(robin::TestStruct_foo const *msg_ptr)
// {
//   shm_ptr_->var_bool_1 = msg_ptr->var_bool_1;
//   shm_ptr_->var_struct_1.var_float64_1 = msg_ptr->var_struct_1.var_float64_1;
//   shm_ptr_->var_struct_1.var_float64_2 = msg_ptr->var_struct_1.var_float64_2;
//   shm_ptr_->var_struct_2.var_float64_1 = msg_ptr->var_struct_2.var_float64_1;
//   shm_ptr_->var_struct_2.var_float64_2 = msg_ptr->var_struct_2.var_float64_2;
// };
// template<typename T> vectorToArray(T &)
template<> void SharedMemory<TestStruct_arr, robin::TestStruct_arr>::read(robin::TestStruct_arr *msg_ptr)
{
  // char[] to std::string
  msg_ptr->var_string = shm_ptr_->var_string;
  
  // fixed length pod array to std::vector
  memcpy(msg_ptr->var_float64_array.data(), shm_ptr_->var_float64_array, sizeof(shm_ptr_->var_float64_array));
  
  // variable length non-pod array to std::vector
  arr_size_ = sizeof(shm_ptr_->var_bytemultiarray.layout.dim) / sizeof(MultiArrayDimension);
  msg_ptr->var_bytemultiarray.layout.dim.resize(arr_size_);  //TODO execute only once, eg. in constructor
  for (int i = 0; i < arr_size_; i++)
  {
    msg_ptr->var_bytemultiarray.layout.dim[i].label = shm_ptr_->var_bytemultiarray.layout.dim[i].label;
    msg_ptr->var_bytemultiarray.layout.dim[i].size = shm_ptr_->var_bytemultiarray.layout.dim[i].size;
    msg_ptr->var_bytemultiarray.layout.dim[i].stride = shm_ptr_->var_bytemultiarray.layout.dim[i].stride;
  }
  
  // pod to pod
  msg_ptr->var_bytemultiarray.layout.data_offset = shm_ptr_->var_bytemultiarray.layout.data_offset;
  
  // variable length pod array to std::vector
  msg_ptr->var_bytemultiarray.data.assign(shm_ptr_->var_bytemultiarray.data, shm_ptr_->var_bytemultiarray.data + sizeof(shm_ptr_->var_bytemultiarray.data) / sizeof(uint8_t));
};
template<> void SharedMemory<TestStruct_arr, robin::TestStruct_arr>::write(robin::TestStruct_arr const *msg_ptr)
{
  // std::string to char[]
  strncpy(shm_ptr_->var_string, msg_ptr->var_string.c_str(), sizeof(shm_ptr_->var_string));

  // std::vector to fixed length pod array
  max_size_ = sizeof(shm_ptr_->var_float64_array) / sizeof(double);
  arr_size_ = std::min(msg_ptr->var_float64_array.size(), max_size_);
  memcpy(shm_ptr_->var_float64_array, msg_ptr->var_float64_array.data(), sizeof(double) * arr_size_);
  // zero unsent elements
  if (arr_size_ < max_size_)
  {
    memset(shm_ptr_->var_float64_array + arr_size_, 0, max_size_ - arr_size_);
  }
  
  // std::vector to variable length non-pod array
  max_size_ = sizeof(shm_ptr_->var_bytemultiarray.layout.dim) / sizeof(MultiArrayDimension);
  arr_size_ = std::min(msg_ptr->var_bytemultiarray.layout.dim.size(), max_size_);
  for (int i = 0; i < arr_size_; i++)
  {
    strncpy(shm_ptr_->var_bytemultiarray.layout.dim[i].label,
            msg_ptr->var_bytemultiarray.layout.dim[i].label.c_str(),
            msg_ptr->var_bytemultiarray.layout.dim[i].label.size());
    shm_ptr_->var_bytemultiarray.layout.dim[i].size = msg_ptr->var_bytemultiarray.layout.dim[i].size;
    shm_ptr_->var_bytemultiarray.layout.dim[i].stride = msg_ptr->var_bytemultiarray.layout.dim[i].stride;
  }
  // zero unsent elements
  if (arr_size_ < max_size_)
  {
    memset(shm_ptr_->var_bytemultiarray.layout.dim + arr_size_, 0, max_size_ - arr_size_);
  }

  // pod to pod
  shm_ptr_->var_bytemultiarray.layout.data_offset = msg_ptr->var_bytemultiarray.layout.data_offset;
  
  // std::vector to variable length pod array
  max_size_ = sizeof(shm_ptr_->var_bytemultiarray.data) / sizeof(uint8_t);
  arr_size_ = std::min(msg_ptr->var_bytemultiarray.data.size(), max_size_);
  memcpy(shm_ptr_->var_bytemultiarray.data, msg_ptr->var_bytemultiarray.data.data(), sizeof(uint8_t) * arr_size_);
  // zero unsent elements
  if (arr_size_ < max_size_)
  {
    memset(shm_ptr_->var_bytemultiarray.data + arr_size_, 0, max_size_ - arr_size_);
  }
};
template class SharedMemory<double, std_msgs::Float64>;
template class SharedMemory<TestStruct_foo, robin::TestStruct_foo>;
template class SharedMemory<TestStruct_arr, robin::TestStruct_arr>;
