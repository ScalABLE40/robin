// #include <algorithm>  // for std::copy()
// #include <cstring>  // for strncpy()
// #include <vector>  // for std::begin(), std::end()
// #include "shared_memory.cpp"
// #include "robin/structs.h"
// #include "robin/TestStruct_arr.h"
// #include "robin/TestStruct_foo.h"
// #include "std_msgs/Float64.h"
// // template<> void SharedMemory<double, std_msgs::Float64>::read(std_msgs::Float64 *msg_ptr)
// // {
// //   msg_ptr->data = *shm_ptr_;
// // };
// // template<> void SharedMemory<double, std_msgs::Float64>::write(std_msgs::Float64 const *msg_ptr)
// // {
// //   *shm_ptr_ = msg_ptr->data;
// // };
// // template<> void SharedMemory<TestStruct_foo, robin::TestStruct_foo>::read(robin::TestStruct_foo *msg_ptr)
// // {
// //   msg_ptr->var_bool_1 = shm_ptr_->var_bool_1;
// //   msg_ptr->var_struct_1.var_float64_1 = shm_ptr_->var_struct_1.var_float64_1;
// //   msg_ptr->var_struct_1.var_float64_2 = shm_ptr_->var_struct_1.var_float64_2;
// //   msg_ptr->var_struct_2.var_float64_1 = shm_ptr_->var_struct_2.var_float64_1;
// //   msg_ptr->var_struct_2.var_float64_2 = shm_ptr_->var_struct_2.var_float64_2;
// // };
// // template<> void SharedMemory<TestStruct_foo, robin::TestStruct_foo>::write(robin::TestStruct_foo const *msg_ptr)
// // {
// //   shm_ptr_->var_bool_1 = msg_ptr->var_bool_1;
// //   shm_ptr_->var_struct_1.var_float64_1 = msg_ptr->var_struct_1.var_float64_1;
// //   shm_ptr_->var_struct_1.var_float64_2 = msg_ptr->var_struct_1.var_float64_2;
// //   shm_ptr_->var_struct_2.var_float64_1 = msg_ptr->var_struct_2.var_float64_1;
// //   shm_ptr_->var_struct_2.var_float64_2 = msg_ptr->var_struct_2.var_float64_2;
// // };
// // template<typename T> vectorToArray(T &)
// template<> void SharedMemory<TestStruct_arr, robin::TestStruct_arr>::read(robin::TestStruct_arr *msg_ptr)
// {
//   // char[] to std::string
//   msg_ptr->var_string = shm_ptr_->var_string;
  
//   // pod array to boost::array (fixed length)
//   memcpy(msg_ptr->var_float64_array.data(), shm_ptr_->var_float64_array, sizeof(shm_ptr_->var_float64_array));
  
//   // non-pod array to std::vector (variable length)
//   msg_size_ = sizeof(shm_ptr_->var_bytemultiarray.layout.dim) / sizeof(MultiArrayDimension);
//   msg_ptr->var_bytemultiarray.layout.dim.resize(msg_size_);  //TODO execute only once, eg. in constructor
//   for (int i = 0; i < msg_size_; i++)
//   {
//     msg_ptr->var_bytemultiarray.layout.dim[i].label = shm_ptr_->var_bytemultiarray.layout.dim[i].label;
//     msg_ptr->var_bytemultiarray.layout.dim[i].size = shm_ptr_->var_bytemultiarray.layout.dim[i].size;
//     msg_ptr->var_bytemultiarray.layout.dim[i].stride = shm_ptr_->var_bytemultiarray.layout.dim[i].stride;
//   }
  
//   // pod to pod
//   msg_ptr->var_bytemultiarray.layout.data_offset = shm_ptr_->var_bytemultiarray.layout.data_offset;
  
//   // pod array to std::vector
//   msg_ptr->var_bytemultiarray.data.assign(shm_ptr_->var_bytemultiarray.data, shm_ptr_->var_bytemultiarray.data + sizeof(shm_ptr_->var_bytemultiarray.data) / sizeof(uint8_t));
// };
// template<> void SharedMemory<TestStruct_arr, robin::TestStruct_arr>::write(robin::TestStruct_arr const *msg_ptr)
// {
//   // std::string to char[]
//   strncpy(shm_ptr_->var_string, msg_ptr->var_string.c_str(), sizeof(shm_ptr_->var_string));

//   // boost::array to pod array
//   shm_size_ = sizeof(shm_ptr_->var_float64_array) / sizeof(double);
//   msg_size_ = std::min(msg_ptr->var_float64_array.size(), shm_size_);
//   memcpy(shm_ptr_->var_float64_array, msg_ptr->var_float64_array.data(), sizeof(double) * msg_size_);
//   // zero unsent elements
//   if (msg_size_ < shm_size_)
//   {
//     memset(shm_ptr_->var_float64_array + msg_size_, 0, shm_size_ - msg_size_);
//   }
//   // zeroUnsent(shm_ptr_->var_float64_array);

//   // std::vector to non-pod array
//   shm_size_ = sizeof(shm_ptr_->var_bytemultiarray.layout.dim) / sizeof(MultiArrayDimension);
//   msg_size_ = std::min(msg_ptr->var_bytemultiarray.layout.dim.size(), shm_size_);
//   for (int i = 0; i < msg_size_; i++)
//   {
//     strncpy(shm_ptr_->var_bytemultiarray.layout.dim[i].label,
//             msg_ptr->var_bytemultiarray.layout.dim[i].label.c_str(),
//             msg_ptr->var_bytemultiarray.layout.dim[i].label.size());
//     shm_ptr_->var_bytemultiarray.layout.dim[i].size = msg_ptr->var_bytemultiarray.layout.dim[i].size;
//     shm_ptr_->var_bytemultiarray.layout.dim[i].stride = msg_ptr->var_bytemultiarray.layout.dim[i].stride;
//   }
//   // zero unsent elements
//   if (msg_size_ < shm_size_)
//   {
//     memset(shm_ptr_->var_bytemultiarray.layout.dim + msg_size_, 0, shm_size_ - msg_size_);
//   }
//   // zeroUnsent(shm_ptr_->var_bytemultiarray.layout.dim);

//   // pod to pod
//   shm_ptr_->var_bytemultiarray.layout.data_offset = msg_ptr->var_bytemultiarray.layout.data_offset;
  
//   // std::vector to pod array
//   shm_size_ = sizeof(shm_ptr_->var_bytemultiarray.data) / sizeof(uint8_t);
//   msg_size_ = std::min(msg_ptr->var_bytemultiarray.data.size(), shm_size_);
//   memcpy(shm_ptr_->var_bytemultiarray.data, msg_ptr->var_bytemultiarray.data.data(), sizeof(uint8_t) * msg_size_);
//   // zero unsent elements
//   if (msg_size_ < shm_size_)
//   {
//     memset(shm_ptr_->var_bytemultiarray.data + msg_size_, 0, shm_size_ - msg_size_);
//   }
//   // zeroUnsent(shm_ptr_->var_bytemultiarray.data);
// };
// template class SharedMemory<double, std_msgs::Float64>;
// template class SharedMemory<TestStruct_foo, robin::TestStruct_foo>;
// template class SharedMemory<TestStruct_arr, robin::TestStruct_arr>;
