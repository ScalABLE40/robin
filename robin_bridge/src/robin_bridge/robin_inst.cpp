#include <cstdio>   // for std::snprintf()
#include <cstring>  // for std::memcpy()
#include "robin_publisher.cpp"
#include "robin_subscriber.cpp"
#include "robin_bridge/structs.h"
#include "robin_bridge/AccelStampedArray.h"
#include "robin_bridge/StringVarLenArray.h"
#include "robin_bridge/TestStruct.h"
#include "std_msgs/Float64.h"
template<> void RobinSubscriber<char[5][81], robin_bridge::StringVarLenArray>::write(robin_bridge::StringVarLenArray const *msg_ptr)
{
  std::printf("[%s] writting shm...\n", name_.c_str());
  // non-pod std::vector to array
  size_t shm_len_0 = sizeof((*shm_ptr_)) / sizeof(char[81]);
  size_t msg_len_0 = std::min((*msg_ptr).data.size(), shm_len_0);
  for (int i_0 = 0; i_0 < msg_len_0; i_0++)
  {
    // std::string to char[]
    std::snprintf((*shm_ptr_)[i_0], sizeof((*shm_ptr_)[i_0]), "%s", (*msg_ptr).data[i_0].c_str());
    // END std::string to char[]
  }
  zeroUnsentElements((*shm_ptr_), msg_len_0, shm_len_0);
  // END non-pod std::vector to array
}
template<> void RobinSubscriber<TestStruct, robin_bridge::TestStruct>::write(robin_bridge::TestStruct const *msg_ptr)
{
  std::printf("[%s] writting shm...\n", name_.c_str());
  (*shm_ptr_).var_bool = (*msg_ptr).var_bool;
  (*shm_ptr_).var_byte = (*msg_ptr).var_byte;
  (*shm_ptr_).var_int16 = (*msg_ptr).var_int16;
  (*shm_ptr_).var_uint64 = (*msg_ptr).var_uint64;
  (*shm_ptr_).var_float32 = (*msg_ptr).var_float32;
  (*shm_ptr_).var_float64 = (*msg_ptr).var_float64;
  // std::string to char[]
  std::snprintf((*shm_ptr_).var_string, sizeof((*shm_ptr_).var_string), "%s", (*msg_ptr).var_string.c_str());
  // END std::string to char[]
  memcpy(&((*shm_ptr_).var_pose), &((*msg_ptr).var_pose), sizeof((*shm_ptr_).var_pose));
  // non-pod boost::array to array
  size_t shm_len_1 = sizeof((*shm_ptr_).var_struct_array) / sizeof(ByteMultiArray);
  for (int i_1 = 0; i_1 < shm_len_1; i_1++)
  {
    // non-pod std::vector to array
    size_t shm_len_1_0 = sizeof((*shm_ptr_).var_struct_array[i_1].layout.dim) / sizeof(MultiArrayDimension);
    size_t msg_len_1_0 = std::min((*msg_ptr).var_struct_array[i_1].layout.dim.size(), shm_len_1_0);
    for (int i_1_0 = 0; i_1_0 < msg_len_1_0; i_1_0++)
    {
      // std::string to char[]
      std::snprintf((*shm_ptr_).var_struct_array[i_1].layout.dim[i_1_0].label, sizeof((*shm_ptr_).var_struct_array[i_1].layout.dim[i_1_0].label), "%s", (*msg_ptr).var_struct_array[i_1].layout.dim[i_1_0].label.c_str());
      // END std::string to char[]
      (*shm_ptr_).var_struct_array[i_1].layout.dim[i_1_0].size = (*msg_ptr).var_struct_array[i_1].layout.dim[i_1_0].size;
      (*shm_ptr_).var_struct_array[i_1].layout.dim[i_1_0].stride = (*msg_ptr).var_struct_array[i_1].layout.dim[i_1_0].stride;
    }
    zeroUnsentElements((*shm_ptr_).var_struct_array[i_1].layout.dim, msg_len_1_0, shm_len_1_0);
    // END non-pod std::vector to array
    (*shm_ptr_).var_struct_array[i_1].layout.data_offset = (*msg_ptr).var_struct_array[i_1].layout.data_offset;
    // pod std::vector to array
    size_t const shm_len_1_1 = sizeof((*shm_ptr_).var_struct_array[i_1].data) / sizeof(int8_t);
    size_t msg_len_1_1 = std::min((*msg_ptr).var_struct_array[i_1].data.size(), shm_len_1_1);
    std::memcpy((*shm_ptr_).var_struct_array[i_1].data, (*msg_ptr).var_struct_array[i_1].data.data(), sizeof(int8_t) * msg_len_1_1);
    zeroUnsentElements((*shm_ptr_).var_struct_array[i_1].data, msg_len_1_1, shm_len_1_1);
    // END pod std::vector to array
  }
  // END non-pod boost::array to array
  // non-pod std::vector to array
  size_t shm_len_1_0 = sizeof((*shm_ptr_).var_struct_varlen_array) / sizeof(ByteMultiArray);
  size_t msg_len_1_0 = std::min((*msg_ptr).var_struct_varlen_array.size(), shm_len_1_0);
  for (int i_1_0 = 0; i_1_0 < msg_len_1_0; i_1_0++)
  {
    // non-pod std::vector to array
    size_t shm_len_1_1 = sizeof((*shm_ptr_).var_struct_varlen_array[i_1_0].layout.dim) / sizeof(MultiArrayDimension);
    size_t msg_len_1_1 = std::min((*msg_ptr).var_struct_varlen_array[i_1_0].layout.dim.size(), shm_len_1_1);
    for (int i_1_1 = 0; i_1_1 < msg_len_1_1; i_1_1++)
    {
      // std::string to char[]
      std::snprintf((*shm_ptr_).var_struct_varlen_array[i_1_0].layout.dim[i_1_1].label, sizeof((*shm_ptr_).var_struct_varlen_array[i_1_0].layout.dim[i_1_1].label), "%s", (*msg_ptr).var_struct_varlen_array[i_1_0].layout.dim[i_1_1].label.c_str());
      // END std::string to char[]
      (*shm_ptr_).var_struct_varlen_array[i_1_0].layout.dim[i_1_1].size = (*msg_ptr).var_struct_varlen_array[i_1_0].layout.dim[i_1_1].size;
      (*shm_ptr_).var_struct_varlen_array[i_1_0].layout.dim[i_1_1].stride = (*msg_ptr).var_struct_varlen_array[i_1_0].layout.dim[i_1_1].stride;
    }
    zeroUnsentElements((*shm_ptr_).var_struct_varlen_array[i_1_0].layout.dim, msg_len_1_1, shm_len_1_1);
    // END non-pod std::vector to array
    (*shm_ptr_).var_struct_varlen_array[i_1_0].layout.data_offset = (*msg_ptr).var_struct_varlen_array[i_1_0].layout.data_offset;
    // pod std::vector to array
    size_t const shm_len_1_1 = sizeof((*shm_ptr_).var_struct_varlen_array[i_1_0].data) / sizeof(int8_t);
    size_t msg_len_1_1 = std::min((*msg_ptr).var_struct_varlen_array[i_1_0].data.size(), shm_len_1_1);
    std::memcpy((*shm_ptr_).var_struct_varlen_array[i_1_0].data, (*msg_ptr).var_struct_varlen_array[i_1_0].data.data(), sizeof(int8_t) * msg_len_1_1);
    zeroUnsentElements((*shm_ptr_).var_struct_varlen_array[i_1_0].data, msg_len_1_1, shm_len_1_1);
    // END pod std::vector to array
  }
  zeroUnsentElements((*shm_ptr_).var_struct_varlen_array, msg_len_1_0, shm_len_1_0);
  // END non-pod std::vector to array
}
template<> void RobinSubscriber<AccelStamped[2], robin_bridge::AccelStampedArray>::write(robin_bridge::AccelStampedArray const *msg_ptr)
{
  std::printf("[%s] writting shm...\n", name_.c_str());
  // non-pod boost::array to array
  size_t shm_len_0 = sizeof((*shm_ptr_)) / sizeof(AccelStamped);
  for (int i_0 = 0; i_0 < shm_len_0; i_0++)
  {
    (*shm_ptr_)[i_0].header.seq = (*msg_ptr).data[i_0].header.seq;
    memcpy(&((*shm_ptr_)[i_0].header.stamp), &((*msg_ptr).data[i_0].header.stamp), sizeof((*shm_ptr_)[i_0].header.stamp));
    // std::string to char[]
    std::snprintf((*shm_ptr_)[i_0].header.frame_id, sizeof((*shm_ptr_)[i_0].header.frame_id), "%s", (*msg_ptr).data[i_0].header.frame_id.c_str());
    // END std::string to char[]
    memcpy(&((*shm_ptr_)[i_0].accel), &((*msg_ptr).data[i_0].accel), sizeof((*shm_ptr_)[i_0].accel));
  }
  // END non-pod boost::array to array
}
template<> void RobinPublisher<char[5][81], robin_bridge::StringVarLenArray>::read()
{
  std::printf("[%s] reading shm...\n", name_.c_str());
  // non-pod array to std::vector
  size_t shm_len_0 = sizeof((*shm_ptr_)) / sizeof(char[81]);
  msg_.data.resize(shm_len_0);  //TODO execute only once, eg. in constructor
  for (int i_0 = 0; i_0 < shm_len_0; i_0++)
  {
    // char[] to std::string
    msg_.data[i_0] = (*shm_ptr_)[i_0];
    // END char[] to std::string
  }
  // END non-pod array to std::vector
}
template<> void RobinPublisher<TestStruct, robin_bridge::TestStruct>::read()
{
  std::printf("[%s] reading shm...\n", name_.c_str());
  msg_.var_bool = (*shm_ptr_).var_bool;
  msg_.var_byte = (*shm_ptr_).var_byte;
  msg_.var_int16 = (*shm_ptr_).var_int16;
  msg_.var_uint64 = (*shm_ptr_).var_uint64;
  msg_.var_float32 = (*shm_ptr_).var_float32;
  msg_.var_float64 = (*shm_ptr_).var_float64;
  // char[] to std::string
  msg_.var_string = (*shm_ptr_).var_string;
  // END char[] to std::string
  memcpy(&(msg_.var_pose), &((*shm_ptr_).var_pose), sizeof(msg_.var_pose));
  // non-pod array to boost::array
  size_t shm_len_1 = sizeof((*shm_ptr_).var_struct_array) / sizeof(ByteMultiArray);
  for (int i_1 = 0; i_1 < shm_len_1; i_1++)
  {
    // non-pod array to std::vector
    size_t shm_len_1_0 = sizeof((*shm_ptr_).var_struct_array[i_1].layout.dim) / sizeof(MultiArrayDimension);
    msg_.var_struct_array[i_1].layout.dim.resize(shm_len_1_0);  //TODO execute only once, eg. in constructor
    for (int i_1_0 = 0; i_1_0 < shm_len_1_0; i_1_0++)
    {
      // char[] to std::string
      msg_.var_struct_array[i_1].layout.dim[i_1_0].label = (*shm_ptr_).var_struct_array[i_1].layout.dim[i_1_0].label;
      // END char[] to std::string
      msg_.var_struct_array[i_1].layout.dim[i_1_0].size = (*shm_ptr_).var_struct_array[i_1].layout.dim[i_1_0].size;
      msg_.var_struct_array[i_1].layout.dim[i_1_0].stride = (*shm_ptr_).var_struct_array[i_1].layout.dim[i_1_0].stride;
    }
    // END non-pod array to std::vector
    msg_.var_struct_array[i_1].layout.data_offset = (*shm_ptr_).var_struct_array[i_1].layout.data_offset;
    // pod array to std::vector
    size_t const shm_len_1_1 = sizeof((*shm_ptr_).var_struct_array[i_1].data) / sizeof(int8_t);
    msg_.var_struct_array[i_1].data.assign((*shm_ptr_).var_struct_array[i_1].data, (*shm_ptr_).var_struct_array[i_1].data + shm_len_1_1);
    // END pod array to std::vector
  }
  // END non-pod array to boost::array
  // non-pod array to std::vector
  size_t shm_len_1_0 = sizeof((*shm_ptr_).var_struct_varlen_array) / sizeof(ByteMultiArray);
  msg_.var_struct_varlen_array.resize(shm_len_1_0);  //TODO execute only once, eg. in constructor
  for (int i_1_0 = 0; i_1_0 < shm_len_1_0; i_1_0++)
  {
    // non-pod array to std::vector
    size_t shm_len_1_1 = sizeof((*shm_ptr_).var_struct_varlen_array[i_1_0].layout.dim) / sizeof(MultiArrayDimension);
    msg_.var_struct_varlen_array[i_1_0].layout.dim.resize(shm_len_1_1);  //TODO execute only once, eg. in constructor
    for (int i_1_1 = 0; i_1_1 < shm_len_1_1; i_1_1++)
    {
      // char[] to std::string
      msg_.var_struct_varlen_array[i_1_0].layout.dim[i_1_1].label = (*shm_ptr_).var_struct_varlen_array[i_1_0].layout.dim[i_1_1].label;
      // END char[] to std::string
      msg_.var_struct_varlen_array[i_1_0].layout.dim[i_1_1].size = (*shm_ptr_).var_struct_varlen_array[i_1_0].layout.dim[i_1_1].size;
      msg_.var_struct_varlen_array[i_1_0].layout.dim[i_1_1].stride = (*shm_ptr_).var_struct_varlen_array[i_1_0].layout.dim[i_1_1].stride;
    }
    // END non-pod array to std::vector
    msg_.var_struct_varlen_array[i_1_0].layout.data_offset = (*shm_ptr_).var_struct_varlen_array[i_1_0].layout.data_offset;
    // pod array to std::vector
    size_t const shm_len_1_1 = sizeof((*shm_ptr_).var_struct_varlen_array[i_1_0].data) / sizeof(int8_t);
    msg_.var_struct_varlen_array[i_1_0].data.assign((*shm_ptr_).var_struct_varlen_array[i_1_0].data, (*shm_ptr_).var_struct_varlen_array[i_1_0].data + shm_len_1_1);
    // END pod array to std::vector
  }
  // END non-pod array to std::vector
}
template<> void RobinPublisher<AccelStamped[2], robin_bridge::AccelStampedArray>::read()
{
  std::printf("[%s] reading shm...\n", name_.c_str());
  // non-pod array to boost::array
  size_t shm_len_0 = sizeof((*shm_ptr_)) / sizeof(AccelStamped);
  for (int i_0 = 0; i_0 < shm_len_0; i_0++)
  {
    msg_.data[i_0].header.seq = (*shm_ptr_)[i_0].header.seq;
    memcpy(&(msg_.data[i_0].header.stamp), &((*shm_ptr_)[i_0].header.stamp), sizeof(msg_.data[i_0].header.stamp));
    // char[] to std::string
    msg_.data[i_0].header.frame_id = (*shm_ptr_)[i_0].header.frame_id;
    // END char[] to std::string
    memcpy(&(msg_.data[i_0].accel), &((*shm_ptr_)[i_0].accel), sizeof(msg_.data[i_0].accel));
  }
  // END non-pod array to boost::array
}
template class RobinSubscriber<double, std_msgs::Float64>;
template class RobinSubscriber<char[5][81], robin_bridge::StringVarLenArray>;
template class RobinSubscriber<TestStruct, robin_bridge::TestStruct>;
template class RobinSubscriber<AccelStamped[2], robin_bridge::AccelStampedArray>;
template class RobinPublisher<double, std_msgs::Float64>;
template class RobinPublisher<char[5][81], robin_bridge::StringVarLenArray>;
template class RobinPublisher<TestStruct, robin_bridge::TestStruct>;
template class RobinPublisher<AccelStamped[2], robin_bridge::AccelStampedArray>;
