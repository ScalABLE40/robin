#include <cstdio>   // for std::snprintf()
#include <cstring>  // for std::memcpy()
#include "robin_publisher.cpp"
#include "robin_subscriber.cpp"
#include "robin/structs.h"
#include "geometry_msgs/AccelStamped.h"
#include "robin/TestStruct.h"
#include "std_msgs/Float64.h"
template<> void RobinSubscriber<double, std_msgs::Float64>::write(std_msgs::Float64 const *msg_ptr)
{
  (*shm_ptr_) = (*msg_ptr).data;
}
template<> void RobinSubscriber<AccelStamped, geometry_msgs::AccelStamped>::write(geometry_msgs::AccelStamped const *msg_ptr)
{
  (*shm_ptr_).header.seq = (*msg_ptr).header.seq;
  // ros_type to pod
  memcpy(&((*shm_ptr_).header.stamp), &((*msg_ptr).header.stamp), sizeof((*shm_ptr_).header.stamp));
  // std::string to char[]
  std::snprintf((*shm_ptr_).header.frame_id, sizeof((*shm_ptr_).header.frame_id), "%s", (*msg_ptr).header.frame_id.c_str());
  (*shm_ptr_).accel.linear.x = (*msg_ptr).accel.linear.x;
  (*shm_ptr_).accel.linear.y = (*msg_ptr).accel.linear.y;
  (*shm_ptr_).accel.linear.z = (*msg_ptr).accel.linear.z;
  (*shm_ptr_).accel.angular.x = (*msg_ptr).accel.angular.x;
  (*shm_ptr_).accel.angular.y = (*msg_ptr).accel.angular.y;
  (*shm_ptr_).accel.angular.z = (*msg_ptr).accel.angular.z;
}
template<> void RobinSubscriber<TestStruct, robin::TestStruct>::write(robin::TestStruct const *msg_ptr)
{
  (*shm_ptr_).var_bool = (*msg_ptr).var_bool;
  (*shm_ptr_).var_byte = (*msg_ptr).var_byte;
  (*shm_ptr_).var_int16 = (*msg_ptr).var_int16;
  (*shm_ptr_).var_uint64 = (*msg_ptr).var_uint64;
  (*shm_ptr_).var_float32 = (*msg_ptr).var_float32;
  (*shm_ptr_).var_float64 = (*msg_ptr).var_float64;
  // std::string to char[]
  std::snprintf((*shm_ptr_).var_string, sizeof((*shm_ptr_).var_string), "%s", (*msg_ptr).var_string.c_str());
  (*shm_ptr_).var_pose.position.x = (*msg_ptr).var_pose.position.x;
  (*shm_ptr_).var_pose.position.y = (*msg_ptr).var_pose.position.y;
  (*shm_ptr_).var_pose.position.z = (*msg_ptr).var_pose.position.z;
  (*shm_ptr_).var_pose.orientation.x = (*msg_ptr).var_pose.orientation.x;
  (*shm_ptr_).var_pose.orientation.y = (*msg_ptr).var_pose.orientation.y;
  (*shm_ptr_).var_pose.orientation.z = (*msg_ptr).var_pose.orientation.z;
  (*shm_ptr_).var_pose.orientation.w = (*msg_ptr).var_pose.orientation.w;
  // boost::array to non-pod array
  size_t shmlen0 = sizeof((*shm_ptr_).var_struct_array) / sizeof(ByteMultiArray);
  for (int i0 = 0; i0 < shmlen0; i0++)
  {
    // std::vector to non-pod array
    size_t shmlen00 = sizeof((*shm_ptr_).var_struct_array[i0].layout.dim) / sizeof(MultiArrayDimension);
    size_t msglen00 = std::min((*msg_ptr).var_struct_array[i0].layout.dim.size(), shm_len_dim);
    for (int i00 = 0; i00 < msglen00; i00++)
    {
      // std::string to char[]
      std::snprintf((*shm_ptr_).var_struct_array[i0].layout.dim[i00].label, sizeof((*shm_ptr_).var_struct_array[i0].layout.dim[i00].label), "%s", (*msg_ptr).var_struct_array[i0].layout.dim[i00].label.c_str());
      (*shm_ptr_).var_struct_array[i0].layout.dim[i00].size = (*msg_ptr).var_struct_array[i0].layout.dim[i00].size;
      (*shm_ptr_).var_struct_array[i0].layout.dim[i00].stride = (*msg_ptr).var_struct_array[i0].layout.dim[i00].stride;
    }
    zeroUnsentElements((*shm_ptr_).var_struct_array[i0].layout.dim, msg_len_dim, shm_len_dim);
    (*shm_ptr_).var_struct_array[i0].layout.data_offset = (*msg_ptr).var_struct_array[i0].layout.data_offset;
    // std::vector to pod array
    size_t const shmlen01 = sizeof((*shm_ptr_).var_struct_array[i0].data) / sizeof(int8_t);
    size_t msglen01 = std::min((*msg_ptr).var_struct_array[i0].data.size(), shm_len_data);
    std::memcpy((*shm_ptr_).var_struct_array[i0].data, (*msg_ptr).var_struct_array[i0].data.data(), sizeof(int8_t) * msg_len_data);
    zeroUnsentElements((*shm_ptr_).var_struct_array[i0].data, msg_len_data, shm_len_data);
  }
  // std::vector to non-pod array
  size_t shmlen1 = sizeof((*shm_ptr_).var_struct_varlen_array) / sizeof(ByteMultiArray);
  size_t msglen1 = std::min((*msg_ptr).var_struct_varlen_array.size(), shm_len_var_struct_varlen_array);
  for (int i1 = 0; i1 < msglen1; i1++)
  {
    // std::vector to non-pod array
    size_t shmlen10 = sizeof((*shm_ptr_).var_struct_varlen_array[i1].layout.dim) / sizeof(MultiArrayDimension);
    size_t msglen10 = std::min((*msg_ptr).var_struct_varlen_array[i1].layout.dim.size(), shm_len_dim);
    for (int i10 = 0; i10 < msglen10; i10++)
    {
      // std::string to char[]
      std::snprintf((*shm_ptr_).var_struct_varlen_array[i1].layout.dim[i10].label, sizeof((*shm_ptr_).var_struct_varlen_array[i1].layout.dim[i10].label), "%s", (*msg_ptr).var_struct_varlen_array[i1].layout.dim[i10].label.c_str());
      (*shm_ptr_).var_struct_varlen_array[i1].layout.dim[i10].size = (*msg_ptr).var_struct_varlen_array[i1].layout.dim[i10].size;
      (*shm_ptr_).var_struct_varlen_array[i1].layout.dim[i10].stride = (*msg_ptr).var_struct_varlen_array[i1].layout.dim[i10].stride;
    }
    zeroUnsentElements((*shm_ptr_).var_struct_varlen_array[i1].layout.dim, msg_len_dim, shm_len_dim);
    (*shm_ptr_).var_struct_varlen_array[i1].layout.data_offset = (*msg_ptr).var_struct_varlen_array[i1].layout.data_offset;
    // std::vector to pod array
    size_t const shmlen11 = sizeof((*shm_ptr_).var_struct_varlen_array[i1].data) / sizeof(int8_t);
    size_t msglen11 = std::min((*msg_ptr).var_struct_varlen_array[i1].data.size(), shm_len_data);
    std::memcpy((*shm_ptr_).var_struct_varlen_array[i1].data, (*msg_ptr).var_struct_varlen_array[i1].data.data(), sizeof(int8_t) * msg_len_data);
    zeroUnsentElements((*shm_ptr_).var_struct_varlen_array[i1].data, msg_len_data, shm_len_data);
  }
  zeroUnsentElements((*shm_ptr_).var_struct_varlen_array, msg_len_var_struct_varlen_array, shm_len_var_struct_varlen_array);
}
template<> void RobinPublisher<double, std_msgs::Float64>::read()
{
  msg_.data = (*shm_ptr_);
}
template<> void RobinPublisher<AccelStamped, geometry_msgs::AccelStamped>::read()
{
  msg_.header.seq = (*shm_ptr_).header.seq;
  // pod to ros_type
  memcpy(&(msg_.header.stamp), &((*shm_ptr_).header.stamp), sizeof(msg_.header.stamp));
  // char[] to std::string
  msg_.header.frame_id = (*shm_ptr_).header.frame_id;
  msg_.accel.linear.x = (*shm_ptr_).accel.linear.x;
  msg_.accel.linear.y = (*shm_ptr_).accel.linear.y;
  msg_.accel.linear.z = (*shm_ptr_).accel.linear.z;
  msg_.accel.angular.x = (*shm_ptr_).accel.angular.x;
  msg_.accel.angular.y = (*shm_ptr_).accel.angular.y;
  msg_.accel.angular.z = (*shm_ptr_).accel.angular.z;
}
template<> void RobinPublisher<TestStruct, robin::TestStruct>::read()
{
  msg_.var_bool = (*shm_ptr_).var_bool;
  msg_.var_byte = (*shm_ptr_).var_byte;
  msg_.var_int16 = (*shm_ptr_).var_int16;
  msg_.var_uint64 = (*shm_ptr_).var_uint64;
  msg_.var_float32 = (*shm_ptr_).var_float32;
  msg_.var_float64 = (*shm_ptr_).var_float64;
  // char[] to std::string
  msg_.var_string = (*shm_ptr_).var_string;
  msg_.var_pose.position.x = (*shm_ptr_).var_pose.position.x;
  msg_.var_pose.position.y = (*shm_ptr_).var_pose.position.y;
  msg_.var_pose.position.z = (*shm_ptr_).var_pose.position.z;
  msg_.var_pose.orientation.x = (*shm_ptr_).var_pose.orientation.x;
  msg_.var_pose.orientation.y = (*shm_ptr_).var_pose.orientation.y;
  msg_.var_pose.orientation.z = (*shm_ptr_).var_pose.orientation.z;
  msg_.var_pose.orientation.w = (*shm_ptr_).var_pose.orientation.w;
  // non-pod array to boost::array
  size_t shm_len_var_struct_array_62409610 = sizeof((*shm_ptr_).var_struct_array) / sizeof(ByteMultiArray);
  for (int i_var_struct_array_62409610 = 0; i_var_struct_array_62409610 < shm_len_var_struct_array_62409610; i_var_struct_array_62409610++)
  {
    // non-pod array to std::vector
    size_t shm_len_dim_62409638 = sizeof((*shm_ptr_).var_struct_array[i_None_62409610].layout.dim) / sizeof(MultiArrayDimension);
    msg_.var_struct_array[i_None_62409610].layout.dim.resize(shm_len_dim);  //TODO execute only once, eg. in constructor
    for (int i_dim_62409638 = 0; i_dim_62409638 < shm_len_dim_62409638; i_dim_62409638++)
    {
      // char[] to std::string
      msg_.var_struct_array[i_None_62409610].layout.dim[i_None_62409638].label = (*shm_ptr_).var_struct_array[i_None_62409610].layout.dim[i_None_62409638].label;
      msg_.var_struct_array[i_None_62409610].layout.dim[i_None_62409638].size = (*shm_ptr_).var_struct_array[i_None_62409610].layout.dim[i_None_62409638].size;
      msg_.var_struct_array[i_None_62409610].layout.dim[i_None_62409638].stride = (*shm_ptr_).var_struct_array[i_None_62409610].layout.dim[i_None_62409638].stride;
    }
    msg_.var_struct_array[i_None_62409610].layout.data_offset = (*shm_ptr_).var_struct_array[i_None_62409610].layout.data_offset;
    // pod array to std::vector
    size_t const shm_len_data_62409709 = sizeof((*shm_ptr_).var_struct_array[i_None_62409610].data) / sizeof(int8_t);
    msg_.var_struct_array[i_None_62409610].data.assign((*shm_ptr_).var_struct_array[i_None_62409610].data, (*shm_ptr_).var_struct_array[i_None_62409610].data + shm_len_data);
  }
  // non-pod array to std::vector
  size_t shm_len_var_struct_varlen_array_62409741 = sizeof((*shm_ptr_).var_struct_varlen_array) / sizeof(ByteMultiArray);
  msg_.var_struct_varlen_array.resize(shm_len_var_struct_varlen_array);  //TODO execute only once, eg. in constructor
  for (int i_var_struct_varlen_array_62409741 = 0; i_var_struct_varlen_array_62409741 < shm_len_var_struct_varlen_array_62409741; i_var_struct_varlen_array_62409741++)
  {
    // non-pod array to std::vector
    size_t shm_len_dim_62409761 = sizeof((*shm_ptr_).var_struct_varlen_array[i_None_62409741].layout.dim) / sizeof(MultiArrayDimension);
    msg_.var_struct_varlen_array[i_None_62409741].layout.dim.resize(shm_len_dim);  //TODO execute only once, eg. in constructor
    for (int i_dim_62409761 = 0; i_dim_62409761 < shm_len_dim_62409761; i_dim_62409761++)
    {
      // char[] to std::string
      msg_.var_struct_varlen_array[i_None_62409741].layout.dim[i_None_62409761].label = (*shm_ptr_).var_struct_varlen_array[i_None_62409741].layout.dim[i_None_62409761].label;
      msg_.var_struct_varlen_array[i_None_62409741].layout.dim[i_None_62409761].size = (*shm_ptr_).var_struct_varlen_array[i_None_62409741].layout.dim[i_None_62409761].size;
      msg_.var_struct_varlen_array[i_None_62409741].layout.dim[i_None_62409761].stride = (*shm_ptr_).var_struct_varlen_array[i_None_62409741].layout.dim[i_None_62409761].stride;
    }
    msg_.var_struct_varlen_array[i_None_62409741].layout.data_offset = (*shm_ptr_).var_struct_varlen_array[i_None_62409741].layout.data_offset;
    // pod array to std::vector
    size_t const shm_len_data_62409828 = sizeof((*shm_ptr_).var_struct_varlen_array[i_None_62409741].data) / sizeof(int8_t);
    msg_.var_struct_varlen_array[i_None_62409741].data.assign((*shm_ptr_).var_struct_varlen_array[i_None_62409741].data, (*shm_ptr_).var_struct_varlen_array[i_None_62409741].data + shm_len_data);
  }
}
template class RobinSubscriber<double, std_msgs::Float64>;
template class RobinSubscriber<AccelStamped, geometry_msgs::AccelStamped>;
template class RobinSubscriber<TestStruct, robin::TestStruct>;
template class RobinPublisher<double, std_msgs::Float64>;
template class RobinPublisher<AccelStamped, geometry_msgs::AccelStamped>;
template class RobinPublisher<TestStruct, robin::TestStruct>;
