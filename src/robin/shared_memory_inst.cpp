#include "shared_memory.cpp"
#include "robin/structs.h"
#include "robin/TestStruct_foo.h"
#include "std_msgs/Float64.h"
template<> void SharedMemory<double, std_msgs::Float64>::read(std_msgs::Float64 *msg_ptr)
{
  msg_ptr->data = *shm_ptr_;
  printf("double read\n");
};
template<> void SharedMemory<double, std_msgs::Float64>::write(std_msgs::Float64 const *msg_ptr)
{
  *shm_ptr_ = msg_ptr->data;
  printf("double written\n");
};
template<> void SharedMemory<TestStruct_foo, robin::TestStruct_foo>::read(robin::TestStruct_foo *msg_ptr)
{
  msg_ptr->var_bool_1 = shm_ptr_->var_bool_1;
  msg_ptr->var_struct_1.var_float64_1 = shm_ptr_->var_struct_1.var_float64_1;
  msg_ptr->var_struct_1.var_float64_2 = shm_ptr_->var_struct_1.var_float64_2;
  msg_ptr->var_struct_2.var_float64_1 = shm_ptr_->var_struct_2.var_float64_1;
  msg_ptr->var_struct_2.var_float64_2 = shm_ptr_->var_struct_2.var_float64_2;
  printf("TestStruct_foo read\n");
};
template<> void SharedMemory<TestStruct_foo, robin::TestStruct_foo>::write(robin::TestStruct_foo const *msg_ptr)
{
  shm_ptr_->var_bool_1 = msg_ptr->var_bool_1;
  shm_ptr_->var_struct_1.var_float64_1 = msg_ptr->var_struct_1.var_float64_1;
  shm_ptr_->var_struct_1.var_float64_2 = msg_ptr->var_struct_1.var_float64_2;
  shm_ptr_->var_struct_2.var_float64_1 = msg_ptr->var_struct_2.var_float64_1;
  shm_ptr_->var_struct_2.var_float64_2 = msg_ptr->var_struct_2.var_float64_2;
  printf("TestStruct_foo written\n");
};
template class SharedMemory<double, std_msgs::Float64>;
template class SharedMemory<TestStruct_foo, robin::TestStruct_foo>;
