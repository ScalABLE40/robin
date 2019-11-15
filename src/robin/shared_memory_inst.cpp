#include "shared_memory.cpp"
#include "robin/structs.h"
#include "robin/TestStruct_foo.h"
#include "std_msgs/Float64.h"
template class SharedMemory<double, std_msgs::Float64>;
template class SharedMemory<TestStruct_foo, robin::TestStruct_foo>;
