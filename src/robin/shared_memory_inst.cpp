#include "shared_memory.cpp"
#include "robin/structs.h"
template class SharedMemory<bool>;
template class SharedMemory<double>;
template class SharedMemory<TestStruct>;
