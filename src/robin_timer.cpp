#include <chrono>
#include "robin/shared_memory.h"
const int DEF_REPS = 1000;
int main(int argc, char **argv)
{
  double data_in;
  SharedMemory<double> shared_memory_out("double_to_codesys");
  SharedMemory<double> shared_memory_in("double_to_ros");
  shared_memory_out.open();
  shared_memory_in.open();
  int reps = (argc == 2) ? atoi(argv[1]) : DEF_REPS;
  printf("reps: %d\n", reps);
  auto t1 = std::chrono::high_resolution_clock::now();
  for (double d = 0.123456789; d < reps; d++)
  {
    shared_memory_out.write(&d);
    do
    {
      shared_memory_in.read(&data_in);
    } while (memcmp(&data_in, &d, sizeof(d)) != 0);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  printf("time: %ld\n", duration / reps);
  return 0;
}
