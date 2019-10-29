#ifndef STRUCTS_H
#define STRUCTS_H
struct TestSubStruct
{
  double var_float64_1;
  double var_float64_2;
};
struct TestStruct
{
  bool var_bool;
  uint16_t var_uint16;
  TestSubStruct var_struct;
};
#endif
