#ifndef STRUCTS_H
#define STRUCTS_H
struct TestSubStruct
{
  double var_float64_1;
  double var_float64_2;
};
struct TestStruct_foo
{
  bool var_bool_1;
  TestSubStruct var_struct_1;
  TestSubStruct var_struct_2;
};
#endif
