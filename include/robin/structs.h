#ifndef ROBIN_STRUCTS_H
#define ROBIN_STRUCTS_H
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
struct MultiArrayDimension
{
    char label[80];
    uint32_t size;
    uint32_t stride;
};
struct MultiArrayLayout
{
    MultiArrayDimension dim[20];
    uint32_t data_offset;
};
struct ByteMultiArray
{
    MultiArrayLayout layout;
    uint8_t data[20];
};
struct TestStruct_arr
{
  char var_string[30];
  double var_float64_array[10];
  ByteMultiArray var_bytemultiarray;
};
#endif
