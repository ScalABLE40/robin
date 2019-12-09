#ifndef ROBIN_STRUCTS_H
#define ROBIN_STRUCTS_H
#include <cstdint>
struct Time
{
  uint32_t secs;
  uint32_t nsecs;
};
struct Header
{
  uint32_t seq;
  Time stamp;
  char frame_id[81];
};
struct Vector3
{
  double x;
  double y;
  double z;
};
struct Accel
{
  Vector3 linear;
  Vector3 angular;
};
struct AccelStamped
{
  Header header;
  Accel accel;
};
struct Point
{
  double x;
  double y;
  double z;
};
struct Quaternion
{
  double x;
  double y;
  double z;
  double w;
};
struct Pose
{
  Point position;
  Quaternion orientation;
};
struct MultiArrayDimension
{
  char label[81];
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
  int8_t data[20];
};
struct TestStruct
{
  uint8_t var_bool;
  int8_t var_byte;
  int16_t var_int16;
  uint64_t var_uint64;
  float var_float32;
  double var_float64;
  char var_string[81];
  Pose var_pose;
  ByteMultiArray var_struct_array[10];
  ByteMultiArray var_struct_varlen_array[10];
};
#endif
