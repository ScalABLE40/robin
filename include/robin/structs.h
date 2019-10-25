#ifndef STRUCTS_H
#define STRUCTS_H
// #include <cstdint>
struct TestSubStruct
{
    double var_float64_1 = 1.23456789;
    double var_float64_2 = -10.987654321;
};
struct TestStruct
{
	bool var_bool = true;
	uint16_t var_uint16 = 54321;
	TestSubStruct var_struct;
};
#endif
