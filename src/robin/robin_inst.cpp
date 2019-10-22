#include "robin.cpp"
#include "robin_reader.cpp"
#include "robin_writer.cpp"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
template class Robin<bool, std_msgs::Bool>;
template class Robin<double, std_msgs::Float64>;
template class RobinReader<bool, std_msgs::Bool>;
template class RobinReader<double, std_msgs::Float64>;
template class RobinWriter<bool, std_msgs::Bool>;
template class RobinWriter<double, std_msgs::Float64>;
