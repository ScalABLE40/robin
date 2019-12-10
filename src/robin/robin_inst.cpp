#include <cstdio>   // for std::snprintf()
#include <cstring>  // for std::memcpy()
#include "robin_publisher.cpp"
#include "robin_subscriber.cpp"
#include "robin/structs.h"
#include "std_msgs/Float64.h"
template class RobinSubscriber<double, std_msgs::Float64>;
template class RobinPublisher<double, std_msgs::Float64>;
