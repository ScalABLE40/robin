#include "robin.cpp"
#include "robin_publisher.cpp"
#include "robin_subscriber.cpp"
#include "robin/structs.h"
#include "robin/TestStruct.h"
#include "std_msgs/Float64.h"
template class Robin<double, std_msgs::Float64>;
template class Robin<TestStruct, robin::TestStruct>;
template class RobinPublisher<double, std_msgs::Float64>;
template class RobinPublisher<TestStruct, robin::TestStruct>;
template class RobinSubscriber<double, std_msgs::Float64>;
template class RobinSubscriber<TestStruct, robin::TestStruct>;
