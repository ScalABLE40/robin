#include "robin.cpp"
#include "robin_publisher.cpp"
#include "robin_subscriber.cpp"
#include "robin/structs.h"
#include "robin/TestStruct.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
template class Robin<bool, std_msgs::Bool>;
template class Robin<double, std_msgs::Float64>;
template class Robin<TestStruct, robin::TestStruct>;
template class RobinPublisher<bool, std_msgs::Bool>;
template class RobinPublisher<double, std_msgs::Float64>;
template class RobinPublisher<TestStruct, robin::TestStruct>;
template class RobinSubscriber<bool, std_msgs::Bool>;
template class RobinSubscriber<double, std_msgs::Float64>;
template class RobinSubscriber<TestStruct, robin::TestStruct>;
