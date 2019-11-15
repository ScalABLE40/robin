#include "robin.cpp"
#include "robin_publisher.cpp"
#include "robin_subscriber.cpp"
#include "robin/structs.h"
#include "robin/TestStruct_foo.h"
#include "std_msgs/Float64.h"
template class RobinPublisher<double, std_msgs::Float64>;
template class RobinPublisher<TestStruct_foo, robin::TestStruct_foo>;
template class RobinSubscriber<double, std_msgs::Float64>;
template class RobinSubscriber<TestStruct_foo, robin::TestStruct_foo>;
// template<> class RobinPublisher<double, std_msgs::Float64>
// {
//     void read()
//     {
//         this->shared_memory_.read(&msg_);  //TRY without 'this->'
//     }
// };
// template<> class RobinSubscriber<double, std_msgs::Float64>
// {
//     void write(std_msgs::Float64 *msg)
//     {
//         this->shared_memory_.write(msg);  //TRY without 'this->'
//     }
// };
