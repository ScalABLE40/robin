#include "robin/robin.h"
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
bool bool_read_ = false;
bool bool_was_read_ = false;
class RobinFixture : public ::testing::Test {
protected:
  std::string name_ = "test_robin";
  std::unique_ptr<Robin> robin_ = std::unique_ptr<Robin>(new Robin(name_));
  const uint32_t queue_size_ = 100;
  void SetUp() override
  {
    ASSERT_TRUE(robinIsClosed()) << "Robin with same name is already open.";
  }
  bool robinIsClosed()
  {
    return semaphoreIsClosed() && sharedMemoryIsClosed() && !topicExists();
  }
  bool semaphoreIsClosed()
  {
    errno = 0;
    return (sem_open(name_.c_str(), 0) == SEM_FAILED && errno == ENOENT);
  }
  bool sharedMemoryIsClosed()
  {
    errno = 0;
    return (shm_open(name_.c_str(), O_RDWR, 0600) == -1 && errno == ENOENT);
  }
  bool topicExists()
  {
    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    for (ros::master::TopicInfo topic : topics)
    {
      if (topic.name.compare("/" + name_) == 0)
      {
        return true;
      }
    }
    return false;
  }
  bool robinIsOpen()
  {
    return semaphoreIsOpen() && sharedMemoryIsOpen() && topicExists();
  }
  bool semaphoreIsOpen()
  {
    errno = 0;
    return (sem_open(name_.c_str(), 0) != SEM_FAILED);
  }
  bool sharedMemoryIsOpen()
  {
    errno = 0;
    return (shm_open(name_.c_str(), O_RDWR, 0600) != -1);
  }
public:
  static void subscriberCallback(const std_msgs::Bool::ConstPtr& msg)  //TODO check static, public, etc. necessary
  {
    bool_read_ = msg->data;
    bool_was_read_ = true;
  }
};
// TEST_F(RobinFixture, construct)  //TODO
// { 
//   // EXPECT_EQ(typeid(std::result_of(Robin::Robin(shm_name_))::type), typeid(Robin));
//   // ::testing::StaticAssertTypeEq<Robin, std::result_of(Robin::Robin(shm_name_))>();
//   // EXPECT_EQ(typeid(*(new Robin::Robin(shm_name_))), typeid(Robin));
//   // robin_ = std::make_unique<Robin>(shm_name_);
//   // EXPECT_EQ(typeid(*robin_), typeid(Robin));
// }
TEST_F(RobinFixture, open)
{
  EXPECT_TRUE(robinIsClosed()) << "Robin opened prematurely.";
  robin_->open();
  EXPECT_TRUE(robinIsOpen()) << "Failed to open robin. errno " << errno << ": " << strerror(errno);
}
TEST_F(RobinFixture, close)
{
  robin_->open();
  EXPECT_TRUE(robinIsOpen()) << "Failed to open robin. errno " << errno << ": " << strerror(errno);
  robin_->close();
  EXPECT_TRUE(robinIsClosed()) << "Failed to close robin.";
}
TEST_F(RobinFixture, destruct)
{
  robin_->open();
  EXPECT_TRUE(robinIsOpen()) << "Failed to open robin. errno " << errno << ": " << strerror(errno);
  robin_.reset();
  EXPECT_TRUE(robinIsClosed()) << "Failed to close robin.";
}
TEST_F(RobinFixture, multipleOpenClose)
{
  robin_->open();
  EXPECT_TRUE(robinIsOpen()) << "Failed to open robin. errno " << errno << ": " << strerror(errno);
  robin_->close();
  EXPECT_TRUE(robinIsClosed()) << "Failed to close robin.";
  robin_->open();
  EXPECT_TRUE(robinIsOpen()) << "Failed to open robin. errno " << errno << ": " << strerror(errno);
  robin_->close();
  EXPECT_TRUE(robinIsClosed()) << "Failed to close robin.";
}
TEST_F(RobinFixture, isOpen)
{
  EXPECT_FALSE(robin_->isOpen());
  robin_->open();
  EXPECT_TRUE(robin_->isOpen());
  robin_->close();
  EXPECT_FALSE(robin_->isOpen());
}
TEST_F(RobinFixture, read)
{
  robin_->open(READ);
  SharedMemory shared_memory(name_);
  shared_memory.open(WRITE);
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<std_msgs::Bool>(name_, queue_size_, &RobinFixture::subscriberCallback);
  bool values[] = {true, false, true, true, false, false, true};
  for (int i = 0; i < sizeof(values) / sizeof(bool); i++)
  {
    shared_memory.write(values[i]);
    while (!bool_was_read_)
    {
      robin_->read();
      std::this_thread::sleep_for(std::chrono::seconds(1));
      ros::spinOnce();
    }
    EXPECT_EQ(bool_read_, values[i]) << "Robin value mismatch.";
    bool_was_read_ = false;
  }
}
TEST_F(RobinFixture, write)
{
  robin_->open(WRITE);
  SharedMemory shared_memory(name_);
  shared_memory.open(READ);
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Bool>(name_, queue_size_);
  std_msgs::Bool msg;
  bool values[] = {true, false, true, true, false, false, true};
  for (int i = 0; i < sizeof(values) / sizeof(bool); i++)
  {
    msg.data = values[i];
    pub.publish(msg);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ros::spinOnce();
    EXPECT_EQ(shared_memory.read(), values[i]) << "Robin value mismatch.";
  }
}
int main(int argc, char **argv)
{
  // if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  // {
  //   ros::console::notifyLoggerLevelsChanged();
  // }
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_robin");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
