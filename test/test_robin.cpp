#include "robin/robin.h"
#include <gtest/gtest.h>
class SemaphoreTest : public ::testing::Test {
protected:
  char *semaphore_name_ = "test_semaphore";
  sem_t *semaphore_check_;
  Semaphore *semaphore_;
  int semaphore_value_ = -1;
  void SetUp() override
  {
    // check same name semaphore does not exist
    ASSERT_TRUE(openSemaphore() == SEM_FAILED && errno == ENOENT) << "Semaphore with same name already exists.";
  }
  sem_t *openSemaphore(void)
  {
    errno = 0;
    semaphore_check_ = sem_open(semaphore_name_, 0);
    return semaphore_check_;
  }
  void TearDown() override
  {
    // unlink semaphore and check semaphore unlinked
    sem_close(semaphore_check_);
    delete semaphore_;
    ASSERT_TRUE(openSemaphore() == SEM_FAILED && errno == ENOENT) << "Failed to unlink semaphore.";
  }
};
TEST_F(SemaphoreTest, semaphoreHandlingOk)  // WIP test isOpen(), open(), close()
{
    // create Semaphore object
    semaphore_ = new Semaphore(semaphore_name_);
    // TODO? check object was created
    // open semaphore and check it openeed
    ASSERT_FALSE(semaphore_->isOpen());
    semaphore_->open();
    ASSERT_TRUE(semaphore_->isOpen());
    ASSERT_NE(openSemaphore(), SEM_FAILED) << "Failed to open semaphore. errno " << errno << ": " << strerror(errno);
    // check semaphore value equals 1
    EXPECT_EQ(sem_getvalue(semaphore_check_, &semaphore_value_), 0) << "Failed to get semaphore value.";
    EXPECT_EQ(semaphore_value_, 1) << "Semaphore opened with wrong value.";
    // wait semaphore and check value equals 0
    semaphore_->wait();
    EXPECT_EQ(sem_getvalue(semaphore_check_, &semaphore_value_), 0) << "Failed to get semaphore value.";
    EXPECT_EQ(semaphore_value_, 0) << "Semaphore value unchanged after wait.";
    // post semaphore and check value equals 1
    semaphore_->post();
    EXPECT_EQ(sem_getvalue(semaphore_check_, &semaphore_value_), 0) << "Failed to get semaphore value.";
    EXPECT_EQ(semaphore_value_, 1) << "Semaphore value unchanged after post.";
}
int main(int argc, char **argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
   ros::console::notifyLoggerLevelsChanged();
  }
  ::testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "test_robin");
  // ros::NodeHandle nh;
  ROS_DEBUG("Hello, test world!");
  return RUN_ALL_TESTS();
}
