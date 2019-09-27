#include <robin/shared_memory.h>
#include <gtest/gtest.h>
class SharedMemoryFixture : public ::testing::Test {
protected:
  std::string shm_name_ = "test_shared_memory";
  std::unique_ptr<SharedMemory> shared_memory_ = std::unique_ptr<SharedMemory>(new SharedMemory(shm_name_));
  int shm_fd_ = -1;
  double *shm_ptr_ = NULL;
  double shm_val_ = 0;
  void SetUp() override
  {
    ASSERT_TRUE(sharedMemoryIsClosed()) << "Shared memory with same name already exists.";
  }
  int openShmFileDescriptor()
  {
    errno = 0;
    shm_fd_ = shm_open(shm_name_.c_str(), O_RDWR, 0600);
    return shm_fd_;
  } 
  bool sharedMemoryIsOpen()
  {
    return (openShmFileDescriptor() != -1);
  }
  bool sharedMemoryIsClosed()
  {
    return (openShmFileDescriptor() == -1 && errno == ENOENT);
  }
  double *openSharedMemory()
  {
    openShmFileDescriptor();
    EXPECT_NE(shm_fd_, -1) << "Failed to open shared memory.";
    errno = 0;
    EXPECT_NE(ftruncate(shm_fd_, sizeof(*shm_ptr_)), -1)
      << "Failed to truncate shared memory. errno " << errno << ": " << strerror(errno);
    errno = 0;
    shm_ptr_ = (double *)mmap(0, sizeof(*shm_ptr_), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    EXPECT_NE(shm_ptr_, MAP_FAILED) << "Failed to map shared memory. errno " << errno << ": " << strerror(errno);
    return shm_ptr_;
  }
};
// TEST_F(SharedMemoryFixture, construct)  //TODO
// { 
//   // EXPECT_EQ(typeid(std::result_of(SharedMemory::SharedMemory(shm_name_))::type), typeid(SharedMemory));
//   // ::testing::StaticAssertTypeEq<SharedMemory, std::result_of(SharedMemory::SharedMemory(shm_name_))>();
//   // EXPECT_EQ(typeid(*(new SharedMemory::SharedMemory(shm_name_))), typeid(SharedMemory));
//   // shared_memory_ = std::make_unique<SharedMemory>(shm_name_);
//   // EXPECT_EQ(typeid(*shared_memory_), typeid(SharedMemory));
// }
TEST_F(SharedMemoryFixture, open)
{
  EXPECT_TRUE(sharedMemoryIsClosed()) << "Shared memory opened prematurely.";
  shared_memory_->open();
  EXPECT_TRUE(sharedMemoryIsOpen()) << "Failed to open shared memory. errno " << errno << ": " << strerror(errno);
}
TEST_F(SharedMemoryFixture, close)
{
  shared_memory_->open();
  EXPECT_TRUE(sharedMemoryIsOpen()) << "Failed to open shared memory. errno " << errno << ": " << strerror(errno);
  shared_memory_->close();
  EXPECT_TRUE(sharedMemoryIsClosed()) << "Failed to close shared memory.";
}
TEST_F(SharedMemoryFixture, destruct)
{
  shared_memory_->open();
  EXPECT_TRUE(sharedMemoryIsOpen()) << "Failed to open shared memory. errno " << errno << ": " << strerror(errno);
  shared_memory_.reset();
  EXPECT_TRUE(sharedMemoryIsClosed()) << "Failed to close shared memory.";
}
TEST_F(SharedMemoryFixture, isOpen)
{
  EXPECT_FALSE(shared_memory_->isOpen());
  shared_memory_->open();
  EXPECT_TRUE(shared_memory_->isOpen());
  shared_memory_->close();
  EXPECT_FALSE(shared_memory_->isOpen());
}
TEST_F(SharedMemoryFixture, read)
{
  shared_memory_->open();
  openSharedMemory();
  *shm_ptr_ = 123.456;
  EXPECT_EQ(shared_memory_->read(), 123.456) << "Shared memory value mismatch.";
}
TEST_F(SharedMemoryFixture, write)
{
  shared_memory_->open(true);
  shared_memory_->write(654.321);
  openSharedMemory();
  EXPECT_EQ(*shm_ptr_, 654.321) << "Shared memory value mismatch.";
}
int main(int argc, char **argv)
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
   ros::console::notifyLoggerLevelsChanged();
  }
  ::testing::InitGoogleTest(&argc, argv);
  // ros::init(argc, argv, "test_shared_memory");
  // ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
